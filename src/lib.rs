#![doc = include_str!("../README.md")]
#![warn(
    missing_debug_implementations,
    missing_copy_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    missing_docs
)]

const PRECISION: f32 = 1000.0;

#[cfg(feature = "stats")]
use std::{cell::Cell, time::Instant};
use std::{cmp::Ordering, fmt::{self, Debug, Display}, hash::Hash};

use bvh2d::{
    aabb::{Bounded, AABB},
    bvh2d::BVH2d,
};
use geo::{BoundingRect, Coord, Intersects, LinesIter};
use glam::{Vec2, vec2};

use helpers::Vec2Helper;
use instance::{EdgeSide, InstanceStep};
use log::{error, warn};
#[cfg(feature = "tracing")]
use tracing::instrument;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(feature = "async")]
mod async_helpers;
mod helpers;
mod input;
mod instance;
mod merger;
mod primitives;

#[cfg(feature = "async")]
pub use async_helpers::FuturePath;
pub use input::polyanya_file::PolyanyaFile;
pub use input::triangulation::{
    Coord as GeoCoord, LineString as GeoLineString, MultiPolygon as GeoMultiPolygon,
    Polygon as GeoPolygon, PolygonMeshSetOperation, Triangulation,
};
pub use input::trimesh::Trimesh;
pub use primitives::{Polygon, Vertex};

use crate::instance::SearchInstance;

/// A path between two points.
#[derive(Debug, PartialEq)]
pub struct Path {
    /// Length of the path.
    pub length: f32,
    /// Coordinates for each step of the path. The destination is the last step.
    pub path: Vec<Vec2>,
}
impl Path {
    /// Creates a new object with the target as single waypoint.
    pub fn new_direct(from: Vec2, to: Vec2) -> Self {
        Self {
            length: from.distance(to),
            path: vec![to]
        }
    }
}

/// A navigation mesh
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Mesh {
    /// List of `Vertex` in this mesh
    pub vertices: Vec<Vertex>,
    /// List of `Polygons` in this mesh
    pub polygons: Vec<Polygon>,
    baked_polygons: Option<BVH2d>,
    islands: Option<Vec<usize>>,
    delta: f32,
    #[cfg(feature = "stats")]
    pub(crate) scenarios: Cell<u32>,
}

impl Default for Mesh {
    fn default() -> Self {
        Self {
            delta: 0.1,
            vertices: Default::default(),
            polygons: Default::default(),
            baked_polygons: Default::default(),
            islands: Default::default(),
            #[cfg(feature = "stats")]
            scenarios: Cell::new(0),
        }
    }
}

struct Root(Vec2);

impl PartialEq for Root {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl Eq for Root {}

impl Hash for Root {
    #[inline(always)]
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        ((self.0.x * PRECISION) as i32).hash(state);
        ((self.0.y * PRECISION) as i32).hash(state);
        state.finish();
    }
}

struct BoundedPolygon {
    aabb: (Vec2, Vec2),
}

impl Bounded for BoundedPolygon {
    fn aabb(&self) -> AABB {
        AABB::with_bounds(self.aabb.0, self.aabb.1)
    }
}

/// TODO
#[derive(Debug, Copy, Clone)]
pub enum PathApproxResultEnum {
    /// The start- and end-point are within the same mesh islands and a path was found.
    ValidPath,
    /// The start- and end-points are within different mesh islands.
    /// There cannot exist a valid path between the two points.
    /// A path with a modified end-point was given that is closest to the original end-point that is on within the same
    /// mesh island of the start point.
    NoPath,
    /// The end-point was inside the mesh, but the start-point was outside the mesh.
    /// The end-point might be modified that the agent can get to the closest point to the original end-point within the
    /// same mesh island that the point of the intersection of the mesh and the line between start- and end-point has.
    ///
    /// This might be a bug, but if the mesh gets smaller and agents are not moved, this situation may occur.
    StartOutside,
    /// The start- and end-point were outside the mesh.
    /// A direct path that ignores the mesh-bounds is given.
    /// This is a bug, but might happen at some point during the game and then the agents should not get stuck.
    BothOutside,
    /// The start-point was inside the mesh, but the end-point was outside the mesh.
    /// The end-point was modified such that the agent can get to the closest point to the original end-point within the
    /// same mesh island that the start-point is in.
    ///
    /// This can happen regularly, e.g., if the user clicks outside the mesh.
    /// The intention of the user is usually to get the agent to a point close to the target point and not for the agent
    /// to not move.
    EndOutside,
    /// The start- and end-point are within the same mesh islands, but a path could not be found and deadlock prevention
    /// kicked in.
    /// A direct path that ignores the mesh-bounds is given.
    /// This is a bug, but does not seem to happen frequently enough to be important.
    InfinitePrevention,
    /// The pathfinding algorithm has a bug, instead of crashing the application, this result was returned.
    /// A direct path that ignores the mesh-bounds is given.
    BugCrashPrevention,
}
impl PathApproxResultEnum {
    /// True if the target might have been modified.
    pub fn is_end_modified(&self) -> bool {
        match self {
            PathApproxResultEnum::ValidPath => {false}
            PathApproxResultEnum::NoPath => {true}
            PathApproxResultEnum::StartOutside => {true}
            PathApproxResultEnum::BothOutside => {false}
            PathApproxResultEnum::EndOutside => {true}
            PathApproxResultEnum::InfinitePrevention => {false}
            PathApproxResultEnum::BugCrashPrevention => {false}
        }
    }
    /// True if the situation is expected to occur.
    pub fn is_valid_situation(&self) -> bool {
        match self {
            PathApproxResultEnum::ValidPath => {true}
            PathApproxResultEnum::NoPath => {true}
            PathApproxResultEnum::StartOutside => {false}
            PathApproxResultEnum::BothOutside => {false}
            PathApproxResultEnum::EndOutside => {true}
            PathApproxResultEnum::InfinitePrevention => {false}
            PathApproxResultEnum::BugCrashPrevention => {false}
        }
    }
}
/// TODO
#[derive(Debug)]
pub struct PathApproxResult {
    /// TODO
    pub path: Path,
    /// TODO
    pub status: PathApproxResultEnum,
}
impl PathApproxResult {
    /// Gets the last element of the path: the destination.
    pub fn get_end(&self) -> &Vec2 {
        self.path.path.last().expect("expected path to never be empty")
    }
}

impl Mesh {
    /// Remove pre-computed optimizations from the mesh. Call this if you modified the [`Mesh`].
    #[inline]
    pub fn unbake(&mut self) {
        self.baked_polygons = None;
        self.islands = None;
    }

    /// Pre-compute optimizations on the mesh
    ///
    /// Optimisations available are:
    /// - [`Self::bake_polygon_finder`]
    /// - [`Self::bake_islands_detection`]
    pub fn bake(&mut self) {
        self.bake_polygon_finder();
        self.bake_islands_detection()
    }

    /// Speed up bailing out if two points are not reachable.
    ///
    /// This is useful if there are isolated zones in the mesh, and you need to check for a path
    /// between them.
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    pub fn bake_islands_detection(&mut self) {
        let mut islands = vec![usize::MAX; self.polygons.len()];
        while let Some((root, _)) = islands
            .iter()
            .enumerate()
            .find(|(_, island)| **island == usize::MAX)
        {
            let mut to_visit = Vec::new();
            to_visit.push(root);
            while let Some(next) = to_visit.pop() {
                if islands[next] == usize::MAX {
                    let polygon = &mut self.polygons[next];
                    islands[next] = root;
                    to_visit.extend(
                        polygon
                            .vertices
                            .iter()
                            .flat_map(|v| self.vertices[*v as usize].polygons.iter())
                            .filter_map(|i| if *i != -1 { Some(*i as usize) } else { None }),
                    );
                }
            }
        }
        self.islands = Some(islands);
    }

    /// Speed up finding which polygon, if any, contains a point in the mesh.
    ///
    /// Uses a BVH. This is useful at the start of the pathfinding, to get the containing polygons
    /// for the start and end point. It can also be used through [`Self::point_in_mesh`] to check
    /// if a point is in the mesh.
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    pub fn bake_polygon_finder(&mut self) {
        let bounded_polygons = self
            .polygons
            .iter_mut()
            .map(|polygon| BoundedPolygon {
                aabb: polygon.vertices.iter().fold(
                    (Vec2::new(f32::MAX, f32::MAX), Vec2::ZERO),
                    |mut aabb, v| {
                        if let Some(v) = self.vertices.get(*v as usize) {
                            if v.coords.x < aabb.0.x {
                                aabb.0.x = v.coords.x;
                            }
                            if v.coords.y < aabb.0.y {
                                aabb.0.y = v.coords.y;
                            }
                            if v.coords.x > aabb.1.x {
                                aabb.1.x = v.coords.x;
                            }
                            if v.coords.y > aabb.1.y {
                                aabb.1.y = v.coords.y;
                            }
                        }
                        aabb
                    },
                ),
            })
            .collect::<Vec<_>>();

        self.baked_polygons = Some(
            BVH2d::build(&bounded_polygons)
                .expect("there should be polygons at this point in time"),
        );
    }

    /// Create a `Mesh` from a list of [`Vertex`] and [`Polygon`].
    pub fn new(vertices: Vec<Vertex>, polygons: Vec<Polygon>) -> Mesh {
        let mut mesh = Mesh {
            vertices,
            polygons,
            ..Default::default()
        };
        #[cfg(not(feature = "no-default-baking"))]
        mesh.bake();
        // just to not get a warning on the mut borrow. should be pretty much free anyway
        #[cfg(feature = "no-default-baking")]
        mesh.unbake();
        mesh
    }

    /// Compute a path between two points.
    ///
    /// This method returns a `Future`, to get the path in a blocking way use [`Self::path`].
    #[cfg(feature = "async")]
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    pub fn get_path(&self, from: Vec2, to: Vec2) -> FuturePath {
        FuturePath {
            from,
            to,
            mesh: self,
            instance: None,
            ending_polygon: -2,
        }
    }

    /// Compute a path between two points.
    ///
    /// This will be a [`Path`] if a path is found, or `None` if not.
    ///
    /// This method is blocking, to get the path in an async way use [`Self::get_path`].
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline(always)]
    pub fn path(&self, from: Vec2, to: Vec2) -> Option<Path> {
        #[cfg(feature = "stats")]
        let start = Instant::now();

        let starting_polygon_index = self.get_point_location(from);
        if starting_polygon_index == u32::MAX {
            return None;
        }
        let ending_polygon = self.get_point_location(to);
        if ending_polygon == u32::MAX {
            return None;
        }
        if let Some(islands) = self.islands.as_ref() {
            let start_island = islands.get(starting_polygon_index as usize);
            let end_island = islands.get(ending_polygon as usize);
            if start_island.is_some() && end_island.is_some() && start_island != end_island {
                return None;
            }
        }

        if starting_polygon_index == ending_polygon {
            #[cfg(feature = "stats")]
            {
                if self.scenarios.get() == 0 {
                    eprintln!(
                    "index;micros;successor_calls;generated;pushed;popped;pruned_post_pop;length",
                );
                }
                eprintln!(
                    "{};{};0;0;0;0;0;{}",
                    self.scenarios.get(),
                    start.elapsed().as_secs_f32() * 1_000_000.0,
                    from.distance(to),
                );
                self.scenarios.set(self.scenarios.get() + 1);
            }
            return Some(Path {
                length: from.distance(to),
                path: vec![to],
            });
        }

        let mut search_instance = SearchInstance::setup(
            self,
            (from, starting_polygon_index),
            (to, ending_polygon),
            #[cfg(feature = "stats")]
            start,
        );

        // Limit search to avoid an infinite loop.
        for _ in 0..self.polygons.len() * 1000 {
            match search_instance.next(true) {
                InstanceStep::Found(path) => return Some(path),
                InstanceStep::NotFound => return None,
                InstanceStep::Continue => (),
            }
        }

        error!("Search from {from} to {to} failed. Please check the mesh is valid as this should not happen.");
        None
    }

    /// Gets the first intersection with the mesh and uses it.
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline(always)]
    fn approx_path_fix_start(&self, _end_index: u32, _from: Vec2, _to: Vec2, intersections: &[Vec2]) -> (u32, Vec2) {
        if let Some(pos) = intersections.iter().next() {
            let starting_polygon_index = self.get_point_location_ignore_delta(*pos);
            if starting_polygon_index == u32::MAX {
                // if this triggers, this is probably because of floating point inaccuracies
                unreachable!("the start is not within mesh but the end should be, therefore the line segment SHOULD HAVE gotten at least one intermediate result that is WITHIN the mesh");
            }
            (starting_polygon_index, *pos)
        }
        else {
            // if this triggers, this is probably a logical error
            unreachable!("the start is not within mesh but the end should be, therefore the line segment SHOULD HAVE gotten at least one intermediate result");
        }
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline(always)]
    fn approx_path_fix_end(&self, start_index: u32, _from: Vec2, _to: Vec2, intersections: &Vec<Vec2>) -> (u32, Vec2) {
        let islands = self.islands.as_ref().expect("islands must exist");
        let start_island = islands.get(start_index as usize).expect("start point island must exist");

        for pos in intersections.iter().rev() {
            let poly_idx = self.get_point_location_ignore_delta(*pos);
            if poly_idx == u32::MAX {
                // if this triggers, this is probably because of floating point inaccuracies
                unreachable!("either the start or the end mus be within the mesh, therefore the line segment SHOULD HAVE gotten at least one intermediate result");
            }

            // if this triggers, this is probably because of floating point inaccuracies
            let end_island = islands.get(poly_idx as usize).expect("end point island must exist");
            if start_island != end_island {
                continue;
            }

            return (poly_idx, *pos)
        }
        // if this triggers, this is probably a logical error
        unreachable!("either the start or the end mus be within the mesh");
    }

    /// Compute a path between two points.
    /// This method is blocking.
    /// - If the starting point and the end point are within the mesh:
    ///   - a path within the mesh is returned.
    /// - If the starting point is not within the mesh:
    ///   - a straight path to the target point is returned.
    /// - If the starting point is within the mesh but the target point is not within the mesh:
    ///   - the target point will be replaced by the point that is closest to the target point,
    ///     that is still reachable from the starting point.
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline(always)]
    pub fn approx_path(&self, mut from: Vec2, mut to: Vec2) -> PathApproxResult {
        #[cfg(feature = "stats")]
        let start = Instant::now();

        let mut starting_polygon_index = self.get_point_location_ignore_delta(from);
        let mut ending_polygon_index = self.get_point_location_ignore_delta(to);
        let mut remove_first_waypoint = true;

        let status = if starting_polygon_index == u32::MAX {
            if ending_polygon_index == u32::MAX {
                return PathApproxResult {
                    path: Path::new_direct(from, to),
                    status: PathApproxResultEnum::BothOutside,
                };
            }
            // move start onto the closest island along the direct start-end-line
            remove_first_waypoint = false;
            let intersections = self.all_line_intersections(from, to);
            if intersections.is_empty() {
                // better buggy movement than an application crash!
                return PathApproxResult {
                    path: Path::new_direct(from, to),
                    status: PathApproxResultEnum::BugCrashPrevention,
                };
            }
            (starting_polygon_index, from) = self.approx_path_fix_start(ending_polygon_index, from, to, &intersections);
            // move end onto the same island
            (ending_polygon_index, to) = self.approx_path_fix_end(starting_polygon_index, from, to, &intersections);
            PathApproxResultEnum::StartOutside
        }
        else if ending_polygon_index == u32::MAX {
            // move end onto the same island
            let intersections = self.all_line_intersections(from, to);
            if intersections.is_empty() {
                // better buggy movement than an application crash!
                return PathApproxResult {
                    path: Path::new_direct(from, to),
                    status: PathApproxResultEnum::BugCrashPrevention,
                };
            }
            (ending_polygon_index, to) = self.approx_path_fix_end(starting_polygon_index, from, to, &intersections);
            PathApproxResultEnum::EndOutside
        }
        else {
            let islands = self.islands.as_ref().expect("islands must exist");
            let start_island = islands.get(starting_polygon_index as usize).expect("start point island must exist");
            let end_island = islands.get(ending_polygon_index as usize).expect("end point island must exist");
            if start_island != end_island {
                // move end onto the same island
                let intersections = self.all_line_intersections(from, to);
                if intersections.is_empty() {
                    // better buggy movement than an application crash!
                    return PathApproxResult {
                        path: Path::new_direct(from, to),
                        status: PathApproxResultEnum::BugCrashPrevention,
                    };
                }
                (ending_polygon_index, to) = self.approx_path_fix_end(starting_polygon_index, from, to, &intersections);
                PathApproxResultEnum::NoPath
            }
            else {
                PathApproxResultEnum::ValidPath
            }
        };


        if starting_polygon_index == ending_polygon_index {
            #[cfg(feature = "stats")]
            {
                if self.scenarios.get() == 0 {
                    eprintln!(
                        "index;micros;successor_calls;generated;pushed;popped;pruned_post_pop;length",
                    );
                }
                eprintln!(
                    "{};{};0;0;0;0;0;{}",
                    self.scenarios.get(),
                    start.elapsed().as_secs_f32() * 1_000_000.0,
                    from.distance(to),
                );
                self.scenarios.set(self.scenarios.get() + 1);
            }
            return PathApproxResult {
                path: Path::new_direct(from, to),
                status: PathApproxResultEnum::ValidPath,
            }
        }

        let mut search_instance = SearchInstance::setup(
            self,
            (from, starting_polygon_index),
            (to, ending_polygon_index),
            #[cfg(feature = "stats")]
            start,
        );

        // Limit search to avoid an infinite loop.
        for _ in 0..self.polygons.len() * 1000 {
            match search_instance.next(remove_first_waypoint) {
                InstanceStep::Found(path) => {
                    return PathApproxResult {
                        path,
                        status,
                    };
                },
                InstanceStep::NotFound => {
                    error!("Search from {from} to {to} failed. Please check if the mesh is valid as this should not happen as we've made sure that the two point are within the same mesh island");
                    return PathApproxResult {
                        path: Path::new_direct(from, to),
                        status: PathApproxResultEnum::NoPath,
                    }
                }
                InstanceStep::Continue => (),
            }
        }

        error!("Search from {from} to {to} failed. Please check if the mesh is valid as this should not happen. Infinite prevention triggered.");
        PathApproxResult {
            path: Path::new_direct(from, to),
            status: PathApproxResultEnum::InfinitePrevention,
        }
    }

    /// Returns a vector with all polygon-edge intersections sorted by their distance to the given `from` point.
    #[inline(always)]
    fn all_line_intersections(&self, from: Vec2, to: Vec2) -> Vec<Vec2> {
        let line = geo::LineString::new(vec![Coord { x: from.x, y: from.y }, Coord { x: to.x, y: to.y }]);
        let line_bb = line.bounding_rect().unwrap();
        let mut intersections = Vec::new();

        // TODO: terribly inefficient, at least cache the polygons as line strips or sth.
        for poly in &self.polygons {
            let poly_strip = poly.vertices.iter().map(|v| {
                let vt = &self.vertices[*v as usize];
                Coord { x: vt.coords.x, y: vt.coords.y }
            }).collect::<Vec<_>>();
            let mut strip = geo::LineString::new(poly_strip);
            strip.close();
            let poly = geo::Polygon::new(strip, vec![]);

            if line_bb.intersects(&poly.bounding_rect().unwrap()) {
                for line_poly in poly.exterior().lines_iter() {
                    if line.intersects(&line_poly) {
                        let intersection_opt = line_intersection_no_parallel( // TODO: since one line strip is fixed some intermediary calculations could be cached in an object
                            from.x, from.y,
                            to.x, to.y,
                            line_poly.start.x, line_poly.start.y,
                            line_poly.end.x, line_poly.end.y,
                        );
                        if let Some(mut intersection) = intersection_opt {
                            // due to floating point rounding inaccuracies,
                            // this intersection point is not guaranteed to be within the polygon
                            let is_in_mesh = self.get_point_location_ignore_delta(intersection) != u32::MAX;
                            if !is_in_mesh {
                                // try normal to get into the polygon
                                let p1_turned_90_deg = vec2(line_poly.start.y, -line_poly.start.x);
                                let p2_turned_90_deg = vec2(line_poly.end.y, -line_poly.end.x);
                                let normal = (p1_turned_90_deg - p2_turned_90_deg).normalize_or_zero();
                                let normal_mu = normal * 0.01;

                                let test_p = intersection + normal_mu;
                                let is_in_mesh = self.get_point_location_ignore_delta(test_p) != u32::MAX;
                                if is_in_mesh {
                                    // info!("had to fix an intersection, {:?} to {:?}", intersection, test_p);
                                    intersection = test_p;
                                }
                                else {
                                    let test_p = intersection - normal_mu;
                                    let is_in_mesh = self.get_point_location_ignore_delta(test_p) != u32::MAX;
                                    if is_in_mesh {
                                        // info!("had to fix an intersection, {:?} to {:?}", intersection, test_p);
                                        intersection = test_p;
                                    }
                                    else {
                                        error!("could not find an intersection point within the tested polygon, even though an intersection point was found! {:?}; {:?}", intersection, line_poly);
                                        continue;
                                    }
                                }
                            }

                            let dist_sq = (intersection - from).length_squared();
                            intersections.push((intersection, dist_sq))
                        }
                        else {
                            warn!("geo crate determined that lines line segments intersect, however the calculation did not retrieve any intersection points. probably a bug (parallel line intersections are not implemented yet)")
                        }
                    }
                }
            }
        }
        intersections.sort_unstable_by(|(_pos, dist_squared), (_pos2, dist_squared2)| dist_squared.total_cmp(dist_squared2));
        intersections.into_iter().map(|(pos, _dist_squared)| pos).collect()
    }

    /// The delta set by [`Mesh::set_delta`]
    pub fn delta(&self) -> f32 {
        self.delta
    }

    /// Set the delta for search with [`Mesh::path`], [`Mesh::get_path`], and [`Mesh::point_in_mesh`].
    /// A given point (x, y) will be searched in a square around a delimited by (x ± delta, y ± delta).
    ///
    /// Default is 0.1
    pub fn set_delta(&mut self, delta: f32) -> &mut Self {
        assert!(delta >= 0.0);
        self.delta = delta;
        self
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[cfg(test)]
    fn successors(&self, node: SearchNode, to: Vec2) -> Vec<SearchNode> {
        use hashbrown::HashMap;
        use std::collections::BinaryHeap;

        let mut search_instance = SearchInstance {
            #[cfg(feature = "stats")]
            start: Instant::now(),
            queue: BinaryHeap::new(),
            node_buffer: Vec::new(),
            root_history: HashMap::new(),
            from: Vec2::ZERO,
            to,
            polygon_to: self.get_point_location(to) as isize,
            mesh: self,
            #[cfg(feature = "stats")]
            pushed: 0,
            #[cfg(feature = "stats")]
            popped: 0,
            #[cfg(feature = "stats")]
            successors_called: 0,
            #[cfg(feature = "stats")]
            nodes_generated: 0,
            #[cfg(feature = "stats")]
            nodes_pruned_post_pop: 0,
            #[cfg(debug_assertions)]
            debug: false,
            #[cfg(debug_assertions)]
            fail_fast: -1,
        };
        search_instance.successors(node);
        search_instance.queue.drain().collect()
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[cfg(test)]
    fn edges_between(&self, node: &SearchNode) -> Vec<instance::Successor> {
        use hashbrown::HashMap;
        use std::collections::BinaryHeap;

        let search_instance = SearchInstance {
            #[cfg(feature = "stats")]
            start: Instant::now(),
            queue: BinaryHeap::new(),
            node_buffer: Vec::new(),
            root_history: HashMap::new(),
            from: Vec2::ZERO,
            to: Vec2::new(0.0, 0.0),
            polygon_to: self.get_point_location(Vec2::new(0.0, 0.0)) as isize,
            mesh: self,
            #[cfg(feature = "stats")]
            pushed: 0,
            #[cfg(feature = "stats")]
            popped: 0,
            #[cfg(feature = "stats")]
            successors_called: 0,
            #[cfg(feature = "stats")]
            nodes_generated: 0,
            #[cfg(feature = "stats")]
            nodes_pruned_post_pop: 0,
            #[cfg(debug_assertions)]
            debug: false,
            #[cfg(debug_assertions)]
            fail_fast: -1,
        };
        search_instance.edges_between(node).to_vec()
    }

    /// Check if a given point is in a `Mesh`
    pub fn point_in_mesh(&self, point: Vec2) -> bool {
        self.get_point_location(point) != u32::MAX
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    fn get_point_location(&self, point: Vec2) -> u32 {
        let delta = self.delta;
        [
            Vec2::new(0.0, 0.0),
            Vec2::new(delta, 0.0),
            Vec2::new(delta, delta),
            Vec2::new(0.0, delta),
            Vec2::new(-delta, delta),
            Vec2::new(-delta, 0.0),
            Vec2::new(-delta, -delta),
            Vec2::new(0.0, -delta),
            Vec2::new(delta, -delta),
        ]
        .iter()
        .map(|delta| {
            if self.baked_polygons.is_none() {
                self.get_point_location_unit(point + *delta)
            } else {
                self.get_point_location_unit_baked(point + *delta)
            }
        })
        .find(|poly| *poly != u32::MAX)
        .unwrap_or(u32::MAX)
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    fn get_point_location_ignore_delta(&self, point: Vec2) -> u32 {
        if self.baked_polygons.is_none() {
            self.get_point_location_unit(point)
        }
        else {
            self.get_point_location_unit_baked(point)
        }
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    fn get_point_location_unit(&self, point: Vec2) -> u32 {
        for (i, polygon) in self.polygons.iter().enumerate() {
            if self.point_in_polygon(point, polygon) {
                return i as u32;
            }
        }
        u32::MAX
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    fn get_point_location_unit_baked(&self, point: Vec2) -> u32 {
        self.baked_polygons
            .as_ref()
            .unwrap()
            .contains_iterator(&point)
            .find(|index| self.point_in_polygon(point, &self.polygons[*index]))
            .map(|index| index as u32)
            .unwrap_or(u32::MAX)
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline(always)]
    fn point_in_polygon(&self, point: Vec2, polygon: &Polygon) -> bool {
        let mut edged = false;
        for edge in polygon.edges_index().iter() {
            if edge.0.max(edge.1) as usize >= self.vertices.len() {
                return false;
            }
            edged = true;
            // Bounds are checked just before
            #[allow(unsafe_code)]
            let (last, next) = unsafe {
                (
                    self.vertices.get_unchecked(edge.0 as usize).coords,
                    self.vertices.get_unchecked(edge.1 as usize).coords,
                )
            };

            let current_side = point.side((last, next));
            if current_side == EdgeSide::Edge && point.on_segment((last, next)) {
                return true;
            }
            if current_side != EdgeSide::Left {
                return false;
            }
        }
        if edged {
            return true;
        }
        false
    }
}

#[derive(PartialEq, Debug)]
struct SearchNode {
    path: Vec<Vec2>,
    root: Vec2,
    interval: (Vec2, Vec2),
    edge: (u32, u32),
    polygon_from: isize,
    polygon_to: isize,
    f: f32,
    g: f32,
}

impl Display for SearchNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&format!("root=({}, {}); ", self.root.x, self.root.y))?;
        f.write_str(&format!(
            "left=({}, {}); ",
            self.interval.1.x, self.interval.1.y
        ))?;
        f.write_str(&format!(
            "right=({}, {}); ",
            self.interval.0.x, self.interval.0.y
        ))?;
        f.write_str(&format!("f={:.2}, g={:.2} ", self.f + self.g, self.f))?;
        Ok(())
    }
}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for SearchNode {}

impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self.f + self.g).total_cmp(&(other.f + other.g)) {
            Ordering::Less => Ordering::Greater,
            Ordering::Equal => self.f.total_cmp(&other.f),
            Ordering::Greater => Ordering::Less,
        }
    }
}


#[inline(always)]
fn line_intersection_no_parallel(x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32, x4: f32, y4: f32) -> Option<Vec2> {
    let x2x1 = x2 - x1;
    let x4x3 = x4 - x3;

    // https://de.wikipedia.org/wiki/Koordinatenform
    let a1 = y1 - y2;
    let b1 = x2x1;
    // let c1 = x2 * y1 - x1 * y2;

    let a2 = y3 - y4;
    let b2 = x4x3;
    // let c2 = x4 * y3 - x3 * y4;

    let is_parallel = a1 * b2 - a2 * b1 == 0.0;
    if is_parallel {
        return None; // TODO: for now ignored, should be calculated properly
    }

    // https://en.wikipedia.org/wiki/Intersection_(geometry)
    let y4y3 = y4 - y3;
    let y2y1 = y2 - y1;

    let x3x1 = x3 - x1;
    let y3y1 = y3 - y1;

    let f1 = x3x1 / x2x1;
    let f2 = x4x3 / x2x1;

    let g2 = y2y1 / y4y3;
    let g1 = y3y1 / y4y3;

    let t = (g2 * f1 - g1) / -(g2 * f2);
    let s = f1 + t * f2;

    if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
        return None; // the intersection is not on the two line segments
    }

    let top_mul_left = x3 * y4 - y3 * x4;
    let top_mul_right = x1 * y2 - y1 * x2;

    let bottom_term = x2x1 * y4y3 - y2y1 * x4x3;
    let intersection = vec2(
        (x2x1 * top_mul_left - x4x3 * top_mul_right) / bottom_term,
        (y2y1 * top_mul_left - y4y3 * top_mul_right) / bottom_term,
    );
    Some(intersection)
}

#[cfg(test)]
mod tests {
    macro_rules! assert_delta {
        ($x:expr, $y:expr) => {
            let val = $x;
            let expected = $y;
            if !((val - expected).abs() < 0.01) {
                assert_eq!(val, expected);
            }
        };
    }

    use glam::Vec2;

    use crate::{helpers::*, Mesh, Path, Polygon, SearchNode, Vertex};

    fn mesh_u_grid() -> Mesh {
        Mesh {
            vertices: vec![
                Vertex::new(Vec2::new(0., 0.), vec![0, -1]),
                Vertex::new(Vec2::new(1., 0.), vec![0, 1, -1]),
                Vertex::new(Vec2::new(2., 0.), vec![1, 2, -1]),
                Vertex::new(Vec2::new(3., 0.), vec![2, -1]),
                Vertex::new(Vec2::new(0., 1.), vec![3, 0, -1]),
                Vertex::new(Vec2::new(1., 1.), vec![3, 1, 0, -1]),
                Vertex::new(Vec2::new(2., 1.), vec![4, 2, 1, -1]),
                Vertex::new(Vec2::new(3., 1.), vec![4, 2, -1]),
                Vertex::new(Vec2::new(0., 2.), vec![3, -1]),
                Vertex::new(Vec2::new(1., 2.), vec![3, -1]),
                Vertex::new(Vec2::new(2., 2.), vec![4, -1]),
                Vertex::new(Vec2::new(3., 2.), vec![4, -1]),
            ],
            polygons: vec![
                Polygon::new(vec![0, 1, 5, 4], false),
                Polygon::new(vec![1, 2, 6, 5], false),
                Polygon::new(vec![2, 3, 7, 6], false),
                Polygon::new(vec![4, 5, 9, 8], true),
                Polygon::new(vec![6, 7, 11, 10], true),
            ],
            ..Default::default()
        }
    }

    #[test]
    fn point_in_polygon() {
        let mut mesh = mesh_u_grid();
        mesh.bake();
        assert_eq!(mesh.get_point_location(Vec2::new(0.5, 0.5)), 0);
        assert_eq!(mesh.get_point_location(Vec2::new(1.5, 0.5)), 1);
        assert_eq!(mesh.get_point_location(Vec2::new(0.5, 1.5)), 3);
        assert_eq!(mesh.get_point_location(Vec2::new(1.5, 1.5)), u32::MAX);
        assert_eq!(mesh.get_point_location(Vec2::new(2.5, 1.5)), 4);
    }

    #[test]
    fn successors_straight_line_ahead() {
        let mesh = mesh_u_grid();

        let from = Vec2::new(0.1, 0.1);
        let to = Vec2::new(2.9, 0.9);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0)),
            edge: (1, 5),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 1,
            f: from.distance(to),
            g: 0.0,
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 1);
        assert_eq!(successors[0].root, from);
        assert_eq!(successors[0].f, from.distance(to));
        assert_eq!(successors[0].g, from.distance(to));
        assert_eq!(successors[0].polygon_from, 1);
        assert_eq!(successors[0].polygon_to, 2);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(2.0, 0.0), Vec2::new(2.0, 1.0))
        );
        assert_eq!(successors[0].edge, (2, 6));

        assert_eq!(successors[0].path, Vec::<Vec2>::new());

        assert_eq!(
            mesh.path(from, to).unwrap(),
            Path {
                path: vec![to],
                length: from.distance(to),
            }
        );
    }

    #[test]
    fn successors_straight_line_reversed() {
        let mesh = mesh_u_grid();

        let to = Vec2::new(0.1, 0.1);
        let from = Vec2::new(2.9, 0.9);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(2.0, 1.0), Vec2::new(2.0, 0.0)),
            edge: (6, 2),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 1,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = mesh.successors(search_node, to);
        assert_eq!(successors.len(), 1);
        assert_eq!(successors[0].root, from);
        assert_eq!(successors[0].f, 0.0);
        assert_eq!(successors[0].g, to.distance(from));
        assert_eq!(successors[0].polygon_from, 1);
        assert_eq!(successors[0].polygon_to, 0);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0))
        );
        assert_eq!(successors[0].edge, (5, 1));
        assert_eq!(successors[0].path, Vec::<Vec2>::new());

        assert_eq!(
            mesh.path(from, to).unwrap(),
            Path {
                path: vec![to],
                length: from.distance(to),
            }
        );
    }

    #[test]
    fn successors_corner_first_step() {
        let mesh = mesh_u_grid();

        let from = Vec2::new(0.1, 1.9);
        let to = Vec2::new(2.1, 1.9);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(0.0, 1.0), Vec2::new(1.0, 1.0)),
            edge: (4, 5),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 0,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 1);
        assert_eq!(successors[0].root, Vec2::new(2.0, 1.0));
        assert_eq!(
            successors[0].f,
            from.distance(Vec2::new(1.0, 1.0)) + Vec2::new(1.0, 1.0).distance(Vec2::new(2.0, 1.0))
        );
        assert_eq!(successors[0].g, Vec2::new(2.0, 1.0).distance(to));
        assert_eq!(successors[0].polygon_from, 2);
        assert_eq!(successors[0].polygon_to, 4);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(3.0, 1.0), Vec2::new(2.0, 1.0))
        );
        assert_eq!(successors[0].edge, (7, 6));
        assert_eq!(successors[0].path, vec![from, Vec2::new(1.0, 1.0)]);

        assert_eq!(
            mesh.path(from, to).unwrap(),
            Path {
                path: vec![Vec2::new(1.0, 1.0), Vec2::new(2.0, 1.0), to],
                length: from.distance(Vec2::new(1.0, 1.0))
                    + Vec2::new(1.0, 1.0).distance(Vec2::new(2.0, 1.0))
                    + Vec2::new(2.0, 1.0).distance(to),
            }
        );
    }

    #[test]
    fn successors_corner_observable_second_step() {
        let mesh = mesh_u_grid();

        let from = Vec2::new(0.1, 1.9);
        let to = Vec2::new(2.1, 1.9);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0)),
            edge: (1, 5),

            polygon_from: 0,
            polygon_to: 1,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 1);
        assert_eq!(successors[0].root, Vec2::new(2.0, 1.0));
        assert_eq!(
            successors[0].f,
            from.distance(Vec2::new(1.0, 1.0)) + Vec2::new(1.0, 1.0).distance(Vec2::new(2.0, 1.0))
        );
        assert_eq!(successors[0].g, Vec2::new(2.0, 1.0).distance(to));
        assert_eq!(successors[0].polygon_from, 2);
        assert_eq!(successors[0].polygon_to, 4);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(3.0, 1.0), Vec2::new(2.0, 1.0))
        );
        assert_eq!(successors[0].edge, (7, 6));
        assert_eq!(successors[0].path, vec![from, Vec2::new(1.0, 1.0)]);

        assert_eq!(
            mesh.path(from, to).unwrap(),
            Path {
                path: vec![Vec2::new(1.0, 1.0), Vec2::new(2.0, 1.0), to],
                length: from.distance(Vec2::new(1.0, 1.0))
                    + Vec2::new(1.0, 1.0).distance(Vec2::new(2.0, 1.0))
                    + Vec2::new(2.0, 1.0).distance(to),
            }
        );
    }

    fn mesh_from_paper() -> Mesh {
        Mesh {
            vertices: vec![
                Vertex::new(Vec2::new(0., 6.), vec![0, -1]),    // 0
                Vertex::new(Vec2::new(2., 5.), vec![0, -1, 2]), // 1
                Vertex::new(Vec2::new(5., 7.), vec![0, 2, -1]), // 2
                Vertex::new(Vec2::new(5., 8.), vec![0, -1]),    // 3
                Vertex::new(Vec2::new(0., 8.), vec![0, -1]),    // 4
                Vertex::new(Vec2::new(1., 4.), vec![1, -1]),    // 5
                Vertex::new(Vec2::new(2., 1.), vec![1, -1]),    // 6
                Vertex::new(Vec2::new(4., 1.), vec![1, -1]),    // 7
                Vertex::new(Vec2::new(4., 2.), vec![1, -1, 2]), // 8
                Vertex::new(Vec2::new(2., 4.), vec![1, 2, -1]), // 9
                Vertex::new(Vec2::new(7., 4.), vec![2, -1, 4]), // 10
                Vertex::new(Vec2::new(10., 7.), vec![2, 4, 6, -1, 3]), // 11
                Vertex::new(Vec2::new(7., 7.), vec![2, 3, -1]), // 12
                Vertex::new(Vec2::new(11., 8.), vec![3, -1]),   // 13
                Vertex::new(Vec2::new(7., 8.), vec![3, -1]),    // 14
                Vertex::new(Vec2::new(7., 0.), vec![5, 4, -1]), // 15
                Vertex::new(Vec2::new(11., 3.), vec![4, 5, -1]), // 16
                Vertex::new(Vec2::new(11., 5.), vec![4, -1, 6]), // 17
                Vertex::new(Vec2::new(12., 0.), vec![5, -1]),   // 18
                Vertex::new(Vec2::new(12., 3.), vec![5, -1]),   // 19
                Vertex::new(Vec2::new(13., 5.), vec![6, -1]),   // 20
                Vertex::new(Vec2::new(13., 7.), vec![6, -1]),   // 21
                Vertex::new(Vec2::new(1., 3.), vec![1, -1]),    // 22
            ],
            polygons: vec![
                Polygon::new(vec![0, 1, 2, 3, 4], true),
                Polygon::new(vec![5, 22, 6, 7, 8, 9], true),
                Polygon::new(vec![1, 9, 8, 10, 11, 12, 2], false),
                Polygon::new(vec![12, 11, 13, 14], true),
                Polygon::new(vec![10, 15, 16, 17, 11], false),
                Polygon::new(vec![15, 18, 19, 16], true),
                Polygon::new(vec![11, 17, 20, 21], true),
            ],
            ..Default::default()
        }
    }

    #[test]
    fn paper_point_in_polygon() {
        let mut mesh = mesh_from_paper();
        mesh.bake();
        assert_eq!(mesh.get_point_location(Vec2::new(0.5, 0.5)), u32::MAX);
        assert_eq!(mesh.get_point_location(Vec2::new(2.0, 6.0)), 0);
        assert_eq!(mesh.get_point_location(Vec2::new(2.0, 5.1)), 0);
        assert_eq!(mesh.get_point_location(Vec2::new(2.0, 1.5)), 1);
        assert_eq!(mesh.get_point_location(Vec2::new(4.0, 2.1)), 2);
    }

    #[test]
    fn paper_straight() {
        let mesh = mesh_from_paper();

        let from = Vec2::new(12.0, 0.0);
        let to = Vec2::new(7.0, 6.9);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(11.0, 3.0), Vec2::new(7.0, 0.0)),
            edge: (16, 15),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 4,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 2);

        assert_eq!(successors[1].root, Vec2::new(11.0, 3.0));
        assert_eq!(successors[1].f, from.distance(Vec2::new(11.0, 3.0)));
        assert_eq!(
            successors[1].g,
            Vec2::new(11.0, 3.0).distance(Vec2::new(9.75, 6.75))
                + Vec2::new(9.75, 6.75).distance(to)
        );
        assert_eq!(successors[1].polygon_from, 4);
        assert_eq!(successors[1].polygon_to, 2);
        assert_eq!(
            successors[1].interval,
            (Vec2::new(10.0, 7.0), Vec2::new(9.75, 6.75))
        );
        assert_eq!(successors[1].edge, (11, 10));
        assert_eq!(successors[1].path, vec![from]);

        assert_eq!(successors[0].root, from);
        assert_eq!(successors[0].f, 0.0);
        assert_eq!(successors[0].g, from.distance(to));
        assert_eq!(successors[0].polygon_from, 4);
        assert_eq!(successors[0].polygon_to, 2);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(9.75, 6.75), Vec2::new(7.0, 4.0))
        );
        assert_eq!(successors[0].edge, (11, 10));
        assert_eq!(successors[0].path, Vec::<Vec2>::new());

        assert_eq!(mesh.path(from, to).unwrap().length, from.distance(to));
        assert_eq!(mesh.path(from, to).unwrap().path, vec![to]);
    }

    #[test]
    fn paper_corner_right() {
        let mesh = mesh_from_paper();

        let from = Vec2::new(12.0, 0.0);
        let to = Vec2::new(13.0, 6.0);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(11.0, 3.0), Vec2::new(7.0, 0.0)),
            edge: (16, 15),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 4,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 3);

        assert_eq!(successors[0].root, Vec2::new(11.0, 3.0));
        assert_eq!(successors[0].f, from.distance(Vec2::new(11.0, 3.0)));
        assert_eq!(
            successors[0].g,
            Vec2::new(11.0, 3.0).distance(Vec2::new(11.0, 5.0)) + Vec2::new(11.0, 5.0).distance(to)
        );
        assert_eq!(successors[0].polygon_from, 4);
        assert_eq!(successors[0].polygon_to, 6);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(11.0, 5.0), Vec2::new(10.0, 7.0))
        );
        assert_eq!(successors[0].edge, (17, 11));
        assert_eq!(successors[0].path, vec![from]);

        assert_eq!(successors[1].root, Vec2::new(11.0, 3.0));
        assert_eq!(successors[1].f, from.distance(Vec2::new(11.0, 3.0)));
        assert_eq!(
            successors[1].g,
            Vec2::new(11.0, 3.0).distance(to.mirror((Vec2::new(10.0, 7.0), Vec2::new(9.75, 6.75))))
        );
        assert_eq!(successors[1].polygon_from, 4);
        assert_eq!(successors[1].polygon_to, 2);
        assert_eq!(
            successors[1].interval,
            (Vec2::new(10.0, 7.0), Vec2::new(9.75, 6.75))
        );
        assert_eq!(successors[1].edge, (11, 10));
        assert_eq!(successors[1].path, vec![from]);

        assert_eq!(successors[2].root, from);
        assert_eq!(successors[2].f, 0.0);
        assert_eq!(
            successors[2].g,
            from.distance(Vec2::new(9.75, 6.75))
                + Vec2::new(9.75, 6.75)
                    .distance(to.mirror((Vec2::new(9.75, 6.75), Vec2::new(7.0, 4.0))))
        );
        assert_eq!(successors[2].polygon_from, 4);
        assert_eq!(successors[2].polygon_to, 2);
        assert_eq!(
            successors[2].interval,
            (Vec2::new(9.75, 6.75), Vec2::new(7.0, 4.0))
        );
        assert_eq!(successors[2].edge, (11, 10));
        assert_eq!(successors[2].path, Vec::<Vec2>::new());

        assert_delta!(
            mesh.path(from, to).unwrap().length,
            from.distance(Vec2::new(11.0, 3.0))
                + Vec2::new(11.0, 3.0).distance(Vec2::new(11.0, 5.0))
                + Vec2::new(11.0, 5.0).distance(to)
        );
        assert_eq!(
            mesh.path(from, to).unwrap().path,
            vec![Vec2::new(11.0, 3.0), Vec2::new(11.0, 5.0), to]
        );
    }

    #[test]
    fn paper_corner_left() {
        let mesh = mesh_from_paper();

        let from = Vec2::new(12.0, 0.0);
        let to = Vec2::new(5.0, 3.0);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(11.0, 3.0), Vec2::new(7.0, 0.0)),
            edge: (16, 15),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 4,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 2);

        assert_eq!(successors[1].root, Vec2::new(11.0, 3.0));
        assert_eq!(successors[1].f, from.distance(Vec2::new(11.0, 3.0)));
        assert_eq!(
            successors[1].g,
            Vec2::new(11.0, 3.0).distance(Vec2::new(9.75, 6.75))
                + Vec2::new(9.75, 6.75).distance(to)
        );
        assert_eq!(successors[1].polygon_from, 4);
        assert_eq!(successors[1].polygon_to, 2);
        assert_eq!(
            successors[1].interval,
            (Vec2::new(10.0, 7.0), Vec2::new(9.75, 6.75))
        );
        assert_eq!(successors[1].edge, (11, 10));
        assert_eq!(successors[1].path, vec![from]);

        assert_eq!(successors[0].root, from);
        assert_eq!(successors[0].f, 0.0);
        assert_eq!(
            successors[0].g,
            from.distance(Vec2::new(7.0, 4.0)) + Vec2::new(7.0, 4.0).distance(to)
        );
        assert_eq!(successors[0].polygon_from, 4);
        assert_eq!(successors[0].polygon_to, 2);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(9.75, 6.75), Vec2::new(7.0, 4.0))
        );
        assert_eq!(successors[0].edge, (11, 10));
        assert_eq!(successors[0].path, Vec::<Vec2>::new());

        assert_delta!(
            mesh.path(from, to).unwrap().length,
            from.distance(Vec2::new(7.0, 4.0)) + Vec2::new(7.0, 4.0).distance(to)
        );
        assert_eq!(
            mesh.path(from, to).unwrap().path,
            vec![Vec2::new(7.0, 4.0), to]
        );
    }

    #[test]
    fn paper_going_to_one_way_polygon() {
        let mesh = mesh_from_paper();

        let from = Vec2::new(11., 0.);
        let to = Vec2::new(9., 3.);
        let path = mesh.path(from, to);

        assert_eq!(path.unwrap().path, vec![to]);

        let path = mesh.path(to, from);

        assert_eq!(path.unwrap().path, vec![from]);
    }

    #[test]
    fn paper_corner_left_twice() {
        let mesh = mesh_from_paper();

        let from = Vec2::new(12.0, 0.0);
        let to = Vec2::new(3.0, 1.0);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(11.0, 3.0), Vec2::new(7.0, 0.0)),
            edge: (16, 15),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 4,
            f: 0.0,
            g: from.distance(to),
        };
        let successors = dbg!(mesh.successors(search_node, to));
        assert_eq!(successors.len(), 2);

        assert_eq!(successors[1].root, Vec2::new(11.0, 3.0));
        assert_eq!(successors[1].f, from.distance(Vec2::new(11.0, 3.0)));
        assert_eq!(
            successors[1].g,
            Vec2::new(11.0, 3.0).distance(Vec2::new(9.75, 6.75))
                + Vec2::new(9.75, 6.75).distance(to)
        );
        assert_eq!(successors[1].polygon_from, 4);
        assert_eq!(successors[1].polygon_to, 2);
        assert_eq!(
            successors[1].interval,
            (Vec2::new(10.0, 7.0), Vec2::new(9.75, 6.75))
        );
        assert_eq!(successors[1].edge, (11, 10));
        assert_eq!(successors[1].path, vec![from]);

        assert_eq!(successors[0].root, from);
        assert_eq!(successors[0].f, 0.0);
        assert_eq!(
            successors[0].g,
            from.distance(Vec2::new(7.0, 4.0)) + Vec2::new(7.0, 4.0).distance(to)
        );
        assert_eq!(successors[0].polygon_from, 4);
        assert_eq!(successors[0].polygon_to, 2);
        assert_eq!(
            successors[0].interval,
            (Vec2::new(9.75, 6.75), Vec2::new(7.0, 4.0))
        );
        assert_eq!(successors[0].edge, (11, 10));
        assert_eq!(successors[0].path, Vec::<Vec2>::new());

        let successor = successors.into_iter().next().unwrap();
        let successors = dbg!(mesh.successors(successor, to));
        dbg!(&successors[0]);
        assert_eq!(successors.len(), 1);

        assert_delta!(
            mesh.path(from, to).unwrap().length,
            from.distance(Vec2::new(7.0, 4.0))
                + Vec2::new(7.0, 4.0).distance(Vec2::new(4.0, 2.0))
                + Vec2::new(4.0, 2.0).distance(to)
        );

        assert_eq!(
            mesh.path(from, to).unwrap().path,
            vec![Vec2::new(7.0, 4.0), Vec2::new(4.0, 2.0), to]
        );
    }

    #[test]
    fn edges_between_simple() {
        let mesh = mesh_from_paper();

        let from = Vec2::new(12.0, 0.0);
        let to = Vec2::new(3.0, 1.0);
        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(11.0, 3.0), Vec2::new(7.0, 0.0)),
            edge: (16, 15),
            polygon_from: mesh.get_point_location(from) as isize,
            polygon_to: 4,
            f: 0.0,
            g: from.distance(to),
        };

        let successors = mesh.edges_between(&search_node);

        for successor in &successors {
            println!("{successor:?}");
        }

        println!("=========================");

        let search_node = SearchNode {
            path: vec![],
            root: from,
            interval: (Vec2::new(9.75, 6.75), Vec2::new(7.0, 4.0)),
            edge: (11, 10),
            polygon_from: 4,
            polygon_to: 2,
            f: 0.0,
            g: from.distance(to),
        };

        let successors = mesh.edges_between(&search_node);

        for successor in &successors {
            println!("{successor:?}");
        }

        println!("=========================");

        let search_node = SearchNode {
            path: vec![],
            root: Vec2::new(11.0, 3.0),
            interval: (Vec2::new(10.0, 7.0), Vec2::new(7.0, 4.0)),
            edge: (11, 10),
            polygon_from: 4,
            polygon_to: 2,
            f: 0.0,
            g: from.distance(to),
        };

        let successors = mesh.edges_between(&search_node);

        for successor in &successors {
            println!("{successor:?}");
        }
    }

    #[test]
    fn edges_between_simple_u() {
        let mesh = mesh_u_grid();

        let search_node = SearchNode {
            path: vec![],
            root: Vec2::new(0.0, 0.0),
            interval: (Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0)),
            edge: (1, 5),
            polygon_from: 0,
            polygon_to: 1,
            f: 0.0,
            g: 1.0,
        };

        let successors = mesh.edges_between(&search_node);

        for successor in &successors {
            println!("{successor:?}");
        }
    }
}
