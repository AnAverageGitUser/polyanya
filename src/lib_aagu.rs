use std::iter;
use geo::{BoundingRect, Coord, Intersects, LinesIter};
use glam::{Vec2, vec2};
use log::{error, warn};
use crate::instance::{InstanceStep, SearchInstance};
use crate::{Mesh, Path};

#[cfg(feature = "tracing")]
use tracing::instrument;

const EPSILON: f32 = 0.001;

/// The mode given to the navigation algorithm to determine which kind of behaviour we expect when the start or end
/// point is not within the mesh.
#[derive(Debug, Copy, Clone)]
pub enum PointCorrectionMode {
    /// If the point is not in the mesh, return no path.
    ///
    /// Choose this for the start point, if your agents are either never outside the navmesh or if it is acceptable
    /// that agents that are outside your navmesh can not move again until they are somehow put onto the navmesh again.
    ///
    /// Choose this for the end point, if your agents must not start moving to the target if the target is outside of
    /// the navmesh. The target point is the deemed unreachable.
    NoCorrection,
    /// This is a somewhat more complicated, more expensive, but higher quality calculation method than
    /// [`PointCorrectionMode::ClosestMeshEdge`].
    ///
    /// If the point is not within the mesh, find a point that is approximately the closest to that point but within the
    /// mesh and within the given distance.
    /// If that point exists, use that point as the next (if used for start point) or previous (if used for end point)
    /// waypoint of the given point, otherwise return no path.
    /// If you want to allow arbitrary distances, parameterize with `f32::INFINITY`.
    ///
    /// Choose this for the start point, if the agent may sometimes be outside the navmesh and you want to make sure
    /// that the agent moves the shortest distance possible to first be within the navmesh again,
    /// before making themself on the way for the target.
    ///
    /// Choose this for the end point, if the agent should move to the closest point within the navmesh possible,
    /// when the end point is outside the navmesh - but still within the given distance of the navmesh.
    ClosestPointInMesh(f32),
    /// This is a somewhat easier, cheaper, but lower quality calculation method than
    /// [`PointCorrectionMode::ClosestPointInMesh`].
    /// Refer to that item for some more info.
    ///
    /// If the point is not within the mesh, find the closest edge point of the mesh that is within the given distance.
    /// If that point exists, use that point as the next (if used for start point) or previous (if used for end point)
    /// waypoint of the given point, otherwise return no path.
    /// If you want to allow arbitrary distances, parameterize with `f32::INFINITY`.
    ClosestMeshEdge(f32),
}
/// The mode given to the navigation algorithm to determine which kind of behaviour we expect when the
/// (possibly corrected) start point or (possibly corrected) end point are on different islands.
#[derive(Debug, Copy, Clone)]
pub enum IslandTraversalMode {
    /// If the (possibly corrected) start point and the (possibly corrected) end point are on different islands,
    /// return no path.
    NoTraversal,
    /// If the (possibly corrected) start point and the (possibly corrected) end point are on different islands,
    /// change the (possibly corrected) end point to the closest point that is on the same island as the
    /// (possibly corrected) start point and the original uncorrected end point,
    /// but only if that point is not farther away than the given distance.
    ClosestOnStartIsland(f32),
}
/// The settings for the navigation algorithm.
/// Certain situation can be configured to be handled differently.
#[derive(Debug, Copy, Clone)]
pub struct NavigationRequestSettings {
    /// The [`PointCorrectionMode`] of this navigation request's start point.
    pub start: PointCorrectionMode,
    /// The [`IslandTraversalMode`] of this navigation request.
    pub islands: IslandTraversalMode,
    /// The [`PointCorrectionMode`] of this navigation request's end point.
    pub end: PointCorrectionMode,
    /// A placeholder, *not used right now*.
    /// The radius that the agent has.
    /// Whenever a path is created, it is checked if this radius fits in both directions on each polygon line it
    /// intersects.
    /// This might shift the waypoint of the agent on each polygon edge, to "make room" for the agent.
    /// This should work as long as every polygon's edge is adjacent to the outside or an obstacle.
    pub _agent_radius: f32,
    /// A placeholder, *not used right now*.
    /// The maximum allowed length that the agent is allowed to travel.
    /// This can be used as an optimization to terminate some paths early on if they are to far away.
    pub _max_length: f32,
}

/// A status indication if a start/end point was modified and if yes to what.
#[derive(Debug, Copy, Clone)]
pub enum PointStatus {
    /// The point was not modified.
    Original,
    /// The point was modified according to the given setting.
    Modified {
        original: Vec2,
        modified: Vec2,
    },
}
/// TODO
#[derive(Debug, Copy, Clone)]
pub enum PathApproxResultEnum {
    /// The (possibly corrected) start point and (possibly corrected) end point are within the same mesh islands and
    /// a path was found.
    ValidPath,/// The start- and end-points are within different mesh islands.
    /// There cannot exist a valid path between the two points.
    ///
    /// A direct path that ignores the mesh-bounds is given.
    NoValidPath,
    /// The (possibly corrected) start point and (possibly corrected) end point are on different mesh islands and
    /// the navigation request did not allow creating a path.
    DifferentIslandsNoPath,
    /// The (possibly corrected) start point and (possibly corrected) end point are on different mesh islands and
    /// the navigation request did allow creating a path that lets the agent move closer to the target, but only within
    /// the starting point's island.
    DifferentIslandsFirstIsland,
    /// The start- and end-point are within the same mesh islands, but a path could not be found and deadlock prevention
    /// kicked in.
    /// This is a bug, but does not seem to happen frequently enough to be important.
    ///
    /// A direct path that ignores the mesh-bounds is given.
    InfinitePrevention,
    /// The navigation algorithm has a bug, instead of crashing the application, this result was returned.
    ///
    /// A direct path that ignores the mesh-bounds is given.
    BugCrashPrevention,
}
impl PathApproxResultEnum {
    /// True if the situation is expected to occur.
    pub fn is_valid_situation(&self) -> bool {
        match self {
            PathApproxResultEnum::ValidPath => {true}
            PathApproxResultEnum::NoValidPath => {true}
            PathApproxResultEnum::DifferentIslandsNoPath => {true}
            PathApproxResultEnum::DifferentIslandsFirstIsland => {true}
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
    pub start: PointStatus,
    /// TODO
    pub status: PathApproxResultEnum,
    /// TODO
    pub end: PointStatus,
}
impl PathApproxResult {
    /// Gets the last element of the path: the destination.
    pub fn get_end(&self) -> &Vec2 {
        self.path.path.last().expect("expected path to never be empty")
    }
}

/// Creates a new object with the target as single waypoint.
pub fn new_direct_path(from: Vec2, to: Vec2) -> Path {
    Path {
        length: from.distance(to),
        path: vec![to]
    }
}

pub fn new_possibly_corrected_path(from: Vec2, to: Vec2, from_orig: Vec2) -> Path {
    if from != from_orig {
        Path {
            length: from_orig.distance(from) + from.distance(to),
            path: vec![from, to]
        }
    }
    else {
        Path {
            length: from.distance(to),
            path: vec![to]
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


/// A navigation mesh
#[derive(Debug, Clone)]
pub struct MeshAagu<'m> {
    pub(crate) mesh: &'m Mesh,
}
impl<'m> MeshAagu<'m> {
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
    fn approx_path_fix_end(&self, start_index: u32, _from: Vec2, _to: Vec2, intersections: &[Vec2]) -> (u32, Vec2) {
        let islands = self.mesh.islands.as_ref().expect("islands must exist");
        let start_island = islands.get(start_index as usize).expect("start point island must exist");

        for pos in intersections.iter().rev() {
            let poly_idx = self.get_point_location_ignore_delta(*pos);
            if poly_idx == u32::MAX {
                // if this triggers, this is probably because of floating point inaccuracies
                unreachable!("either the start or the end must be within the mesh, therefore the line segment SHOULD HAVE gotten at least one intermediate result");
            }

            // if this triggers, this is probably because of floating point inaccuracies
            let end_island = islands.get(poly_idx as usize).expect("end point island must exist");
            if start_island != end_island {
                continue;
            }

            return (poly_idx, *pos)
        }
        // if this triggers, this is probably a logical error
        unreachable!("either the start or the end must be within the mesh");
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
    pub fn approx_path(&self, mut from: Vec2, mut to: Vec2, settings: NavigationRequestSettings) -> PathApproxResult {
        #[cfg(feature = "stats")]
        let start = std::time::Instant::now();

        let from_orig = from;
        let to_orig = to;
        let mut starting_polygon_idx = self.get_point_location_ignore_delta(from);
        let mut ending_polygon_idx = self.get_point_location_ignore_delta(to);
        let mut start_status = PointStatus::Original;
        let mut end_status = PointStatus::Original;

        let fallback = ||{ new_direct_path(from_orig, to_orig) };

        if starting_polygon_idx == u32::MAX {
            match self.fix_outside_mesh_point(&settings.start, to, starting_polygon_idx, fallback, |_| true) {
                Ok(res) => { (from, starting_polygon_idx) = res; }
                Err(res) => { return res; }
            }
            if from != from_orig {
                start_status = PointStatus::Modified {
                    original: from_orig,
                    modified: from,
                }
            }
        }

        let islands = self.mesh.islands.as_ref().expect("island baking is a prerequisite");
        let starting_island = islands[starting_polygon_idx as usize];
        let same_island_as_start = |poly_idx: usize| {
            islands[poly_idx] == starting_island
        };

        if ending_polygon_idx == u32::MAX {
            match self.fix_outside_mesh_point(&settings.end, to, starting_polygon_idx, fallback, same_island_as_start) {
                Ok(res) => { (to, ending_polygon_idx) = res; }
                Err(res) => { return res; }
            }
            if to != to_orig {
                end_status = PointStatus::Modified {
                    original: to_orig,
                    modified: to,
                }
            }
        }

        if starting_polygon_idx == ending_polygon_idx {
            #[cfg(feature = "stats")]
            {
                if self.mesh.scenarios.get() == 0 {
                    eprintln!(
                        "index;micros;successor_calls;generated;pushed;popped;pruned_post_pop;length",
                    );
                }
                eprintln!(
                    "{};{};0;0;0;0;0;{}",
                    self.mesh.scenarios.get(),
                    start.elapsed().as_secs_f32() * 1_000_000.0,
                    from.distance(to),
                );
                self.mesh.scenarios.set(self.mesh.scenarios.get() + 1);
            }
            return PathApproxResult {
                path: new_possibly_corrected_path(from, to, from_orig),
                start: start_status,
                status: PathApproxResultEnum::ValidPath,
                end: end_status,
            }
        }

        let mut search_instance = SearchInstance::setup(
            self.mesh,
            (from, starting_polygon_idx),
            (to, ending_polygon_idx),
            #[cfg(feature = "stats")]
            start,
        );

        // Limit search to avoid an infinite loop.
        for _ in 0..self.mesh.polygons.len() * 1000 {
            match search_instance.next() {
                InstanceStep::Found(path) => {
                    return PathApproxResult {
                        path,
                        start: start_status,
                        status: PathApproxResultEnum::ValidPath,
                        end: end_status,
                    };
                },
                InstanceStep::NotFound => {
                    error!("Search from {from_orig} to {to_orig} failed. Please check if the mesh is valid as this should not happen as we've made sure that the two point are within the same mesh island");
                    return PathApproxResult {
                        path: new_direct_path(from_orig, to_orig),
                        start: PointStatus::Original,
                        status: PathApproxResultEnum::NoValidPath,
                        end: PointStatus::Original,
                    }
                }
                InstanceStep::Continue => (),
            }
        }

        error!("Search from {from_orig} to {to_orig} failed. Please check if the mesh is valid as this should not happen. Infinite prevention triggered.");
        PathApproxResult {
            path: new_direct_path(from_orig, to_orig),
            start: PointStatus::Original,
            status: PathApproxResultEnum::InfinitePrevention,
            end: PointStatus::Original,
        }
    }

    fn fix_outside_mesh_point(
        &self,
        mode: &PointCorrectionMode,
        outside_mesh_point: Vec2,
        polygon_index: u32,
        fallback: impl FnOnce() -> Path,
        polygon_filter: impl Fn(usize) -> bool,
    )
        -> Result<(Vec2, u32), PathApproxResult>
    {
        if polygon_index != u32::MAX {
            return Ok((outside_mesh_point, polygon_index));
        }

        let corrected_opt = match mode {
            PointCorrectionMode::NoCorrection => {
                return Err(PathApproxResult {
                    path: fallback(),
                    start: PointStatus::Original,
                    status: PathApproxResultEnum::NoValidPath,
                    end: PointStatus::Original,
                })
            },
            PointCorrectionMode::ClosestPointInMesh(max_dist) => self.closest_exterior_point(outside_mesh_point, *max_dist, polygon_filter),
            PointCorrectionMode::ClosestMeshEdge(max_dist) => self.closest_exterior_point_line_edge(outside_mesh_point, *max_dist, polygon_filter),
        };

        let Some((possibly_corrected_start_point, new_polygon_index, max_dist_sq)) = corrected_opt else {
            return Err(PathApproxResult {
                path: fallback(),
                start: PointStatus::Original,
                status: PathApproxResultEnum::NoValidPath,
                end: PointStatus::Original,
            })
        };

        debug_assert_ne!(new_polygon_index, u32::MAX, "The point was fixed, therefore a polygon with which it was fixed must have been found.");
        if let Some(possibly_corrected_start_point) = self.fix_intersection_point(outside_mesh_point, possibly_corrected_start_point, new_polygon_index, max_dist_sq) {
            Ok((possibly_corrected_start_point, new_polygon_index))
        }
        else {
            // this should only happen because of floating point inaccuracies, and because the polygon was to small/thin
            // if even the correction for the correction failed, we give up - this hopefully almost never happens
            Err(PathApproxResult {
                path: fallback(),
                start: PointStatus::Original,
                status: PathApproxResultEnum::BugCrashPrevention,
                end: PointStatus::Original,
            })
        }
    }

    fn fix_intersection_point(&self, point_orig: Vec2, point: Vec2, polygon_idx: u32, max_dist_sq: f32) -> Option<Vec2> {
        {
            let polygon_idx_test = self.get_point_location_ignore_delta(point);
            if polygon_idx_test != u32::MAX {
                debug_assert_eq!(polygon_idx, polygon_idx_test);
                // this point is already good to go
                return Some(point);
            }
        }
        let concave_polygon = &self.mesh.polygons[polygon_idx as usize];
        let concave_poly_center = concave_polygon.vertices
            .iter()
            .map(|idx| self.mesh.vertices[*idx as usize].coords)
            .sum::<Vec2>()
            / (concave_polygon.vertices.len() as f32);
        {
            let polygon_idx_test = self.get_point_location_ignore_delta(concave_poly_center);
            if polygon_idx_test == u32::MAX {
                // this should only happen because of floating point inaccuracies
                // and because the polygon was to small/thin
                return None;
            }
        }

        let p_to_c = concave_poly_center - point;

        macro_rules! return_if_in_mesh {
            ($factor:expr) => {{
                let between_edge_and_center = point + p_to_c * $factor;
                let polygon_idx_test = self.get_point_location_ignore_delta(between_edge_and_center);
                if polygon_idx_test != u32::MAX {
                    debug_assert_eq!(polygon_idx, polygon_idx_test);

                    if point_orig.distance_squared(between_edge_and_center) <= max_dist_sq {
                        return Some(between_edge_and_center);
                    }
                    return None;
                }
            }};
        }

        // it's kind of stupid, but well...
        // the order is important because of the max_dist_sq check
        return_if_in_mesh!(0.00001);
        return_if_in_mesh!(0.0001);
        return_if_in_mesh!(0.001);
        return_if_in_mesh!(0.01);
        return_if_in_mesh!(0.1);
        return_if_in_mesh!(0.25);
        return_if_in_mesh!(0.5);
        return_if_in_mesh!(0.75);

        // we checked earlier that the center is "within" the polygon
        Some(concave_poly_center)
    }

    /// Returns a vector with all polygon-edge intersections sorted by their distance to the given `from` point.
    #[inline(always)]
    fn all_line_intersections(&self, from: Vec2, to: Vec2) -> Vec<Vec2> {
        let line = geo::LineString::new(vec![Coord { x: from.x, y: from.y }, Coord { x: to.x, y: to.y }]);
        let line_bb = line.bounding_rect().unwrap();
        let mut intersections = Vec::new();

        // TODO: terribly inefficient, at least cache the polygons as line strips or sth.
        for poly in &self.mesh.polygons {
            let poly_strip = poly.vertices.iter().map(|v| {
                let vt = &self.mesh.vertices[*v as usize];
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

    /// Returns the closest point in the mesh to the given point that is outside the mesh.
    #[inline(always)]
    fn closest_exterior_point(&self, point_outside_mesh: Vec2, max_dist: f32, polygon_filter: impl Fn(usize) -> bool) -> Option<(Vec2, u32, f32)> {
        let max_dist_sq = max_dist * max_dist;
        let mut closest = None;
        let mut distance_squared = max_dist_sq;
        for (poly_idx, poly) in self.mesh.polygons.iter().enumerate() {
            if !polygon_filter(poly_idx) {
                // do not consider this polygon because of the filter predicate
                continue;
            }

            let iter_1 = poly.vertices.iter();
            let mut iter_2 = poly.vertices.iter();
            iter_2.next();
            for (p1i, p2i) in iter_1.zip(iter_2.chain(iter::once(poly.vertices.first().expect("polygon must not be empty"))))
                .map(|(a, b)| (*a as usize, *b as usize))
            {
                let p1 = self.mesh.vertices[p1i].coords;
                let p2 = self.mesh.vertices[p2i].coords;
                let on_line_segment = calc_projected_and_clipped_pos_on_line_segment(p1, p2, point_outside_mesh, EPSILON);
                let dist_sq = on_line_segment.distance_squared(point_outside_mesh);
                if dist_sq <= distance_squared { // use "<=" to overwrite possible "None" element
                    distance_squared = dist_sq;
                    closest = Some((on_line_segment, poly_idx as u32, max_dist_sq));
                }
            }
        }
        closest
    }

    /// Returns the closest point in the mesh to the given point that is outside the mesh.
    #[inline(always)]
    fn closest_exterior_point_line_edge(&self, point_outside_mesh: Vec2, max_dist: f32, polygon_filter: impl Fn(usize) -> bool) -> Option<(Vec2, u32, f32)> {
        let max_dist_sq = max_dist * max_dist;
        let mut closest = None;
        let mut distance_squared = max_dist_sq;
        for (poly_idx, poly) in self.mesh.polygons.iter().enumerate() {
            if !polygon_filter(poly_idx) {
                // do not consider this polygon because of the filter predicate
                continue;
            }

            for vertex_idx in poly.vertices.iter() {
                let p1 = self.mesh.vertices[*vertex_idx as usize].coords;
                let p1_dist_sq = p1.distance_squared(point_outside_mesh);
                if p1_dist_sq <= distance_squared { // use "<=" to overwrite possible "None" element
                    distance_squared = p1_dist_sq;
                    closest = Some((p1, poly_idx as u32, max_dist_sq));
                }

                let p2 = self.mesh.vertices[*vertex_idx as usize].coords;
                let p2_dist_sq = p2.distance_squared(point_outside_mesh);
                if p2_dist_sq <= distance_squared { // use "<=" to overwrite possible "None" element
                    distance_squared = p2_dist_sq;
                    closest = Some((p2, poly_idx as u32, max_dist_sq));
                }
            }
        }
        closest
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline(always)]
    fn get_point_location_ignore_delta(&self, point: Vec2) -> u32 {
        if self.mesh.baked_polygons.is_none() {
            self.mesh.get_point_location_unit(point)
        }
        else {
            self.mesh.get_point_location_unit_baked(point)
        }
    }
}




#[derive(Debug, Copy, Clone)]
enum LineSegmentProjection {
    OutsideSmallerP1,
    Inside,
    OutsideBiggerP2,
}
/// If the projected line segment is on either `p1` or `p2` the point is seen as [`LineSegmentProjection::Inside`].
#[inline(always)]
fn _calc_projected_pos_on_line_segment(p1: Vec2, p2: Vec2, to_be_projected_pt: Vec2) -> LineSegmentProjection {
    let p1_to_p2 = vec2(p2.x - p1.x, p2.y - p1.y);
    let line_segment_dot = p1_to_p2.dot(p1_to_p2);

    let p1_to_pr = vec2(to_be_projected_pt.x - p1.x, to_be_projected_pt.y - p1.y);
    let projection_dot = p1_to_p2.dot(p1_to_pr);

    if projection_dot < 0.0 {
        LineSegmentProjection::OutsideSmallerP1
    }
    else if projection_dot > line_segment_dot {
        LineSegmentProjection::OutsideBiggerP2
    }
    else {
        LineSegmentProjection::Inside
    }
}
/// Calculates the projection of the given point onto the given line.
/// If the given point would be outside the given line segment, it is clipped to p1 or p2 - depending on which point is
/// closer.
#[inline(always)]
fn calc_projected_and_clipped_pos_on_line_segment(p1: Vec2, p2: Vec2, to_be_projected_pt: Vec2, epsilon: f32) -> Vec2 {
    debug_assert!(epsilon > 0.0);

    let p1_to_p2 = vec2(p2.x - p1.x, p2.y - p1.y);
    let line_segment_dot = p1_to_p2.dot(p1_to_p2);

    let p1_to_pr = vec2(to_be_projected_pt.x - p1.x, to_be_projected_pt.y - p1.y);
    let projection_dot = p1_to_p2.dot(p1_to_pr);

    if projection_dot <= 0.0 + epsilon {
        p1
    }
    else if projection_dot >= line_segment_dot - epsilon {
        p2
    }
    else {
        let len_squared = p1_to_p2.x * p1_to_p2.x + p1_to_p2.y * p1_to_p2.y;
        p1 + (projection_dot * p1_to_p2) / len_squared
    }
}
/// Calculates the projection of the given point onto the given line.
#[inline(always)]
fn _project_point_onto_line(p1: Vec2, p2: Vec2, to_be_projected_pt: Vec2) -> Vec2 {
    let p1_to_p2 = vec2(p2.x - p1.x, p2.y - p1.y);
    let p1_to_pr = vec2(to_be_projected_pt.x - p1.x, to_be_projected_pt.y - p1.y);
    let projection_dot = p1_to_p2.dot(p1_to_pr);
    let len_squared = p1_to_p2.x * p1_to_p2.x + p1_to_p2.y * p1_to_p2.y;
    p1 + (projection_dot * p1_to_p2) / len_squared
}
