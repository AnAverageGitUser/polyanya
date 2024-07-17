use geo::{BoundingRect, Coord, Intersects, LinesIter};
use glam::{Vec2, vec2};
use log::{error, warn};
use crate::instance::{InstanceStep, SearchInstance};
use crate::{Mesh, Path};

#[cfg(feature = "tracing")]
use tracing::instrument;

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

/// Creates a new object with the target as single waypoint.
pub fn new_direct_path(from: Vec2, to: Vec2) -> Path {
    Path {
        length: from.distance(to),
        path: vec![to]
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
    pub fn approx_path(&self, mut from: Vec2, mut to: Vec2) -> PathApproxResult {
        #[cfg(feature = "stats")]
        let start = std::time::Instant::now();

        let mut starting_polygon_index = self.get_point_location_ignore_delta(from);
        let mut ending_polygon_index = self.get_point_location_ignore_delta(to);
        let mut remove_first_waypoint = true;

        let status = if starting_polygon_index == u32::MAX {
            if ending_polygon_index == u32::MAX {
                return PathApproxResult {
                    path: new_direct_path(from, to),
                    status: PathApproxResultEnum::BothOutside,
                };
            }
            // move start onto the closest island along the direct start-end-line
            remove_first_waypoint = false;
            let intersections = self.all_line_intersections(from, to);
            if intersections.is_empty() {
                // better buggy movement than an application crash!
                return PathApproxResult {
                    path: new_direct_path(from, to),
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
                    path: new_direct_path(from, to),
                    status: PathApproxResultEnum::BugCrashPrevention,
                };
            }
            (ending_polygon_index, to) = self.approx_path_fix_end(starting_polygon_index, from, to, &intersections);
            PathApproxResultEnum::EndOutside
        }
        else {
            let islands = self.mesh.islands.as_ref().expect("islands must exist");
            let start_island = islands.get(starting_polygon_index as usize).expect("start point island must exist");
            let end_island = islands.get(ending_polygon_index as usize).expect("end point island must exist");
            if start_island != end_island {
                // move end onto the same island
                let intersections = self.all_line_intersections(from, to);
                if intersections.is_empty() {
                    // better buggy movement than an application crash!
                    return PathApproxResult {
                        path: new_direct_path(from, to),
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
                path: new_direct_path(from, to),
                status: PathApproxResultEnum::ValidPath,
            }
        }

        let mut search_instance = SearchInstance::setup(
            self.mesh,
            (from, starting_polygon_index),
            (to, ending_polygon_index),
            #[cfg(feature = "stats")]
            start,
        );

        // Limit search to avoid an infinite loop.
        for _ in 0..self.mesh.polygons.len() * 1000 {
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
                        path: new_direct_path(from, to),
                        status: PathApproxResultEnum::NoPath,
                    }
                }
                InstanceStep::Continue => (),
            }
        }

        error!("Search from {from} to {to} failed. Please check if the mesh is valid as this should not happen. Infinite prevention triggered.");
        PathApproxResult {
            path: new_direct_path(from, to),
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

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    fn get_point_location_ignore_delta(&self, point: Vec2) -> u32 {
        if self.mesh.baked_polygons.is_none() {
            self.mesh.get_point_location_unit(point)
        }
        else {
            self.mesh.get_point_location_unit_baked(point)
        }
    }
}