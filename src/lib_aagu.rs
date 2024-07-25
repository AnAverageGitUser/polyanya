use std::iter;
use glam::{Vec2, vec2};
use log::error;
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
pub enum DifferentIslandMode {
    /// If the (possibly corrected) start point and the (possibly corrected) end point are on different islands,
    /// return no path.
    NoPath,
    /// If the (possibly corrected) start point and the (possibly corrected) end point are on different islands,
    /// change the end point to the closest point that is on the same island as the (possibly corrected) start point and
    /// adheres to the end points [`PointCorrectionMode`].
    ///
    /// If not such end point can be found, return no path.
    ClosestOnStartIsland,
}
/// The settings for the navigation algorithm.
/// Certain situation can be configured to be handled differently.
#[derive(Debug, Copy, Clone)]
pub struct NavigationRequestSettings {
    /// The [`PointCorrectionMode`] of this navigation request's start point.
    pub start: PointCorrectionMode,
    /// The [`DifferentIslandMode`] of this navigation request.
    pub islands: DifferentIslandMode,
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
impl Default for NavigationRequestSettings {
    fn default() -> Self {
        Self {
            start: PointCorrectionMode::ClosestPointInMesh(f32::INFINITY),
            islands: DifferentIslandMode::ClosestOnStartIsland,
            end: PointCorrectionMode::ClosestPointInMesh(f32::INFINITY),
            _agent_radius: 0.0,
            _max_length: 0.0,
        }
    }
}

/// A status indication if a start/end point was modified and if yes to what.
#[derive(Debug, Copy, Clone)]
pub enum PointStatus {
    /// The point was not modified.
    Original(Vec2),
    /// The point was modified according to the given setting.
    Modified {
        /// The original point that was not within the mesh.
        original: Vec2,
        /// The modified point that is within the mesh.
        modified: Vec2,
    },
}
/// TODO
#[derive(Debug, Copy, Clone)]
pub enum NavigationResultStatus {
    /// The (possibly corrected) start point and (possibly corrected) end point are within the same mesh islands and
    /// a path was found.
    PathFound,

    /// The start- and/or end-points were not within the mesh and could not be rectified because of their
    /// [`PointCorrectionMode`].
    ///
    /// No path is given as a result.
    OutsideMesh,

    /// The (possibly corrected) start point and (possibly corrected) end point are on different mesh islands.
    /// If a path is returned and what path is returned depends on the given [`DifferentIslandMode`].
    DifferentIslands,

    /// The start- and end-point are within the same mesh islands, but a path could not be found and deadlock prevention
    /// kicked in.
    /// This is a bug, but does not seem to happen frequently enough to be important.
    ///
    /// No path is given as a result.
    BugInfinitePrevention,

    /// The navigation algorithm has a bug: it can not handle some situations due to floating point inaccuracies.
    /// This is specific to the shape and size of the navmesh's polygons.
    /// Extremely small and or extremely thin polygons could trigger this behaviour.
    ///
    /// No path is given as a result.
    BugFloatingPointInaccuracies,
}
impl NavigationResultStatus {
    /// True if the situation is expected to occur.
    pub fn is_valid_situation(&self) -> bool {
        match self {
            NavigationResultStatus::PathFound => {true}
            NavigationResultStatus::OutsideMesh => {true}
            NavigationResultStatus::DifferentIslands => {true}
            NavigationResultStatus::BugInfinitePrevention => {false}
            NavigationResultStatus::BugFloatingPointInaccuracies => {false}
        }
    }
}
/// TODO
#[derive(Debug, Copy, Clone)]
pub struct PathApproxResultStatus {
    /// TODO
    pub start: PointStatus,
    /// TODO
    pub status: NavigationResultStatus,
    /// TODO
    pub end: PointStatus,
}
impl PathApproxResultStatus {
    /// Constructor
    pub fn new(start: PointStatus, end: PointStatus, status: NavigationResultStatus) -> Self {
        Self {
            start,
            status,
            end,
        }
    }
    /// Build like setter for status.
    pub fn with_status(mut self, status: NavigationResultStatus) -> Self {
        self.status = status;
        self
    }
    /// Build like setter for start.
    pub fn with_start(mut self, start: PointStatus) -> Self {
        self.start = start;
        self
    }
    /// Build like setter for end.
    pub fn with_end(mut self, end: PointStatus) -> Self {
        self.end = end;
        self
    }
}

/// TODO
#[derive(Debug)]
pub struct PathApproxResult {
    /// TODO
    pub path: Option<Path>,
    /// TODO
    pub status: PathApproxResultStatus,
}
impl PathApproxResult {
    /// Gets the last element of the path: the destination.
    pub fn get_end(&self) -> &Vec2 {
        self.path.as_ref().expect("expected path to exist").path.last().expect("expected path to never be empty")
    }
}

fn new_possibly_corrected_path(from: Vec2, to: Vec2, from_orig: Vec2) -> Path {
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

/// A navigation mesh
#[derive(Debug, Clone)]
pub struct MeshAagu<'m> {
    pub(crate) mesh: &'m Mesh,
}
impl<'m> MeshAagu<'m> {
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
        let mut status = PathApproxResultStatus{
            start: PointStatus::Original(from_orig),
            status: NavigationResultStatus::PathFound,
            end: PointStatus::Original(to_orig),
        };

        let any_island = |_| true;
        if starting_polygon_idx == u32::MAX {
            match self.fix_outside_mesh_point(&settings.start, from, any_island) {
                Ok(res) => { (from, starting_polygon_idx) = res; }
                Err(nav_res_status) => {
                    return PathApproxResult { path: None, status: status.with_status(nav_res_status) };
                }
            }
            if from != from_orig {
                status = status.with_start(PointStatus::Modified { original: from_orig, modified: from })
            }
        }
        debug_assert_ne!(starting_polygon_idx, u32::MAX);

        let islands = self.mesh.islands.as_ref().expect("island baking is a prerequisite");
        let starting_island = islands[starting_polygon_idx as usize];

        match settings.islands {
            DifferentIslandMode::NoPath => {
                if ending_polygon_idx == u32::MAX {
                    // use any_island, if a different island is closer, we must know it, as this was requested
                    match self.fix_outside_mesh_point(&settings.end, to, any_island) {
                        Ok(res) => { (to, ending_polygon_idx) = res; }
                        Err(nav_res_status) => {
                            return PathApproxResult { path: None, status: status.with_status(nav_res_status) };
                        }
                    }
                    if to != to_orig {
                        status = status.with_end(PointStatus::Modified { original: to_orig, modified: to })
                    }
                    if starting_island != islands[ending_polygon_idx as usize] {
                        // in this case, the corrected target point got snapped to its closest island
                        // but the island was different to the start island
                        return PathApproxResult {
                            path: None,
                            status: status.with_status(NavigationResultStatus::DifferentIslands),
                        }
                    }
                }
                else {
                    if islands[ending_polygon_idx as usize] != starting_island {
                        return PathApproxResult {
                            path: None,
                            status: status.with_status(NavigationResultStatus::DifferentIslands),
                        };
                    }
                }
            }
            DifferentIslandMode::ClosestOnStartIsland => {
                if ending_polygon_idx == u32::MAX || islands[ending_polygon_idx as usize] != starting_island {
                    let same_island_as_start = |poly_idx: usize| {
                        islands[poly_idx] == starting_island
                    };
                    match self.fix_outside_mesh_point(&settings.end, to, same_island_as_start) {
                        Ok(res) => { (to, ending_polygon_idx) = res; }
                        Err(nav_res_status) => {
                            return PathApproxResult { path: None, status: status.with_status(nav_res_status) };
                        }
                    }
                    if to != to_orig {
                        status = status.with_end(PointStatus::Modified { original: to_orig, modified: to })
                    }
                }
            }
        }
        debug_assert_ne!(ending_polygon_idx, u32::MAX);

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
                path: Some(new_possibly_corrected_path(from, to, from_orig)),
                status: status.with_status(NavigationResultStatus::PathFound),
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
            match search_instance.next(false) {
                InstanceStep::Found(path) => {
                    return PathApproxResult { path: Some(path), status: status.with_status(NavigationResultStatus::PathFound) };
                },
                InstanceStep::NotFound => {
                    error!("Search from {from_orig} (corrected {from}) to {to_orig} (corrected {to}) failed. Please check if the mesh is valid as this should not happen as we've made sure that the two point are within the same mesh island.");
                    return PathApproxResult { path: None, status: status.with_status(NavigationResultStatus::OutsideMesh) }
                }
                InstanceStep::Continue => (),
            }
        }

        error!("Search from {from_orig} (corrected {from}) to {to_orig} (corrected {to}) failed. Please check if the mesh is valid as this should not happen. Infinite prevention triggered.");
        PathApproxResult { path: None, status: status.with_status(NavigationResultStatus::BugInfinitePrevention) }
    }

    fn fix_outside_mesh_point(
        &self,
        mode: &PointCorrectionMode,
        outside_mesh_point: Vec2,
        polygon_filter: impl Fn(usize) -> bool,
    )
        -> Result<(Vec2, u32), NavigationResultStatus>
    {
        let corrected_opt = match mode {
            PointCorrectionMode::NoCorrection => {
                return Err(NavigationResultStatus::OutsideMesh);
            },
            PointCorrectionMode::ClosestPointInMesh(max_dist) => {
                if *max_dist == 0.0 {
                    return Err(NavigationResultStatus::OutsideMesh);
                }
                self.closest_exterior_point(outside_mesh_point, *max_dist, polygon_filter)
            }
            PointCorrectionMode::ClosestMeshEdge(max_dist) => {
                if *max_dist == 0.0 {
                    return Err(NavigationResultStatus::OutsideMesh);
                }
                self.closest_exterior_point_line_edge(outside_mesh_point, *max_dist, polygon_filter)
            }
        };

        let Some((possibly_corrected_start_point, new_polygon_index, max_dist_sq)) = corrected_opt else {
            return Err(NavigationResultStatus::OutsideMesh)
        };

        debug_assert_ne!(new_polygon_index, u32::MAX, "The point was fixed, therefore a polygon with which it was fixed must have been found.");
        if let Some(possibly_corrected_start_point) = self.fix_intersection_point(outside_mesh_point, possibly_corrected_start_point, new_polygon_index, max_dist_sq) {
            Ok((possibly_corrected_start_point, new_polygon_index))
        }
        else {
            // this should only happen because of floating point inaccuracies, and because the polygon was to small/thin
            // if even the correction for the correction failed, we give up - this hopefully almost never happens
            Err(NavigationResultStatus::BugFloatingPointInaccuracies)
        }
    }

    fn fix_intersection_point(&self, point_orig: Vec2, point: Vec2, polygon_idx: u32, max_dist_sq: f32) -> Option<Vec2> {
        {
            let polygon_idx_test = self.get_point_location_ignore_delta(point);
            if polygon_idx_test != u32::MAX {
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
