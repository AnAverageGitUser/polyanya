use std::assert_matches;
use glam::vec2;
use polyanya::{DifferentIslandMode, Mesh, MeshAagu, NavigationRequestSettings, NavigationResultStatus, PointCorrectionMode, PointStatus, Triangulation};
use std::fs;
use std::str::FromStr;

#[test]
fn is_in_mesh_overlapping_simplified_2() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 2.5),
        vec2(2.5, 6.0),
        vec2(6.0, 6.0),
        vec2(6.0, 2.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 4.0),
        vec2(2.5, 7.5),
        vec2(6.0, 7.5),
        vec2(6.0, 4.0),
    ]);
    triangulation.add_obstacle(vec![
        vec2(4.0, 2.5),
        vec2(4.0, 6.0),
        vec2(7.5, 6.0),
        vec2(7.5, 2.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(4.0, 4.0),
        vec2(4.0, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 4.0),
    ]);
    let polygons_before = triangulation.as_navmesh().layers[0].polygons.clone();
    triangulation.simplify(1.0);
    let mesh: Mesh = triangulation.as_navmesh();
    assert!(dbg!(polygons_before.len()) >= dbg!(mesh.layers[0].polygons.len()));
    for i in 0..10 {
        for j in 0..10 {
            if i > 2 && i < 8 && j > 2 && j < 8 {
                assert!(!mesh.point_in_mesh(vec2(i as f32, j as f32)));
            } else {
                assert!(mesh.point_in_mesh(vec2(i as f32, j as f32)));
            }
        }
    }
}

#[test]
fn is_in_mesh_merge_overlapping_of_inner_bounds_can_shrink_the_outer_bounds() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);

    triangulation.add_obstacle(vec![
        vec2(0.0, 0.0),
        vec2(2.5, 0.0),
        vec2(2.5, 10.),
        vec2(0.0, 10.),
    ]);
    triangulation.add_obstacle(vec![
        vec2(7.5, 0.0),
        vec2(10., 0.0),
        vec2(10., 10.),
        vec2(7.5, 10.),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 0.0),
        vec2(7.5, 0.0),
        vec2(7.5, 2.5),
        vec2(2.5, 2.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 10.),
        vec2(2.5, 10.),
    ]);
    let polygons_before = triangulation.as_navmesh().layers[0].polygons.clone();
    triangulation.simplify(1.0);
    let mesh: Mesh = triangulation.as_navmesh();
    assert!(dbg!(polygons_before.len()) >= dbg!(mesh.layers[0].polygons.len()));
    for i in 0..=10 {
        for j in 0..=10 {
            if i > 2 && i < 8 && j > 2 && j < 8 {
                // println!("({}, {}) should be IN", i, j);
                assert!(mesh.point_in_mesh(vec2(i as f32, j as f32)));
            } else {
                // println!("({}, {}) should be OUT", i, j);
                assert!(!mesh.point_in_mesh(vec2(i as f32, j as f32)));
            }
        }
    }
}

#[test]
fn is_in_mesh_with_divided_islands() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);

    triangulation.add_obstacle(vec![
        vec2(2.5, 0.0),
        vec2(7.5, 0.0),
        vec2(7.5, 10.),
        vec2(2.5, 10.),
    ]);
    let mesh: Mesh = triangulation.as_navmesh();
    for x in 0..=10 {
        for y in 0..=10 {
            if x < 3 || x > 7 {
                // println!("({}, {}) should be IN", x, y);
                assert!(mesh.point_in_mesh(vec2(x as f32, y as f32)));
            } else {
                // println!("({}, {}) should be OUT", x, y);
                assert!(!mesh.point_in_mesh(vec2(x as f32, y as f32)));
            }
        }
    }
}

#[test]
fn is_in_mesh_with_border_and_middle_island() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);

    triangulation.add_obstacle(vec![
        vec2(1.5, 1.5),
        vec2(2.5, 1.5),
        vec2(2.5, 8.5),
        vec2(1.5, 8.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(7.5, 1.5),
        vec2(8.5, 1.5),
        vec2(8.5, 8.5),
        vec2(7.5, 8.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 1.5),
        vec2(7.5, 1.5),
        vec2(7.5, 2.5),
        vec2(2.5, 2.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 8.5),
        vec2(2.5, 8.5),
    ]);

    triangulation.simplify(0.0);
    let mesh: Mesh = triangulation.as_navmesh();
    for x in 0..=10 {
        for y in 0..=10 {
            if x < 2 || x > 8 || y < 2 || y > 8 || (x > 2 && x < 8 && y > 2 && y < 8) {
                println!("({}, {}) should be IN", x, y);
                assert!(mesh.point_in_mesh(vec2(x as f32, y as f32)));
            } else {
                println!("({}, {}) should be OUT", x, y);
                assert!(!mesh.point_in_mesh(vec2(x as f32, y as f32)));
            }
        }
    }
}

/// This crashed because of lines crossing when trying to build the CDT.
/// This data has a passable path mesh island in its middle.
#[cfg(not(debug_assertions))] // This test is very slow, use cargo test --release.
#[test]
fn previous_crash_case_1() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(512.0, 0.0),
        vec2(512.0, 512.0),
        vec2(0.0, 512.0),
    ]);

    for line in fs::read_to_string("meshes/previous_crash_case_1.txt")
        .expect("Should have been able to read the file")
        .lines()
    {
        let obstacle: Vec<f32> = line.split(" ").map(|v| f32::from_str(v).unwrap()).collect();
        assert_eq!(obstacle.len(), 8);
        triangulation.add_obstacle(vec![
            vec2(obstacle[0], obstacle[1]),
            vec2(obstacle[2], obstacle[3]),
            vec2(obstacle[4], obstacle[5]),
            vec2(obstacle[6], obstacle[7]),
        ]);
    }

    triangulation.simplify(0.0);
    triangulation.as_navmesh();
}

#[cfg(not(debug_assertions))] // This test is slow, use cargo test --release.
#[test]
fn previous_crash_case_2() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(512.0, 0.0),
        vec2(512.0, 512.0),
        vec2(0.0, 512.0),
    ]);

    for line in fs::read_to_string("meshes/previous_crash_case_2.txt")
        .expect("Should have been able to read the file")
        .lines()
    {
        let obstacle: Vec<f32> = line.split(" ").map(|v| f32::from_str(v).unwrap()).collect();
        assert_eq!(obstacle.len(), 8);
        triangulation.add_obstacle(vec![
            vec2(obstacle[0], obstacle[1]),
            vec2(obstacle[2], obstacle[3]),
            vec2(obstacle[4], obstacle[5]),
            vec2(obstacle[6], obstacle[7]),
        ]);
    }

    triangulation.simplify(0.0);
    triangulation.as_navmesh();
}

#[test]
fn previous_crash_case_3() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(512.0, 0.0),
        vec2(512.0, 512.0),
        vec2(0.0, 512.0),
    ]);

    for line in fs::read_to_string("meshes/previous_crash_case_3.txt")
        .expect("Should have been able to read the file")
        .lines()
    {
        let obstacle: Vec<f32> = line.split(" ").map(|v| f32::from_str(v).unwrap()).collect();
        assert_eq!(obstacle.len(), 8);
        triangulation.add_obstacle(vec![
            vec2(obstacle[0], obstacle[1]),
            vec2(obstacle[2], obstacle[3]),
            vec2(obstacle[4], obstacle[5]),
            vec2(obstacle[6], obstacle[7]),
        ]);
    }

    triangulation.simplify(0.0);
    triangulation.as_navmesh();
}

// ---- MeshAagu integration tests ----

fn make_obstacle_mesh() -> Mesh {
    let mut tri = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(20.0, 0.0),
        vec2(20.0, 20.0),
        vec2(0.0, 20.0),
    ]);
    tri.add_obstacle(vec![
        vec2(8.0, 8.0),
        vec2(8.0, 12.0),
        vec2(12.0, 12.0),
        vec2(12.0, 8.0),
    ]);
    let mut mesh = tri.as_navmesh();
    mesh.bake();
    mesh
}

fn make_two_island_mesh() -> Mesh {
    let mut tri = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(20.0, 0.0),
        vec2(20.0, 20.0),
        vec2(0.0, 20.0),
    ]);
    tri.add_obstacle(vec![
        vec2(9.0, 0.0),
        vec2(11.0, 0.0),
        vec2(11.0, 20.0),
        vec2(9.0, 20.0),
    ]);
    let mut mesh = tri.as_navmesh();
    mesh.bake();
    mesh
}

#[test]
fn approx_path_both_inside_finds_path() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let result = aagu.approx_path(
        vec2(2.0, 10.0),
        vec2(18.0, 10.0),
        NavigationRequestSettings::default(),
    );
    assert!(result.path.is_some());
    let path = result.path.unwrap();
    assert!(path.length > vec2(2.0, 10.0).distance(vec2(18.0, 10.0)));
    assert_eq!(*path.path.last().unwrap(), vec2(18.0, 10.0));
}

#[test]
fn approx_path_start_outside_corrected() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        start: PointCorrectionMode::ClosestMeshEdge(f32::INFINITY),
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(-2.0, 10.0), vec2(18.0, 10.0), settings);
    assert!(result.path.is_some());
    assert_matches!(result.status.start, PointStatus::Modified { .. });
}

#[test]
fn approx_path_end_outside_corrected_closest_edge() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        end: PointCorrectionMode::ClosestMeshEdge(f32::INFINITY),
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(2.0, 10.0), vec2(25.0, 10.0), settings);
    assert!(result.path.is_some());
    assert_matches!(result.status.end, PointStatus::Modified { .. });
}

#[test]
fn approx_path_end_outside_corrected_closest_vertex() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        end: PointCorrectionMode::ClosestMeshVertex(f32::INFINITY),
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(2.0, 10.0), vec2(25.0, 10.0), settings);
    assert!(result.path.is_some());
    assert_matches!(result.status.end, PointStatus::Modified { .. });
}

#[test]
fn approx_path_no_correction_start_outside_fails() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        start: PointCorrectionMode::NoCorrection,
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(-2.0, 10.0), vec2(18.0, 10.0), settings);
    assert!(result.path.is_none());
    assert_matches!(
        result.status.status,
        NavigationResultStatus::OutsideMesh
    );
}

#[test]
fn approx_path_different_islands_no_path_mode() {
    let mesh = make_two_island_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        islands: DifferentIslandMode::NoPath,
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(2.0, 10.0), vec2(18.0, 10.0), settings);
    assert!(result.path.is_none());
    assert_matches!(
        result.status.status,
        NavigationResultStatus::DifferentIslands
    );
}

#[test]
fn approx_path_different_islands_closest_on_start() {
    let mesh = make_two_island_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        islands: DifferentIslandMode::ClosestOnStartIsland,
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(2.0, 10.0), vec2(18.0, 10.0), settings);
    assert!(result.path.is_some());
    let end = *result.path.as_ref().unwrap().path.last().unwrap();
    assert!(end.x <= 9.0 + 0.1);
}

#[test]
fn approx_path_same_island_no_path_mode_succeeds() {
    let mesh = make_two_island_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        islands: DifferentIslandMode::NoPath,
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(2.0, 2.0), vec2(2.0, 18.0), settings);
    assert!(result.path.is_some());
}

#[test]
fn approx_path_max_dist_too_small_for_correction() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        start: PointCorrectionMode::ClosestMeshEdge(0.5),
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(-5.0, 10.0), vec2(18.0, 10.0), settings);
    assert!(result.path.is_none());
}

#[test]
fn get_island_id_integration() {
    let mesh = make_two_island_mesh();
    let aagu = MeshAagu::new(&mesh);
    let left = aagu.get_island_id(vec2(2.0, 10.0));
    let right = aagu.get_island_id(vec2(18.0, 10.0));
    let obstacle = aagu.get_island_id(vec2(10.0, 10.0));
    let outside = aagu.get_island_id(vec2(-5.0, -5.0));
    assert!(left.is_some());
    assert!(right.is_some());
    assert_ne!(left, right);
    assert!(obstacle.is_none());
    assert!(outside.is_none());
}

#[test]
fn approx_path_end_in_obstacle_corrected() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        end: PointCorrectionMode::ClosestMeshEdge(f32::INFINITY),
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(2.0, 10.0), vec2(10.0, 10.0), settings);
    assert!(result.path.is_some());
    assert_matches!(result.status.end, PointStatus::Modified { .. });
}

#[test]
fn approx_path_both_outside_corrected() {
    let mesh = make_obstacle_mesh();
    let aagu = MeshAagu::new(&mesh);
    let settings = NavigationRequestSettings {
        start: PointCorrectionMode::ClosestMeshEdge(f32::INFINITY),
        end: PointCorrectionMode::ClosestMeshEdge(f32::INFINITY),
        ..Default::default()
    };
    let result = aagu.approx_path(vec2(-2.0, 10.0), vec2(25.0, 10.0), settings);
    assert!(result.path.is_some());
    assert_matches!(result.status.start, PointStatus::Modified { .. });
    assert_matches!(result.status.end, PointStatus::Modified { .. });
}

/// A wall from the bottom edge up to y = 8 between x = 4 and 6, in a 10 × 10 square.
fn make_wall_mesh() -> Mesh {
    let mut tri = Triangulation::from_outer_edges(&[vec2(0.0, 0.0), vec2(10.0, 0.0), vec2(10.0, 10.0), vec2(0.0, 10.0)]);
    tri.add_obstacle(vec![vec2(4.0, 0.0), vec2(6.0, 0.0), vec2(6.0, 8.0), vec2(4.0, 8.0)]);
    let mut mesh = tri.as_navmesh();
    mesh.bake();
    mesh
}

fn path_length(start: glam::Vec2, points: &[glam::Vec2]) -> f32 {
    std::iter::once(&start).chain(points).zip(points).map(|(a, b)| a.distance(*b)).sum()
}

/// a corrected start is the first point of a searched path, and the length
/// includes the way from the original start to it — as in the same-polygon shortcut.
#[test]
fn approx_path_corrected_start_is_the_first_point_of_a_searched_path() {
    let mesh = make_wall_mesh();
    let aagu = MeshAagu::new(&mesh);
    let (from, to) = (vec2(4.5, 4.0), vec2(9.0, 1.0)); // inside the wall near its left side, end behind it
    let result = aagu.approx_path(from, to, NavigationRequestSettings::default());
    let PointStatus::Modified { modified, .. } = result.status.start else { panic!("start corrected: {:?}", result.status) };
    let path = result.path.expect("path");
    assert!(path.path.len() > 2, "a searched path (around the wall): {:?}", path.path);
    assert_eq!(path.path[0], modified, "{:?}", path.path);
    assert_eq!(*path.path.last().unwrap(), to);
    assert!((path.length - path_length(from, &path.path)).abs() < 0.001, "{} vs {:?}", path.length, path.path);
}

/// The same-polygon shortcut and the search agree: the path starts with the corrected start and the length counts it.
#[test]
fn approx_path_corrected_start_same_polygon_counts_the_correction() {
    let mesh = make_wall_mesh();
    let aagu = MeshAagu::new(&mesh);
    let (from, to) = (vec2(10.5, 9.5), vec2(9.9, 9.9)); // just outside the outer edge, end next to it
    let result = aagu.approx_path(from, to, NavigationRequestSettings::default());
    let PointStatus::Modified { modified, .. } = result.status.start else { panic!("start corrected: {:?}", result.status) };
    let path = result.path.expect("path");
    assert_eq!(path.path, vec![modified, to]);
    assert!((path.length - path_length(from, &path.path)).abs() < 0.001);
}

/// A correction too small to matter (the agent stands on the mesh border) adds no extra point: no zero-length first
/// segment for the caller.
#[test]
fn approx_path_tiny_start_correction_adds_no_point() {
    let mesh = make_wall_mesh();
    let aagu = MeshAagu::new(&mesh);
    let to = vec2(9.0, 1.0);
    let from = vec2(4.0 + 0.00001, 4.0); // on the wall's edge, a hair inside the wall
    let result = aagu.approx_path(from, to, NavigationRequestSettings::default());
    let path = result.path.expect("path");
    if let PointStatus::Modified { modified, .. } = result.status.start {
        assert!(modified.distance(from) < 0.001, "tiny correction: {modified}");
        assert_ne!(path.path[0], modified, "no near-zero first segment: {:?}", path.path);
    }
    assert_eq!(*path.path.last().unwrap(), to);
}

/// `ClosestMeshEdge` snaps to the closest edge point, `ClosestMeshVertex` to the
/// closest polygon corner — in the middle of a long wall these are far apart.
#[test]
fn closest_mesh_edge_snaps_to_the_edge_closest_mesh_vertex_to_a_corner() {
    let mesh = make_wall_mesh(); // wall x 4..6, y 0..8
    let aagu = MeshAagu::new(&mesh);
    let (from, to) = (vec2(4.5, 4.0), vec2(1.0, 1.0)); // in the wall near its left side, end left of it
    let snapped = |mode| {
        let result = aagu.approx_path(from, to, NavigationRequestSettings { start: mode, ..Default::default() });
        let PointStatus::Modified { modified, .. } = result.status.start else { panic!("start corrected: {:?}", result.status) };
        modified
    };
    let edge = snapped(PointCorrectionMode::ClosestMeshEdge(f32::INFINITY));
    assert!(edge.distance(vec2(4.0, 4.0)) < 0.01, "beside the start on the wall's left side: {edge}");
    let vertex = snapped(PointCorrectionMode::ClosestMeshVertex(f32::INFINITY));
    assert!(vertex.distance(from) > 3.0, "a corner, far from the start: {vertex}");
    assert!(vertex.x <= 4.0 + 0.01 || vertex.y >= 8.0 - 0.01, "a corner outside the wall: {vertex}");
}

fn square_triangulation() -> Triangulation {
    Triangulation::from_outer_edges(&[vec2(0.0, 0.0), vec2(10.0, 0.0), vec2(10.0, 10.0), vec2(0.0, 10.0)])
}

/// Points are in mesh coordinates: a layer's offset is taken into account (as `Mesh::point_in_mesh` does).
#[test]
fn approx_path_respects_the_layer_offset() {
    let mut tri = square_triangulation();
    tri.add_obstacle(vec![vec2(4.0, 0.0), vec2(6.0, 0.0), vec2(6.0, 8.0), vec2(4.0, 8.0)]);
    let mut mesh = tri.as_navmesh();
    mesh.layers[0].offset = vec2(100.0, 0.0);
    mesh.bake();
    let aagu = MeshAagu::new(&mesh);
    let (from, to) = (vec2(101.0, 1.0), vec2(109.0, 1.0));
    assert!(mesh.point_in_mesh(from) && mesh.point_in_mesh(to));
    let result = aagu.approx_path(from, to, NavigationRequestSettings::default());
    assert_matches!(result.status.start, PointStatus::Original(_));
    assert_matches!(result.status.end, PointStatus::Original(_));
    let path = result.path.expect("path around the wall");
    assert_eq!(*path.path.last().unwrap(), to);
    assert!(path.path.len() >= 3, "around the wall: {:?}", path.path);
    assert!(aagu.get_island_id(from).is_some());
    // a point in the wall is corrected to the wall's border, in mesh coordinates
    let result = aagu.approx_path(from, vec2(104.5, 4.0), NavigationRequestSettings::default());
    let PointStatus::Modified { modified, .. } = result.status.end else { panic!("{:?}", result.status) };
    assert!(modified.distance(vec2(104.0, 4.0)) < 0.01, "{modified}");
}

/// An unbaked mesh (no islands, no polygon finder) gives a result instead of panicking.
#[test]
fn approx_path_on_an_unbaked_mesh() {
    let mut tri = square_triangulation();
    tri.add_obstacle(vec![vec2(4.0, 0.0), vec2(6.0, 0.0), vec2(6.0, 8.0), vec2(4.0, 8.0)]);
    let mesh = tri.as_navmesh(); // not baked
    let aagu = MeshAagu::new(&mesh);
    let result = aagu.approx_path(vec2(1.0, 1.0), vec2(9.0, 1.0), NavigationRequestSettings::default());
    assert!(result.path.is_some(), "{:?}", result.status);
    let result = aagu.approx_path(vec2(1.0, 1.0), vec2(4.5, 4.0), NavigationRequestSettings::default());
    assert!(result.path.is_some(), "end in the wall is corrected: {:?}", result.status);
}

/// Islands are per layer: a point on another layer than the start counts as another island.
#[test]
fn approx_path_islands_on_two_layers() {
    let mut mesh = square_triangulation().as_navmesh();
    let mut other = square_triangulation().as_layer();
    other.offset = vec2(20.0, 0.0);
    mesh.layers.push(other);
    mesh.bake();
    let aagu = MeshAagu::new(&mesh);
    let (from, to) = (vec2(5.0, 5.0), vec2(25.0, 5.0));
    assert!(mesh.point_in_mesh(to));
    let settings = |islands| NavigationRequestSettings { islands, ..Default::default() };
    let result = aagu.approx_path(from, to, settings(DifferentIslandMode::NoPath));
    assert!(result.path.is_none());
    assert_matches!(result.status.status, NavigationResultStatus::DifferentIslands);
    let result = aagu.approx_path(from, to, settings(DifferentIslandMode::ClosestOnStartIsland));
    let PointStatus::Modified { modified, .. } = result.status.end else { panic!("{:?}", result.status) };
    assert!(modified.distance(vec2(10.0, 5.0)) < 0.01, "on the start's layer: {modified}");
    assert!(result.path.is_some());
}
