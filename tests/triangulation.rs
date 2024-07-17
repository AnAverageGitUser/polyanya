use glam::vec2;
use polyanya::{Mesh, Triangulation};
use std::fs;
use std::str::FromStr;

#[test]
fn is_in_mesh() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 2.5),
        vec2(2.5, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 2.5),
    ]);
    let mesh: Mesh = triangulation.as_navmesh();
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
fn is_in_mesh_4_obstacles() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 2.5),
        vec2(2.5, 5.0),
        vec2(5.0, 5.0),
        vec2(5.0, 2.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(2.5, 5.0),
        vec2(2.5, 7.5),
        vec2(5.0, 7.5),
        vec2(5.0, 5.0),
    ]);
    triangulation.add_obstacle(vec![
        vec2(5.0, 2.5),
        vec2(5.0, 5.0),
        vec2(7.5, 5.0),
        vec2(7.5, 2.5),
    ]);
    triangulation.add_obstacle(vec![
        vec2(5.0, 5.0),
        vec2(5.0, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 5.0),
    ]);
    triangulation.simplify(0.5);
    let mesh: Mesh = triangulation.as_navmesh();

    dbg!(mesh.polygons.len());
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
fn is_in_mesh_overlapping() {
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
    let mesh: Mesh = triangulation.as_navmesh();
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
fn is_in_mesh_overlapping_merged() {
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
    let mesh: Mesh = triangulation.as_navmesh();
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
fn is_in_mesh_overlapping_simplified() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);
    // adding a circle obstacle in the middle
    let nb_points = 1000;
    let radius = 2.5;
    triangulation.add_obstacle(
        (0..nb_points)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::TAU / nb_points as f32;
                let (x, y) = angle.sin_cos();
                vec2(x, y) * radius + vec2(5.0, 5.0)
            })
            .collect(),
    );
    triangulation.add_obstacle(vec![
        vec2(2.5, 2.5),
        vec2(2.5, 5.0),
        vec2(5.0, 5.0),
        vec2(5.0, 2.5),
    ]);

    let mesh_before = triangulation.as_navmesh();
    triangulation.simplify(0.01);
    let mesh: Mesh = triangulation.as_navmesh();
    assert!(dbg!(mesh_before.polygons.len()) > dbg!(mesh.polygons.len()));
    let resolution = 5;
    for i in 0..(10 * resolution) {
        for j in 0..(10 * resolution) {
            let point = vec2(i as f32 / resolution as f32, j as f32 / resolution as f32);
            assert_eq!(mesh.point_in_mesh(point), mesh_before.point_in_mesh(point));
        }
    }
}

#[test]
fn is_in_mesh_simplified() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);
    // adding a circle obstacle in the middle
    let nb_points = 100;
    let radius = 2.5;
    triangulation.add_obstacle(
        (0..nb_points)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::TAU / nb_points as f32;
                let (x, y) = angle.sin_cos();
                vec2(x, y) * radius + vec2(5.0, 5.0)
            })
            .collect(),
    );
    let polygons_before = triangulation.as_navmesh().polygons;
    triangulation.simplify(0.1);
    let mesh: Mesh = triangulation.as_navmesh();
    assert!(dbg!(polygons_before.len()) > dbg!(mesh.polygons.len()));
    for i in 0..20 {
        for j in 0..20 {
            let point = vec2(i as f32 / 2.0, j as f32 / 2.0);
            if point.distance(vec2(5.0, 5.0)) < radius {
                assert!(!mesh.point_in_mesh(point));
            } else {
                assert!(mesh.point_in_mesh(point));
            }
        }
    }
}

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
    let polygons_before = triangulation.as_navmesh().polygons;
    triangulation.simplify(1.0);
    let mesh: Mesh = triangulation.as_navmesh();
    assert!(dbg!(polygons_before.len()) >= dbg!(mesh.polygons.len()));
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
    let polygons_before = triangulation.as_navmesh().polygons;
    triangulation.simplify(1.0);
    let mesh: Mesh = triangulation.as_navmesh();
    assert!(dbg!(polygons_before.len()) >= dbg!(mesh.polygons.len()));
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
///
/// This test is slow, either use cargo --release or use the feature-flag "wasm-incompatible" instead of
/// "wasm-compatible".
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
