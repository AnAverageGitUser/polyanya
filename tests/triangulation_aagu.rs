use glam::vec2;
use polyanya::{Mesh, Triangulation};
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
