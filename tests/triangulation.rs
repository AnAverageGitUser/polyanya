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
    triangulation.queue_subtract(&[
        vec2(2.5, 2.5),
        vec2(2.5, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 2.5),
    ]);
    triangulation.update_mesh();
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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
    triangulation.queue_subtract(&[
        vec2(2.5, 2.5),
        vec2(2.5, 5.0),
        vec2(5.0, 5.0),
        vec2(5.0, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 5.0),
        vec2(2.5, 7.5),
        vec2(5.0, 7.5),
        vec2(5.0, 5.0),
    ]);
    triangulation.queue_subtract(&[
        vec2(5.0, 2.5),
        vec2(5.0, 5.0),
        vec2(7.5, 5.0),
        vec2(7.5, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(5.0, 5.0),
        vec2(5.0, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 5.0),
    ]);
    triangulation.update_mesh();
    triangulation.simplify(0.5);
    let mesh: Mesh = triangulation.as_navmesh().unwrap();

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
#[should_panic]
fn is_in_mesh_overlapping() {
    let mut triangulation = Triangulation::from_outer_edges(&[
        vec2(0.0, 0.0),
        vec2(10.0, 0.0),
        vec2(10.0, 10.0),
        vec2(0.0, 10.0),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 2.5),
        vec2(2.5, 6.0),
        vec2(6.0, 6.0),
        vec2(6.0, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 4.0),
        vec2(2.5, 7.5),
        vec2(6.0, 7.5),
        vec2(6.0, 4.0),
    ]);
    triangulation.queue_subtract(&[
        vec2(4.0, 2.5),
        vec2(4.0, 6.0),
        vec2(7.5, 6.0),
        vec2(7.5, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(4.0, 4.0),
        vec2(4.0, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 4.0),
    ]);
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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
    triangulation.queue_subtract(&[
        vec2(2.5, 2.5),
        vec2(2.5, 6.0),
        vec2(6.0, 6.0),
        vec2(6.0, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 4.0),
        vec2(2.5, 7.5),
        vec2(6.0, 7.5),
        vec2(6.0, 4.0),
    ]);
    triangulation.queue_subtract(&[
        vec2(4.0, 2.5),
        vec2(4.0, 6.0),
        vec2(7.5, 6.0),
        vec2(7.5, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(4.0, 4.0),
        vec2(4.0, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 4.0),
    ]);
    triangulation.update_mesh();
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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
    triangulation.queue_subtract(&[
        vec2(2.5, 2.5),
        vec2(2.5, 6.0),
        vec2(6.0, 6.0),
        vec2(6.0, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 4.0),
        vec2(2.5, 7.5),
        vec2(6.0, 7.5),
        vec2(6.0, 4.0),
    ]);
    triangulation.queue_subtract(&[
        vec2(4.0, 2.5),
        vec2(4.0, 6.0),
        vec2(7.5, 6.0),
        vec2(7.5, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(4.0, 4.0),
        vec2(4.0, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 4.0),
    ]);
    triangulation.update_mesh();
    let polygons_before = triangulation.as_navmesh().unwrap().polygons;
    triangulation.simplify(1.0);
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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

    triangulation.queue_subtract(&[
        vec2(0.0, 0.0),
        vec2(2.5, 0.0),
        vec2(2.5, 10.),
        vec2(0.0, 10.),
    ]);
    triangulation.queue_subtract(&[
        vec2(7.5, 0.0),
        vec2(10., 0.0),
        vec2(10., 10.),
        vec2(7.5, 10.),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 0.0),
        vec2(7.5, 0.0),
        vec2(7.5, 2.5),
        vec2(2.5, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 10.),
        vec2(2.5, 10.),
    ]);
    triangulation.update_mesh();
    let polygons_before = triangulation.as_navmesh().unwrap().polygons;
    triangulation.simplify(1.0);
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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

    triangulation.queue_subtract(&[
        vec2(2.5, 0.0),
        vec2(7.5, 0.0),
        vec2(7.5, 10.),
        vec2(2.5, 10.),
    ]);
    triangulation.update_mesh();
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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

    triangulation.queue_subtract(&[
        vec2(1.5, 1.5),
        vec2(2.5, 1.5),
        vec2(2.5, 8.5),
        vec2(1.5, 8.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(7.5, 1.5),
        vec2(8.5, 1.5),
        vec2(8.5, 8.5),
        vec2(7.5, 8.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 1.5),
        vec2(7.5, 1.5),
        vec2(7.5, 2.5),
        vec2(2.5, 2.5),
    ]);
    triangulation.queue_subtract(&[
        vec2(2.5, 7.5),
        vec2(7.5, 7.5),
        vec2(7.5, 8.5),
        vec2(2.5, 8.5),
    ]);

    triangulation.update_mesh();
    triangulation.simplify(0.0);
    let mesh: Mesh = triangulation.as_navmesh().unwrap();
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
        triangulation.queue_subtract(&[
            vec2(obstacle[0], obstacle[1]),
            vec2(obstacle[2], obstacle[3]),
            vec2(obstacle[4], obstacle[5]),
            vec2(obstacle[6], obstacle[7]),
        ]);
    }

    triangulation.update_mesh();
    triangulation.simplify(0.0);
    assert!(triangulation.as_navmesh().is_some());
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
        triangulation.queue_subtract(&[
            vec2(obstacle[0], obstacle[1]),
            vec2(obstacle[2], obstacle[3]),
            vec2(obstacle[4], obstacle[5]),
            vec2(obstacle[6], obstacle[7]),
        ]);
    }

    triangulation.update_mesh();
    triangulation.simplify(0.0);
    assert!(triangulation.as_navmesh().is_some());
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
        triangulation.queue_subtract(&[
            vec2(obstacle[0], obstacle[1]),
            vec2(obstacle[2], obstacle[3]),
            vec2(obstacle[4], obstacle[5]),
            vec2(obstacle[6], obstacle[7]),
        ]);
    }

    triangulation.update_mesh();
    triangulation.simplify(0.0);
    assert!(triangulation.as_navmesh().is_some());
}
