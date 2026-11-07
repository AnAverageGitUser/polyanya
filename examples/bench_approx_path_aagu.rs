//! Timing of `MeshAagu::approx_path` on a game-sized mesh.
//! An example instead of a `[[bench]]` so that the upstream `Cargo.toml` stays untouched.
//!
//! Run: `cargo run --release --example bench_approx_path_aagu`. Prints the mean time per call for each case and a
//! checksum of all results: an optimisation must not change any checksum.

use std::hint::black_box;
use std::time::Instant;

use glam::{vec2, Vec2};
use polyanya::{MeshAagu, NavigationRequestSettings, PathApproxResult, PointCorrectionMode, PointStatus, Triangulation};

/// 200 × 200 map with 30 × 30 = 900 obstacles of about 1 × 1 (like trees and stones), deterministic jitter.
fn game_like_mesh() -> (polyanya::Mesh, Vec<Vec2>) {
    let mut tri = Triangulation::from_outer_edges(&[vec2(0., 0.), vec2(200., 0.), vec2(200., 200.), vec2(0., 200.)]);
    let mut rng = Lcg(7);
    let mut centers = vec![];
    for i in 0..30 {
        for j in 0..30 {
            let c = vec2(5.0 + i as f32 * 6.5 + rng.f() * 2.0, 5.0 + j as f32 * 6.5 + rng.f() * 2.0);
            let h = 0.5 + rng.f() * 0.3;
            tri.add_obstacle(vec![c + vec2(-h, -h), c + vec2(-h, h), c + vec2(h, h), c + vec2(h, -h)]);
            centers.push(c);
        }
    }
    tri.simplify(0.1);
    let mut mesh = tri.as_navmesh();
    for _ in 0..3 {
        mesh.merge_polygons();
    }
    mesh.bake();
    (mesh, centers)
}

/// Name, queries (from, to), settings.
type Case<'a> = (&'a str, &'a [(Vec2, Vec2)], NavigationRequestSettings);

struct Lcg(u64);
impl Lcg {
    fn f(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / ((1u64 << 24) as f32)
    }
}

fn checksum(r: &PathApproxResult) -> u64 {
    let mut h = 1469598103934665603u64;
    let mut add = |v: f32| h = (h ^ v.to_bits() as u64).wrapping_mul(1099511628211);
    if let Some(p) = &r.path {
        p.path.iter().for_each(|q| { add(q.x); add(q.y) });
        add(p.length);
    }
    for s in [r.status.start, r.status.end] {
        if let PointStatus::Modified { modified, .. } = s { add(modified.x); add(modified.y) }
    }
    h
}

fn main() {
    let t = Instant::now();
    let (mesh, centers) = game_like_mesh();
    let polygons: usize = mesh.layers.iter().map(|l| l.polygons.len()).sum();
    println!("mesh: {} polygons, built in {:.1} ms", polygons, t.elapsed().as_secs_f64() * 1e3);
    let aagu = MeshAagu::new(&mesh);
    let mut rng = Lcg(42);
    let free = |rng: &mut Lcg| vec2(2.0 + rng.f() * 196.0, 2.0 + rng.f() * 196.0);
    let n = 2000;
    let mut free_pairs = vec![];
    let mut obstacle_targets = vec![];
    for _ in 0..n {
        free_pairs.push((free(&mut rng), free(&mut rng)));
        let c = centers[(rng.f() * centers.len() as f32) as usize % centers.len()];
        obstacle_targets.push((free(&mut rng), c));
    }
    let edge = |d| NavigationRequestSettings { end: PointCorrectionMode::ClosestMeshEdge(d), ..Default::default() };
    let vertex = |d| NavigationRequestSettings { end: PointCorrectionMode::ClosestMeshVertex(d), ..Default::default() };
    let both = NavigationRequestSettings {
        start: PointCorrectionMode::ClosestMeshEdge(1.0),
        end: PointCorrectionMode::ClosestMeshEdge(2.0),
        ..Default::default()
    };
    let swapped: Vec<_> = obstacle_targets.iter().map(|(a, b)| (*b, *a)).collect();
    let cases: [Case; 5] = [
        ("both in the mesh", &free_pairs[..], NavigationRequestSettings::default()),
        ("end in an obstacle, edge 2.0", &obstacle_targets[..], edge(2.0)),
        ("end in an obstacle, vertex 2.0", &obstacle_targets[..], vertex(2.0)),
        ("end in an obstacle, edge INFINITY", &obstacle_targets[..], edge(f32::INFINITY)),
        ("start in an obstacle (1.0), end free (2.0)", &swapped[..], both),
    ];
    for (name, queries, settings) in cases {
        let mut sum = 0u64;
        let t = Instant::now();
        for (from, to) in queries.iter() {
            let r = aagu.approx_path(black_box(*from), black_box(*to), settings);
            sum = sum.wrapping_add(checksum(&r));
        }
        let us = t.elapsed().as_secs_f64() * 1e6 / queries.len() as f64;
        println!("{name:<45} {us:>9.1} µs/call   checksum {sum:016x}");
    }
}
