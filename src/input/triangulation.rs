#[cfg(any(
    not(any(feature = "wasm-compatible", feature = "wasm-incompatible")),
    all(feature = "wasm-compatible", feature = "wasm-incompatible")
))]
compile_error!(
    "You must choose exactly one of the features: [\"wasm-compatible\", \"wasm-incompatible\"]."
);

#[cfg(all(feature = "wasm-incompatible", not(feature = "wasm-compatible")))]
use geo_clipper::Clipper;

#[cfg(feature = "wasm-compatible")]
use geo_booleanop::boolean::BooleanOp;

#[cfg(feature = "tracing")]
use tracing::instrument;

use geo::{Contains, CoordsIter, SimplifyVwPreserve};
use geo_offset::Offset;
use glam::{vec2, Vec2};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation as SpadeTriangulation};
use std::collections::VecDeque;

pub use geo::{Coord, LineString, MultiPolygon, Polygon};

use crate::Mesh;

/// Keep the precision of 3 digits behind the decimal point (e.g. "25.219"), when using geo-clipper calculations.
#[cfg(all(feature = "wasm-incompatible", not(feature = "wasm-compatible")))]
const GEO_CLIPPER_CLIP_PRECISION: f32 = 1000.0;

/// TODO
#[derive(Debug, Clone)]
pub enum PolygonMeshSetOperation {
    /// TODO
    Add(MultiPolygon<f32>),
    /// TODO
    Subtract(MultiPolygon<f32>),
    // not certain if these will be needed
    // Union(MultiPolygon<f32>),
    // Intersection(MultiPolygon<f32>),
    // Difference(MultiPolygon<f32>),
}

/// An helper to create a [`Mesh`] from a list of edges and obstacle, using a constrained Delaunay triangulation.
#[derive(Debug, Clone)]
pub struct Triangulation {
    queued_operations: Vec<PolygonMeshSetOperation>,
    multi_poly_mesh: MultiPolygon<f32>,
    unit_radius: f32,
}

/// TODO
#[inline]
fn multi_polygon_union(
    multi_polygon: &MultiPolygon<f32>,
    polygon: &Polygon<f32>,
) -> MultiPolygon<f32> {
    #[cfg(all(feature = "wasm-incompatible", not(feature = "wasm-compatible")))]
    {
        multi_polygon.union(polygon, GEO_CLIPPER_CLIP_PRECISION)
    }
    #[cfg(feature = "wasm-compatible")]
    {
        multi_polygon.union(polygon)
    }
}

impl Triangulation {
    /// Create a new triangulation from a the list of points on its outer edges.
    pub fn from_outer_edges(shape: &[Vec2]) -> Triangulation {
        let outer_line = LineString::from(shape.iter().map(|v| (v.x, v.y)).collect::<Vec<_>>());
        let poly = Polygon::<f32>::new(outer_line, vec![]);
        let multi_poly = MultiPolygon::<f32>::new(vec![poly]);

        Self {
            queued_operations: vec![],
            multi_poly_mesh: multi_poly,
            unit_radius: 0.0,
        }
    }

    /// Add a shape to the triangulation history.
    /// The current island is marked as out of date.
    pub fn queue_add(&mut self, shape: &[Vec2]) {
        let outer_line = LineString::from(shape.iter().map(|v| (v.x, v.y)).collect::<Vec<_>>());
        let poly = Polygon::<f32>::new(outer_line, vec![]);

        if let Some(PolygonMeshSetOperation::Add(ref mut existing_polygon)) =
            self.queued_operations.last_mut()
        {
            *existing_polygon = multi_polygon_union(existing_polygon, &poly);
        } else {
            let multi_poly = MultiPolygon::<f32>::new(vec![poly]);
            self.queued_operations
                .push(PolygonMeshSetOperation::Add(multi_poly));
        }
    }

    /// Subtract a shape to the triangulation history.
    /// The current island is marked as out of date.
    pub fn queue_subtract(&mut self, shape: &[Vec2]) {
        let outer_line = LineString::from(shape.iter().map(|v| (v.x, v.y)).collect::<Vec<_>>());
        let poly = Polygon::<f32>::new(outer_line, vec![]);

        if let Some(PolygonMeshSetOperation::Subtract(ref mut existing_polygon)) =
            self.queued_operations.last_mut()
        {
            *existing_polygon = multi_polygon_union(existing_polygon, &poly);
        } else {
            let multi_poly = MultiPolygon::<f32>::new(vec![poly]);
            self.queued_operations
                .push(PolygonMeshSetOperation::Subtract(multi_poly));
        }
    }

    /// The mesh is up to date if all queued mesh operations have been applied and no further mesh operations have been queued yet.
    pub fn is_mesh_up_to_date(&self) -> bool {
        self.queued_operations.is_empty()
    }

    /// TODO: will this be supported?
    pub fn set_unit_radius(&mut self, radius: f32) {
        self.unit_radius = radius;
    }

    /// Apply all set queued set operations in the order they were previously queued.
    ///
    /// This must be called before converting the triangulation into a [`Mesh`] if there are overlapping obstacles,
    /// otherwise it will fail.
    ///
    /// # See Also
    /// - [`queue_add`](Triangulation::queue_add)
    /// - [`queue_subtract`](Triangulation::queue_subtract)
    pub fn update_mesh(&mut self) {
        for mesh_op in self.queued_operations.drain(..) {
            match mesh_op {
                PolygonMeshSetOperation::Add(shape) => {
                    #[cfg(all(feature = "wasm-incompatible", not(feature = "wasm-compatible")))]
                    {
                        self.multi_poly_mesh = self
                            .multi_poly_mesh
                            .union(&shape, GEO_CLIPPER_CLIP_PRECISION);
                    }
                    #[cfg(feature = "wasm-compatible")]
                    {
                        self.multi_poly_mesh = self.multi_poly_mesh.union(&shape);
                    }
                }
                PolygonMeshSetOperation::Subtract(shape) => {
                    #[cfg(all(feature = "wasm-incompatible", not(feature = "wasm-compatible")))]
                    {
                        self.multi_poly_mesh = self
                            .multi_poly_mesh
                            .difference(&shape, GEO_CLIPPER_CLIP_PRECISION);
                    }
                    #[cfg(feature = "wasm-compatible")]
                    {
                        self.multi_poly_mesh = self.multi_poly_mesh.difference(&shape);
                    }
                }
            }
        }
    }
}
impl Triangulation {
    /// Simplify the outer edge and obstacles, using a topology-preserving variant of the
    /// [Visvalingam-Whyatt algorithm](https://www.tandfonline.com/doi/abs/10.1179/000870493786962263).
    ///
    /// Epsilon is the minimum area a point should contribute to a polygon.
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    pub fn simplify(&mut self, epsilon: f32) {
        self.multi_poly_mesh = self.multi_poly_mesh.simplify_vw_preserve(&epsilon);
    }

    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    #[inline]
    fn add_constraint_edges(
        cdt: &mut ConstrainedDelaunayTriangulation<Point2<f32>>,
        edges: &LineString<f32>,
    ) -> Option<()> {
        let mut edge_iter = edges.coords().peekable();
        loop {
            let from = edge_iter.next().unwrap();
            let next = edge_iter.peek();
            // println!("{:.1}, {:.1}", from.x, from.y);

            let point_a = cdt
                .insert(Point2 {
                    x: from.x,
                    y: from.y,
                })
                .unwrap();
            let point_b = if let Some(next) = next {
                cdt.insert(Point2 {
                    x: next.x,
                    y: next.y,
                })
                .unwrap()
            } else {
                // println!("{:.1}, {:.1}", edges[0].x, edges[0].y);
                cdt.insert(Point2 {
                    x: edges[0].x,
                    y: edges[0].y,
                })
                .unwrap()
            };
            if cdt.can_add_constraint(point_a, point_b) {
                cdt.add_constraint(point_a, point_b);
            } else {
                // println!("{:.1}, {:.1}", next.unwrap().x, next.unwrap().y);
                return None;
            }
            if next.is_none() {
                break;
            }
        }
        Some(())
    }

    /// Convert the triangulation into a [`Mesh`].
    ///
    /// Meshes generated are not [baked](Mesh::bake), as they are made of triangles and it is recommended to
    /// call [`Mesh::update_mesh`] on them before baking.
    ///
    /// # Common Pitfall
    /// If you want an up to date representation of the mesh, you have to call [`update_mesh()`](Triangulation::update_mesh) before calling this method.
    ///
    /// # Example
    /// ```
    /// # use glam::vec2;
    /// # use polyanya::Triangulation;
    /// let mut triangulation = Triangulation::from_outer_edges(&[vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(0.0, 1.0)]);
    /// triangulation.queue_subtract(&vec![
    ///     vec2(0.3, 0.3),
    ///     vec2(0.7, 0.0),
    ///     vec2(0.3, 0.7),
    /// ]);
    ///
    /// // Update the triangulation, so all queued changes are reflected within the current navmesh.
    /// triangulation.update_mesh();
    ///
    /// let mut mesh = triangulation.as_navmesh().unwrap();
    ///
    /// // One call to merge should have reduced the number of polygons, baking will be less expensive.
    /// mesh.bake();
    /// ```
    #[cfg_attr(feature = "tracing", instrument(skip_all))]
    pub fn as_navmesh(&self) -> Option<Mesh> {
        if self.unit_radius != 0.0 {
            let with_radius_offset = self
                .multi_poly_mesh
                .offset_with_arc_segments(-self.unit_radius, 5)
                .unwrap();
            let poly = &with_radius_offset.simplify_vw_preserve(&(self.unit_radius / 100.0));
            let cdts = to_cdt(poly)?;
            cdt_to_navmesh(cdts, poly)
        } else {
            let cdt = to_cdt(&self.multi_poly_mesh)?;
            cdt_to_navmesh(cdt, &self.multi_poly_mesh)
        }
    }
}

/// The call of MultiPolygon.exterior_coords_iter() of the MultiPolygon is not sufficient to gather separate line strings,
/// because a naive approach (concatenating all of them) would just return one single mixed up LineString.
/// We have to do some work to separate them into their actual strings.
///
/// # Panic
/// This function acts upon the precondition that MultiPolygon.exterior_coords_iter() returns the outer bound points
/// in the correct order.
/// This seems to be the case at the time of writing.
///
/// If this precondition is not given at a later point in time, **this function will panic**!
/// You can fix that at the marked position down below.
fn get_bounds_as_line_strings(multi_poly: &MultiPolygon<f32>) -> Vec<LineString<f32>> {
    if multi_poly.0.is_empty() {
        return Vec::new();
    }

    let mut coord_without_guess = Vec::<Coord<f32>>::new();
    let mut possible_coord_with_guess = Vec::<(Coord<f32>, Coord<f32>)>::new();
    let mut bounds = Vec::<LineString<f32>>::new();
    let exterior_coords: Vec<_> = multi_poly.exterior_coords_iter().collect();
    let mut last_start_coord: Option<Coord<f32>> = None;

    let mut edge_iter = exterior_coords.iter().peekable();
    'polyon_edge_check: while let (Some(from), Some(to)) = (edge_iter.next(), edge_iter.peek()) {
        if let None = last_start_coord {
            last_start_coord = Some(*from);
        }

        // only allow to touch the edge of a polygon once (otherwise it's not an edge of the bound of the MultiPolygon)
        let mut touched_edge_of_polygon = 0;

        for poly in &multi_poly.0 {
            let mut exterior_iter = poly.exterior().0.iter().peekable();
            while let (Some(from_i), Some(to_i)) = (exterior_iter.next(), exterior_iter.peek()) {
                if *from == *from_i && **to == **to_i {
                    touched_edge_of_polygon += 1;
                }
                if touched_edge_of_polygon > 1 {
                    // TODO: see below: if this code path is taken, the function will panic
                    possible_coord_with_guess.push((*from, last_start_coord.unwrap()));
                    last_start_coord = Some(**to);
                    coord_without_guess.push(**to);
                    continue 'polyon_edge_check;
                }
            }
            let interiors = poly.interiors();
            for interior in interiors {
                let mut interior_iter = interior.0.iter().peekable();
                while let (Some(from_i), Some(to_i)) = (interior_iter.next(), interior_iter.peek())
                {
                    if *from == *from_i && **to == **to_i {
                        touched_edge_of_polygon += 1;
                    }
                    if touched_edge_of_polygon > 1 {
                        // TODO: see below: if this code path is taken, the function will panic
                        possible_coord_with_guess.push((*from, last_start_coord.unwrap()));
                        last_start_coord = Some(**to);
                        coord_without_guess.push(**to);
                        continue 'polyon_edge_check;
                    }
                }
            }
        }

        if touched_edge_of_polygon == 1 {
            match bounds.last_mut() {
                None => {
                    bounds.push(LineString::new(vec![*from, **to]));
                }
                Some(last_line_string) => {
                    if last_line_string.0.is_empty() {
                        last_line_string.0.push(*from);
                        last_line_string.0.push(**to);
                    } else {
                        last_line_string.0.push(**to);
                    }
                }
            }
        } else if touched_edge_of_polygon == 0 {
            // println!("Creating new line string. Starting point: {:?}, {:?}", *from, **to);
            bounds.push(LineString::new(vec![]));
        } else {
            unreachable!("There should have been an 'continue' before this statement is reached.");
        }
    }

    // TODO: insert special end - start edge check here
    assert!(coord_without_guess.is_empty());
    assert!(possible_coord_with_guess.is_empty());

    // finalize all line strings
    for bound in &mut bounds {
        bound.close();
    }

    bounds
}

fn to_cdt(multi_poly: &MultiPolygon<f32>) -> Option<ConstrainedDelaunayTriangulation<Point2<f32>>> {
    let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f32>>::new();
    for line_string in get_bounds_as_line_strings(multi_poly) {
        Triangulation::add_constraint_edges(&mut cdt, &line_string).unwrap();
    }

    for poly in &multi_poly.0 {
        if poly
            .interiors()
            .iter()
            .any(|obstacle| Triangulation::add_constraint_edges(&mut cdt, obstacle).is_none())
        {
            return None;
        }
    }
    Some(cdt)
}

fn cdt_to_navmesh(
    cdt: ConstrainedDelaunayTriangulation<Point2<f32>>,
    poly: &MultiPolygon<f32>,
) -> Option<Mesh> {
    #[cfg(feature = "tracing")]
    let polygon_span = tracing::info_span!("listing polygons").entered();

    let mut face_to_polygon: Vec<isize> = vec![-1; cdt.all_faces().len()];
    let mut i = 0;
    let polygons = cdt
        .inner_faces()
        .filter_map(|face| {
            #[cfg(feature = "tracing")]
            let _checking_span = tracing::info_span!("checking polygon").entered();

            let center = face.center();
            let center = Coord::from((center.x, center.y));
            poly.contains(&center).then(|| {
                #[cfg(feature = "tracing")]
                let _preparing_span = tracing::info_span!("preparing polygon").entered();

                face_to_polygon[face.index()] = i;
                i += 1;
                crate::Polygon::new(
                    face.vertices()
                        .iter()
                        .map(|vertex| vertex.index() as u32)
                        .collect(),
                    // TODO: can this be set to the correct value?
                    false,
                )
            })
        })
        .collect::<Vec<_>>();

    #[cfg(feature = "tracing")]
    drop(polygon_span);

    #[cfg(feature = "tracing")]
    let vertex_span = tracing::info_span!("listing vertices").entered();

    let vertices = cdt
        .vertices()
        .map(|point| {
            #[cfg(feature = "tracing")]
            let _preparing_span = tracing::info_span!("preparing vertex").entered();

            let mut neighbour_polygons = point
                .out_edges()
                .map(|out_edge| face_to_polygon[out_edge.face().index()])
                .collect::<VecDeque<_>>();
            let neighbour_polygons: Vec<_> = if neighbour_polygons.iter().all(|i| *i == -1) {
                vec![-1]
            } else {
                while neighbour_polygons[0] == -1 {
                    neighbour_polygons.rotate_left(1);
                }
                let mut neighbour_polygons: Vec<_> = neighbour_polygons.into();
                neighbour_polygons.dedup();
                neighbour_polygons
            };
            let point = point.position();
            crate::Vertex::new(vec2(point.x, point.y), neighbour_polygons)
        })
        .collect::<Vec<_>>();

    #[cfg(feature = "tracing")]
    drop(vertex_span);

    Some(Mesh {
        vertices,
        polygons,
        ..Default::default()
    })
}
