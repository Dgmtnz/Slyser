use crate::cut::{unproject_from_plane_2d, CutResult};
use crate::mesh::{Mesh, Triangle};
use crate::primitives::create_box;
use glam::{Vec2, Vec3};

/// Configuration parameters for female sockets and square alignment pins.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinConfig {
    /// Width and height of square pin in mm (default: 4.0 mm).
    pub size: f32,
    /// Insertion depth into each piece in mm (default: 5.0 mm).
    pub depth: f32,
    /// XY clearance per side for FDM fit in mm (default: 0.2 mm, socket hole is size + 2 * clearance).
    pub clearance: f32,
    /// Minimum distance from socket boundary to cut section edge in mm (default: 2.0 mm).
    pub edge_margin: f32,
}

impl Default for PinConfig {
    fn default() -> Self {
        Self {
            size: 4.0,
            depth: 5.0,
            clearance: 0.2,
            edge_margin: 2.0,
        }
    }
}

/// Represents the planned placement of an alignment pin on the cut plane.
#[derive(Debug, Clone, PartialEq)]
pub struct PinPlacement {
    /// 2D center position in cut plane local coordinates (u, v).
    pub center_2d: Vec2,
    /// 3D center position in world space.
    pub center_3d: Vec3,
    /// Orientation angle in radians in plane coordinates (default: 0.0).
    pub angle: f32,
}

/// Tests whether point `p` is strictly inside or on the boundary of CCW 2D triangle `(a, b, c)`.
#[inline]
fn point_in_triangle_2d(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let cross1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    let cross2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    let cross3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

    let eps = -1e-6;
    cross1 >= eps && cross2 >= eps && cross3 >= eps
}

/// Point-in-polygon test using the standard even-odd ray casting algorithm.
pub fn point_in_polygon_2d(pt: Vec2, poly: &[Vec2]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    for i in 0..n {
        let j = (i + 1) % n;
        let pi = poly[i];
        let pj = poly[j];
        if ((pi.y > pt.y) != (pj.y > pt.y))
            && (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }
    }
    inside
}

/// Returns true if open segment (a, b) intersects segment (c, d).
fn segments_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let eps = 1e-4;

    // Disregard if endpoints match
    if (a - c).length_squared() < eps * eps
        || (a - d).length_squared() < eps * eps
        || (b - c).length_squared() < eps * eps
        || (b - d).length_squared() < eps * eps
    {
        return false;
    }

    #[inline]
    fn ccw(p1: Vec2, p2: Vec2, p3: Vec2) -> f32 {
        (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)
    }

    let cp1 = ccw(a, b, c);
    let cp2 = ccw(a, b, d);
    let cp3 = ccw(c, d, a);
    let cp4 = ccw(c, d, b);

    if ((cp1 > eps && cp2 < -eps) || (cp1 < -eps && cp2 > eps))
        && ((cp3 > eps && cp4 < -eps) || (cp3 < -eps && cp4 > eps))
    {
        return true;
    }

    false
}

/// Checks if an axis-aligned square of size `side` centered at `center` fits strictly inside `poly`.
fn is_square_inside_polygon(center: Vec2, side: f32, poly: &[Vec2]) -> bool {
    let half = side * 0.5;
    let corners = [
        center + Vec2::new(-half, -half),
        center + Vec2::new(half, -half),
        center + Vec2::new(half, half),
        center + Vec2::new(-half, half),
    ];

    if !point_in_polygon_2d(center, poly) {
        return false;
    }

    for &c in &corners {
        if !point_in_polygon_2d(c, poly) {
            return false;
        }
    }

    let n = poly.len();
    for i in 0..n {
        let p0 = poly[i];
        let p1 = poly[(i + 1) % n];

        // Check if any polygon vertex is inside square
        if p0.x >= center.x - half
            && p0.x <= center.x + half
            && p0.y >= center.y - half
            && p0.y <= center.y + half
        {
            return false;
        }

        // Check edge intersections
        for j in 0..4 {
            let c0 = corners[j];
            let c1 = corners[(j + 1) % 4];
            if segments_intersect(p0, p1, c0, c1) {
                return false;
            }
        }
    }

    true
}

/// Computes the signed area of a 2D polygon loop.
fn polygon_area_2d(poly: &[Vec2]) -> f32 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    area * 0.5
}

/// Computes the centroid of a 2D polygon loop.
fn polygon_centroid_2d(poly: &[Vec2]) -> Vec2 {
    let n = poly.len();
    if n < 3 {
        return Vec2::ZERO;
    }
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut signed_area = 0.0;

    for i in 0..n {
        let j = (i + 1) % n;
        let factor = poly[i].x * poly[j].y - poly[j].x * poly[i].y;
        cx += (poly[i].x + poly[j].x) * factor;
        cy += (poly[i].y + poly[j].y) * factor;
        signed_area += factor;
    }

    signed_area *= 0.5;
    if signed_area.abs() < 1e-6 {
        return Vec2::ZERO;
    }

    Vec2::new(cx / (6.0 * signed_area), cy / (6.0 * signed_area))
}

/// Finds safe locations inside the largest 2D cut polygon where a square socket fits completely.
pub fn find_pin_placements(cut_result: &CutResult, config: &PinConfig) -> Vec<PinPlacement> {
    if cut_result.cut_polygons_2d.is_empty() {
        return Vec::new();
    }

    // Find the largest 2D polygon by area
    let mut max_idx = 0;
    let mut max_area = -1.0;
    for (i, poly) in cut_result.cut_polygons_2d.iter().enumerate() {
        let area = polygon_area_2d(poly).abs();
        if area > max_area {
            max_area = area;
            max_idx = i;
        }
    }

    let poly = &cut_result.cut_polygons_2d[max_idx];
    if poly.len() < 3 {
        return Vec::new();
    }

    let socket_width = config.size + 2.0 * config.clearance;
    let safe_width = socket_width + 2.0 * config.edge_margin;

    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for p in poly {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }

    let span_x = max_x - min_x;
    let span_y = max_y - min_y;

    if span_x < safe_width || span_y < safe_width {
        return Vec::new();
    }

    let centroid = polygon_centroid_2d(poly);
    let mut candidate_centers = Vec::new();

    // Check if section is long enough for multiple pins
    let is_long_x = span_x >= 2.5 * safe_width && span_x >= 1.8 * span_y;
    let is_long_y = span_y >= 2.5 * safe_width && span_y >= 1.8 * span_x;
    let is_large_both = span_x >= 50.0 && span_y >= 50.0 && max_area >= 2500.0;

    if is_large_both {
        let dx = span_x * 0.25;
        let dy = span_y * 0.25;
        candidate_centers.push(centroid + Vec2::new(-dx, -dy));
        candidate_centers.push(centroid + Vec2::new(dx, -dy));
        candidate_centers.push(centroid + Vec2::new(dx, dy));
        candidate_centers.push(centroid + Vec2::new(-dx, dy));
    } else if is_long_x {
        let dx = span_x * 0.25;
        candidate_centers.push(centroid + Vec2::new(-dx, 0.0));
        candidate_centers.push(centroid + Vec2::new(dx, 0.0));
    } else if is_long_y {
        let dy = span_y * 0.25;
        candidate_centers.push(centroid + Vec2::new(0.0, -dy));
        candidate_centers.push(centroid + Vec2::new(0.0, dy));
    }

    // Validate candidates
    let mut valid_placements = Vec::new();
    for c in candidate_centers {
        if is_square_inside_polygon(c, safe_width, poly) {
            let p3d = unproject_from_plane_2d(
                c,
                cut_result.cut_plane.point,
                cut_result.plane_basis.0,
                cut_result.plane_basis.1,
            );
            valid_placements.push(PinPlacement {
                center_2d: c,
                center_3d: p3d,
                angle: 0.0,
            });
        }
    }

    if valid_placements.is_empty() && is_square_inside_polygon(centroid, safe_width, poly) {
        let p3d = unproject_from_plane_2d(
            centroid,
            cut_result.cut_plane.point,
            cut_result.plane_basis.0,
            cut_result.plane_basis.1,
        );
        valid_placements.push(PinPlacement {
            center_2d: centroid,
            center_3d: p3d,
            angle: 0.0,
        });
    }

    valid_placements
}

/// Generates a standalone solid square alignment pin mesh.
/// Dimensions: size x size x (2.0 * depth - 0.4 mm).
/// Placed flat on Z=0 for immediate 3D printing.
pub fn generate_square_pin(config: &PinConfig) -> Mesh {
    let pin_length = (2.0 * config.depth - 0.4).max(0.1);
    let half_size = config.size * 0.5;
    create_box(
        Vec3::new(-half_size, -half_size, 0.0),
        Vec3::new(half_size, half_size, pin_length),
    )
}

/// Generates the 4 corners of a square hole in 2D plane coordinates in clockwise (CW) order.
fn square_hole_corners(center: Vec2, side: f32, angle: f32) -> [Vec2; 4] {
    let half = side * 0.5;
    let cos_a = angle.cos();
    let sin_a = angle.sin();

    let rot = |x: f32, y: f32| Vec2::new(x * cos_a - y * sin_a, x * sin_a + y * cos_a);

    // CW order in 2D: top-left -> top-right -> bottom-right -> bottom-left
    [
        center + rot(-half, half),
        center + rot(half, half),
        center + rot(half, -half),
        center + rot(-half, -half),
    ]
}

/// Triangulates a simple 2D CCW polygon loop into a list of triangles.
fn triangulate_ear_clipping(poly: &[Vec2]) -> Vec<[Vec2; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![[poly[0], poly[1], poly[2]]];
    }

    let mut indices: Vec<usize> = (0..n).collect();
    let mut triangles = Vec::with_capacity(n - 2);

    let mut iterations = 0;
    let max_iterations = n * n * 2;

    while indices.len() > 3 && iterations < max_iterations {
        let m = indices.len();
        let mut ear_found = false;

        for k in 0..m {
            iterations += 1;
            let prev_idx = indices[(k + m - 1) % m];
            let curr_idx = indices[k];
            let next_idx = indices[(k + 1) % m];

            let a = poly[prev_idx];
            let b = poly[curr_idx];
            let c = poly[next_idx];

            let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
            if cross <= 1e-6 {
                continue;
            }

            let mut has_point_inside = false;
            for &other_idx in &indices {
                if other_idx == prev_idx || other_idx == curr_idx || other_idx == next_idx {
                    continue;
                }
                let pt = poly[other_idx];
                // Ignore coincident boundary vertices from bridges
                if (pt - a).length_squared() < 1e-6
                    || (pt - b).length_squared() < 1e-6
                    || (pt - c).length_squared() < 1e-6
                {
                    continue;
                }
                if point_in_triangle_2d(pt, a, b, c) {
                    has_point_inside = true;
                    break;
                }
            }

            if !has_point_inside {
                triangles.push([a, b, c]);
                indices.remove(k);
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            let mut best_k = 0;
            let mut max_cross = f32::NEG_INFINITY;
            for k in 0..m {
                let prev_idx = indices[(k + m - 1) % m];
                let curr_idx = indices[k];
                let next_idx = indices[(k + 1) % m];
                let a = poly[prev_idx];
                let b = poly[curr_idx];
                let c = poly[next_idx];
                let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
                if cross > max_cross {
                    max_cross = cross;
                    best_k = k;
                }
            }
            let prev_idx = indices[(best_k + m - 1) % m];
            let curr_idx = indices[best_k];
            let next_idx = indices[(best_k + 1) % m];
            triangles.push([poly[prev_idx], poly[curr_idx], poly[next_idx]]);
            indices.remove(best_k);
        }
    }

    if indices.len() == 3 {
        triangles.push([poly[indices[0]], poly[indices[1]], poly[indices[2]]]);
    }

    triangles
}

/// Connects CW square holes to a CCW outer polygon using seam bridges and triangulates the result.
fn triangulate_polygon_with_holes(outer: &[Vec2], holes: &[[Vec2; 4]]) -> Vec<[Vec2; 3]> {
    if holes.is_empty() {
        return triangulate_ear_clipping(outer);
    }

    let mut current_loop = outer.to_vec();

    for hole in holes {
        // Find best bridge between a vertex in hole and a vertex in current_loop
        let mut best_dist_sq = f32::INFINITY;
        let mut best_outer_idx = 0;
        let mut best_hole_idx = 0;

        for (hi, &hp) in hole.iter().enumerate() {
            for (oi, &op) in current_loop.iter().enumerate() {
                let d_sq = (hp - op).length_squared();
                if d_sq < best_dist_sq {
                    // Test segment visibility
                    let mut blocked = false;

                    // Midpoint must be inside outer and outside hole
                    let mid = (hp + op) * 0.5;
                    if !point_in_polygon_2d(mid, outer) {
                        blocked = true;
                    }

                    if !blocked {
                        let cl_len = current_loop.len();
                        for e in 0..cl_len {
                            let p0 = current_loop[e];
                            let p1 = current_loop[(e + 1) % cl_len];
                            if segments_intersect(op, hp, p0, p1) {
                                blocked = true;
                                break;
                            }
                        }
                    }

                    if !blocked {
                        for e in 0..4 {
                            let p0 = hole[e];
                            let p1 = hole[(e + 1) % 4];
                            if segments_intersect(op, hp, p0, p1) {
                                blocked = true;
                                break;
                            }
                        }
                    }

                    if !blocked {
                        best_dist_sq = d_sq;
                        best_outer_idx = oi;
                        best_hole_idx = hi;
                    }
                }
            }
        }

        // Bridge insertion:
        // current_loop[0..=best_outer_idx] +
        // hole[best_hole_idx..] + hole[..=best_hole_idx] +
        // current_loop[best_outer_idx..]
        let mut new_loop = Vec::with_capacity(current_loop.len() + 6);
        new_loop.extend_from_slice(&current_loop[0..=best_outer_idx]);

        for i in 0..4 {
            let idx = (best_hole_idx + i) % 4;
            new_loop.push(hole[idx]);
        }
        new_loop.push(hole[best_hole_idx]);

        new_loop.extend_from_slice(&current_loop[best_outer_idx..]);
        current_loop = new_loop;
    }

    triangulate_ear_clipping(&current_loop)
}

/// Applies female socket cavities into both `positive_mesh` and `negative_mesh` on the cut plane,
/// and generates independent square alignment pin meshes.
pub fn apply_female_sockets_and_generate_pins(
    cut_result: &CutResult,
    config: &PinConfig,
) -> (Mesh, Mesh, Vec<Mesh>) {
    let placements = find_pin_placements(cut_result, config);
    if placements.is_empty() {
        return (
            cut_result.positive_mesh.clone(),
            cut_result.negative_mesh.clone(),
            Vec::new(),
        );
    }

    let socket_width = config.size + 2.0 * config.clearance;
    let depth = config.depth;
    let norm = cut_result.cut_plane.normal.normalize();
    let plane_pt = cut_result.cut_plane.point;
    let (u_axis, v_axis) = cut_result.plane_basis;

    // Filter out original cap triangles from positive_mesh and negative_mesh
    let is_pos_cap = |tri: &Triangle| {
        tri.normal.dot(-norm) > 0.9
            && (tri.vertices[0] - plane_pt).dot(norm).abs() < 1e-4
            && (tri.vertices[1] - plane_pt).dot(norm).abs() < 1e-4
            && (tri.vertices[2] - plane_pt).dot(norm).abs() < 1e-4
    };

    let is_neg_cap = |tri: &Triangle| {
        tri.normal.dot(norm) > 0.9
            && (tri.vertices[0] - plane_pt).dot(norm).abs() < 1e-4
            && (tri.vertices[1] - plane_pt).dot(norm).abs() < 1e-4
            && (tri.vertices[2] - plane_pt).dot(norm).abs() < 1e-4
    };

    let mut new_pos_tris: Vec<Triangle> = cut_result
        .positive_mesh
        .triangles
        .iter()
        .filter(|t| !is_pos_cap(t))
        .copied()
        .collect();

    let mut new_neg_tris: Vec<Triangle> = cut_result
        .negative_mesh
        .triangles
        .iter()
        .filter(|t| !is_neg_cap(t))
        .copied()
        .collect();

    // Identify largest polygon index
    let mut max_idx = 0;
    let mut max_area = -1.0;
    for (i, poly) in cut_result.cut_polygons_2d.iter().enumerate() {
        let area = polygon_area_2d(poly).abs();
        if area > max_area {
            max_area = area;
            max_idx = i;
        }
    }

    let holes: Vec<[Vec2; 4]> = placements
        .iter()
        .map(|p| square_hole_corners(p.center_2d, socket_width, p.angle))
        .collect();

    // Reconstruct cap triangles
    for (i, poly) in cut_result.cut_loops_2d.iter().enumerate() {
        let tri_2d = if i == max_idx {
            triangulate_polygon_with_holes(poly, &holes)
        } else {
            triangulate_ear_clipping(poly)
        };

        for [p0_2d, p1_2d, p2_2d] in tri_2d {
            let p0 = unproject_from_plane_2d(p0_2d, plane_pt, u_axis, v_axis);
            let p1 = unproject_from_plane_2d(p1_2d, plane_pt, u_axis, v_axis);
            let p2 = unproject_from_plane_2d(p2_2d, plane_pt, u_axis, v_axis);

            // Cap for negative mesh: outward normal +norm
            new_neg_tris.push(Triangle::with_normal([p0, p1, p2], norm));

            // Cap for positive mesh: outward normal -norm
            new_pos_tris.push(Triangle::with_normal([p0, p2, p1], -norm));
        }
    }

    // Add cavity walls and bottom faces for each socket
    for hole in &holes {
        let m0 = unproject_from_plane_2d(hole[0], plane_pt, u_axis, v_axis);
        let m1 = unproject_from_plane_2d(hole[1], plane_pt, u_axis, v_axis);
        let m2 = unproject_from_plane_2d(hole[2], plane_pt, u_axis, v_axis);
        let m3 = unproject_from_plane_2d(hole[3], plane_pt, u_axis, v_axis);

        let m = [m0, m1, m2, m3];

        // --- Positive mesh cavity (extends into +norm direction) ---
        let bp0 = m0 + depth * norm;
        let bp1 = m1 + depth * norm;
        let bp2 = m2 + depth * norm;
        let bp3 = m3 + depth * norm;
        let bp = [bp0, bp1, bp2, bp3];

        // Bottom face: outward normal -norm
        new_pos_tris.push(Triangle::with_normal([bp0, bp1, bp2], -norm));
        new_pos_tris.push(Triangle::with_normal([bp0, bp2, bp3], -norm));

        // 4 Side walls for positive mesh
        for k in 0..4 {
            let next = (k + 1) % 4;
            new_pos_tris.push(Triangle::new(m[k], m[next], bp[next]));
            new_pos_tris.push(Triangle::new(m[k], bp[next], bp[k]));
        }

        // --- Negative mesh cavity (extends into -norm direction) ---
        let bn0 = m0 - depth * norm;
        let bn1 = m1 - depth * norm;
        let bn2 = m2 - depth * norm;
        let bn3 = m3 - depth * norm;
        let bn = [bn0, bn1, bn2, bn3];

        // Bottom face: outward normal +norm
        new_neg_tris.push(Triangle::with_normal([bn0, bn3, bn2], norm));
        new_neg_tris.push(Triangle::with_normal([bn0, bn2, bn1], norm));

        // 4 Side walls for negative mesh
        for k in 0..4 {
            let next = (k + 1) % 4;
            new_neg_tris.push(Triangle::new(m[next], m[k], bn[k]));
            new_neg_tris.push(Triangle::new(m[next], bn[k], bn[next]));
        }
    }

    let positive_mesh = Mesh::new(new_pos_tris);
    let negative_mesh = Mesh::new(new_neg_tris);

    let pin_meshes = placements
        .iter()
        .map(|_| generate_square_pin(config))
        .collect();

    (positive_mesh, negative_mesh, pin_meshes)
}
