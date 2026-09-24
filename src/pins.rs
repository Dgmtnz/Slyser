use crate::bvh::BVH;
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
    /// Whether adaptive pin sizing is enabled (default: false).
    pub adaptive: bool,
    /// Minimum allowed pin size when adaptive sizing is enabled in mm (default: 1.5 mm).
    pub min_size: f32,
}

impl Default for PinConfig {
    fn default() -> Self {
        Self {
            size: 4.0,
            depth: 5.0,
            clearance: 0.2,
            edge_margin: 2.0,
            adaptive: false,
            min_size: 1.5,
        }
    }
}

impl PinConfig {
    /// Creates a default pin configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets whether adaptive pin sizing is enabled.
    pub fn with_adaptive(mut self, adaptive: bool) -> Self {
        self.adaptive = adaptive;
        self
    }

    /// Sets the minimum pin size in mm for adaptive sizing.
    pub fn with_min_size(mut self, min_size: f32) -> Self {
        self.min_size = min_size;
        self
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
    /// Alignment pin size in mm.
    pub size: f32,
    /// Safe cavity insertion depth in mm into each piece.
    pub depth: f32,
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

/// Probes the distance from `origin` along `dir` to the outer surface of a mesh.
/// Returns 0.0 if the ray misses or hits non-positively.
fn probe_mesh_thickness(bvh: &BVH, origin: Vec3, dir: Vec3) -> f32 {
    if let Some(hit) = bvh.intersect_ray_with_options(origin, dir, 1e-3, 1e4, None) {
        return hit.t;
    }
    // Fallback: slight perturbation in case origin was exactly on a triangle edge or vertex
    let offsets = [
        Vec3::new(0.01, 0.0, 0.0),
        Vec3::new(-0.01, 0.0, 0.0),
        Vec3::new(0.0, 0.01, 0.0),
        Vec3::new(0.0, -0.01, 0.0),
    ];
    for off in offsets {
        if let Some(hit) = bvh.intersect_ray_with_options(origin + off, dir, 1e-3, 1e4, None) {
            return hit.t;
        }
    }
    0.0
}

/// Computes the safe socket cavity depth at candidate pin position `center_2d` with given `size` and `angle`.
/// Probes solid wall thickness into both positive_mesh (+norm) and negative_mesh (-norm)
/// at the socket center and the 4 corners.
/// Leaves a mandatory solid safety wall of at least 1.8 mm.
fn compute_safe_pin_depth(
    center_2d: Vec2,
    size: f32,
    angle: f32,
    cut_result: &CutResult,
    config: &PinConfig,
    bvh_pos: &BVH,
    bvh_neg: &BVH,
) -> f32 {
    let socket_width = size + 2.0 * config.clearance;
    let corners_2d = square_hole_corners(center_2d, socket_width, angle);
    let plane_pt = cut_result.cut_plane.point;
    let (u_axis, v_axis) = cut_result.plane_basis;
    let norm = cut_result.cut_plane.normal.normalize();

    let center_3d = unproject_from_plane_2d(center_2d, plane_pt, u_axis, v_axis);
    let mut probe_points = Vec::with_capacity(5);
    probe_points.push(center_3d);
    for c in &corners_2d {
        probe_points.push(unproject_from_plane_2d(*c, plane_pt, u_axis, v_axis));
    }

    let mut t_pos = f32::INFINITY;
    for &pt in &probe_points {
        let d = probe_mesh_thickness(bvh_pos, pt, norm);
        if d <= 0.0 {
            t_pos = 0.0;
            break;
        }
        t_pos = t_pos.min(d);
    }

    let mut t_neg = f32::INFINITY;
    for &pt in &probe_points {
        let d = probe_mesh_thickness(bvh_neg, pt, -norm);
        if d <= 0.0 {
            t_neg = 0.0;
            break;
        }
        t_neg = t_neg.min(d);
    }

    let safe_depth_pos = (t_pos - 1.8).max(0.0);
    let safe_depth_neg = (t_neg - 1.8).max(0.0);
    config.depth.min(safe_depth_pos).min(safe_depth_neg)
}

fn find_candidate_placements(
    cut_result: &CutResult,
    config: &PinConfig,
    candidate_size: f32,
    candidate_margin: f32,
    bvh_pos: &BVH,
    bvh_neg: &BVH,
) -> Vec<PinPlacement> {
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

    let socket_width = candidate_size + 2.0 * config.clearance;
    let safe_width = socket_width + 2.0 * candidate_margin;

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
            let safe_depth = compute_safe_pin_depth(
                c,
                candidate_size,
                0.0,
                cut_result,
                config,
                bvh_pos,
                bvh_neg,
            );
            if safe_depth >= 1.5 {
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
                    size: candidate_size,
                    depth: safe_depth,
                });
            }
        }
    }

    if valid_placements.is_empty() && is_square_inside_polygon(centroid, safe_width, poly) {
        let safe_depth = compute_safe_pin_depth(
            centroid,
            candidate_size,
            0.0,
            cut_result,
            config,
            bvh_pos,
            bvh_neg,
        );
        if safe_depth >= 1.5 {
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
                size: candidate_size,
                depth: safe_depth,
            });
        }
    }

    if valid_placements.is_empty() {
        let grid_steps = 11;
        for ix in 1..grid_steps {
            let tx = ix as f32 / grid_steps as f32;
            let gx = min_x + tx * span_x;
            for iy in 1..grid_steps {
                let ty = iy as f32 / grid_steps as f32;
                let gy = min_y + ty * span_y;
                let pt = Vec2::new(gx, gy);
                if is_square_inside_polygon(pt, safe_width, poly) {
                    let safe_depth = compute_safe_pin_depth(
                        pt,
                        candidate_size,
                        0.0,
                        cut_result,
                        config,
                        bvh_pos,
                        bvh_neg,
                    );
                    if safe_depth >= 1.5 {
                        let p3d = unproject_from_plane_2d(
                            pt,
                            cut_result.cut_plane.point,
                            cut_result.plane_basis.0,
                            cut_result.plane_basis.1,
                        );
                        valid_placements.push(PinPlacement {
                            center_2d: pt,
                            center_3d: p3d,
                            angle: 0.0,
                            size: candidate_size,
                            depth: safe_depth,
                        });
                        return valid_placements;
                    }
                }
            }
        }
    }

    valid_placements
}

/// Finds safe locations inside the largest 2D cut polygon where a square socket fits completely.
/// When adaptive pin sizing is enabled on config, adaptively finds smaller pin sizes and relaxed margins.
pub fn find_pin_placements(cut_result: &CutResult, config: &PinConfig) -> Vec<PinPlacement> {
    let pos_tris = if cut_result.positive_body.is_empty() {
        &cut_result.positive_mesh.triangles
    } else {
        &cut_result.positive_body
    };
    let neg_tris = if cut_result.negative_body.is_empty() {
        &cut_result.negative_mesh.triangles
    } else {
        &cut_result.negative_body
    };
    let bvh_pos = BVH::build(pos_tris);
    let bvh_neg = BVH::build(neg_tris);

    let default_placements = find_candidate_placements(
        cut_result,
        config,
        config.size,
        config.edge_margin,
        &bvh_pos,
        &bvh_neg,
    );
    if !default_placements.is_empty() || !config.adaptive {
        return default_placements;
    }

    let candidate_sizes = [3.5f32, 3.0, 2.5, 2.0, 1.5];
    let min_sz = config.min_size.max(1.0);

    // Pass 1: Try smaller sizes with default edge_margin
    for &sz in &candidate_sizes {
        if sz < config.size && sz >= min_sz {
            let placements = find_candidate_placements(
                cut_result,
                config,
                sz,
                config.edge_margin,
                &bvh_pos,
                &bvh_neg,
            );
            if !placements.is_empty() {
                return placements;
            }
        }
    }

    // Pass 2: Relax edge_margin down to 1.0 mm (first 1.5 mm, then 1.0 mm)
    for &relaxed_margin in &[1.5f32, 1.0] {
        if relaxed_margin < config.edge_margin {
            let placements = find_candidate_placements(
                cut_result,
                config,
                config.size,
                relaxed_margin,
                &bvh_pos,
                &bvh_neg,
            );
            if !placements.is_empty() {
                return placements;
            }
            for &sz in &candidate_sizes {
                if sz < config.size && sz >= min_sz {
                    let placements = find_candidate_placements(
                        cut_result,
                        config,
                        sz,
                        relaxed_margin,
                        &bvh_pos,
                        &bvh_neg,
                    );
                    if !placements.is_empty() {
                        return placements;
                    }
                }
            }
        }
    }

    Vec::new()
}

/// Helper to search pin placements with adaptive sizing enabled and a specific minimum size.
pub fn find_adaptive_pin_placements(
    cut_result: &CutResult,
    config: &PinConfig,
    min_size: f32,
) -> Vec<PinPlacement> {
    let mut cfg = *config;
    cfg.adaptive = true;
    cfg.min_size = min_size;
    find_pin_placements(cut_result, &cfg)
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

/// Triangulates a simple 2D CCW polygon loop into a list of triangle index triplets.
fn triangulate_ear_clipping_indexed(current_loop: &[usize], verts_2d: &[Vec2]) -> Vec<[usize; 3]> {
    let n = current_loop.len();
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![[current_loop[0], current_loop[1], current_loop[2]]];
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
            let prev_node = indices[(k + m - 1) % m];
            let curr_node = indices[k];
            let next_node = indices[(k + 1) % m];

            let prev_idx = current_loop[prev_node];
            let curr_idx = current_loop[curr_node];
            let next_idx = current_loop[next_node];

            let a = verts_2d[prev_idx];
            let b = verts_2d[curr_idx];
            let c = verts_2d[next_idx];

            let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
            if cross <= 1e-6 {
                continue;
            }

            let mut has_point_inside = false;
            for &other_node in &indices {
                if other_node == prev_node || other_node == curr_node || other_node == next_node {
                    continue;
                }
                let pt = verts_2d[current_loop[other_node]];
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
                triangles.push([prev_idx, curr_idx, next_idx]);
                indices.remove(k);
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            let mut best_k = 0;
            let mut max_cross = f32::NEG_INFINITY;
            for k in 0..m {
                let prev_node = indices[(k + m - 1) % m];
                let curr_node = indices[k];
                let next_node = indices[(k + 1) % m];
                let a = verts_2d[current_loop[prev_node]];
                let b = verts_2d[current_loop[curr_node]];
                let c = verts_2d[current_loop[next_node]];
                let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
                if cross > max_cross {
                    max_cross = cross;
                    best_k = k;
                }
            }
            let prev_node = indices[(best_k + m - 1) % m];
            let curr_node = indices[best_k];
            let next_node = indices[(best_k + 1) % m];
            triangles.push([
                current_loop[prev_node],
                current_loop[curr_node],
                current_loop[next_node],
            ]);
            indices.remove(best_k);
        }
    }

    if indices.len() == 3 {
        triangles.push([
            current_loop[indices[0]],
            current_loop[indices[1]],
            current_loop[indices[2]],
        ]);
    }

    triangles
}

/// Connects CW square holes to a CCW outer polygon using seam bridges and triangulates the result.
fn triangulate_polygon_with_holes_indexed(
    outer_len: usize,
    num_holes: usize,
    verts_2d: &[Vec2],
) -> Vec<[usize; 3]> {
    if num_holes == 0 {
        let current_loop: Vec<usize> = (0..outer_len).collect();
        return triangulate_ear_clipping_indexed(&current_loop, verts_2d);
    }

    let mut current_loop: Vec<usize> = (0..outer_len).collect();
    let outer_poly = &verts_2d[0..outer_len];

    for h in 0..num_holes {
        let hole_start = outer_len + 4 * h;
        let hole_indices = [
            hole_start,
            hole_start + 1,
            hole_start + 2,
            hole_start + 3,
        ];
        let hole_pts = [
            verts_2d[hole_indices[0]],
            verts_2d[hole_indices[1]],
            verts_2d[hole_indices[2]],
            verts_2d[hole_indices[3]],
        ];

        // Find best bridge between a vertex in hole and a vertex in current_loop
        let mut best_dist_sq = f32::INFINITY;
        let mut best_outer_node = 0;
        let mut best_hole_idx = 0;

        for (hi, &h_idx) in hole_indices.iter().enumerate() {
            let hp = verts_2d[h_idx];
            for (oi, &o_idx) in current_loop.iter().enumerate() {
                let op = verts_2d[o_idx];
                let d_sq = (hp - op).length_squared();
                if d_sq < best_dist_sq {
                    // Test segment visibility
                    let mut blocked = false;

                    // Midpoint must be inside outer and outside hole
                    let mid = (hp + op) * 0.5;
                    if !point_in_polygon_2d(mid, outer_poly) {
                        blocked = true;
                    }

                    if !blocked {
                        let cl_len = current_loop.len();
                        for e in 0..cl_len {
                            let p0 = verts_2d[current_loop[e]];
                            let p1 = verts_2d[current_loop[(e + 1) % cl_len]];
                            if segments_intersect(op, hp, p0, p1) {
                                blocked = true;
                                break;
                            }
                        }
                    }

                    if !blocked {
                        for e in 0..4 {
                            let p0 = hole_pts[e];
                            let p1 = hole_pts[(e + 1) % 4];
                            if segments_intersect(op, hp, p0, p1) {
                                blocked = true;
                                break;
                            }
                        }
                    }

                    if !blocked {
                        best_dist_sq = d_sq;
                        best_outer_node = oi;
                        best_hole_idx = hi;
                    }
                }
            }
        }

        // Bridge insertion:
        // current_loop[0..=best_outer_node] +
        // hole[best_hole_idx..] + hole[..=best_hole_idx] +
        // current_loop[best_outer_node..]
        let mut new_loop = Vec::with_capacity(current_loop.len() + 6);
        new_loop.extend_from_slice(&current_loop[0..=best_outer_node]);

        for i in 0..4 {
            let idx = (best_hole_idx + i) % 4;
            new_loop.push(hole_indices[idx]);
        }
        new_loop.push(hole_indices[best_hole_idx]);

        new_loop.extend_from_slice(&current_loop[best_outer_node..]);
        current_loop = new_loop;
    }

    triangulate_ear_clipping_indexed(&current_loop, verts_2d)
}

/// Applies female socket cavities into both `positive_mesh` and `negative_mesh` on the cut plane,
/// and generates independent square alignment pin meshes.
fn apply_sockets_for_placements(
    cut_result: &CutResult,
    config: &PinConfig,
    placements: &[PinPlacement],
) -> (Mesh, Mesh, Vec<Mesh>) {
    if placements.is_empty() {
        return (
            cut_result.positive_mesh.clone(),
            cut_result.negative_mesh.clone(),
            Vec::new(),
        );
    }

    let norm = cut_result.cut_plane.normal.normalize();
    let plane_pt = cut_result.cut_plane.point;
    let (u_axis, v_axis) = cut_result.plane_basis;

    let mut new_pos_tris: Vec<Triangle> = cut_result.positive_body.clone();
    let mut new_neg_tris: Vec<Triangle> = cut_result.negative_body.clone();

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
        .map(|p| square_hole_corners(p.center_2d, p.size + 2.0 * config.clearance, p.angle))
        .collect();

    // Reconstruct cap triangles
    for (i, loop_2d) in cut_result.cut_loops_2d.iter().enumerate() {
        let loop_3d = &cut_result.cut_loops_3d[i];
        let n = loop_3d.len();
        if n < 3 {
            continue;
        }

        let mut verts_2d = loop_2d.clone();
        let mut verts_3d = loop_3d.clone();

        let num_holes = if i == max_idx {
            for hole in &holes {
                for &c in hole {
                    verts_2d.push(c);
                    verts_3d.push(unproject_from_plane_2d(c, plane_pt, u_axis, v_axis));
                }
            }
            holes.len()
        } else {
            0
        };

        let tri_indices = triangulate_polygon_with_holes_indexed(n, num_holes, &verts_2d);

        for [i0, i1, i2] in tri_indices {
            let p0 = verts_3d[i0];
            let p1 = verts_3d[i1];
            let p2 = verts_3d[i2];

            // Cap for negative mesh: outward normal +norm
            new_neg_tris.push(Triangle::with_normal([p0, p1, p2], norm));

            // Cap for positive mesh: outward normal -norm
            new_pos_tris.push(Triangle::with_normal([p0, p2, p1], -norm));
        }
    }

    // Add cavity walls and bottom faces for each socket using its adaptive safe depth
    for (hole, p) in holes.iter().zip(placements.iter()) {
        let depth = p.depth;
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
        .map(|p| {
            let pin_cfg = PinConfig {
                size: p.size,
                depth: p.depth,
                clearance: config.clearance,
                edge_margin: config.edge_margin,
                adaptive: false,
                min_size: p.size,
            };
            generate_square_pin(&pin_cfg)
        })
        .collect();

    (positive_mesh, negative_mesh, pin_meshes)
}

/// Applies female socket cavities into both `positive_mesh` and `negative_mesh` on the cut plane,
/// verifying that sockets do not pierce the outer shell and remain 100% closed solids.
pub fn apply_female_sockets_and_generate_pins(
    cut_result: &CutResult,
    config: &PinConfig,
) -> (Mesh, Mesh, Vec<Mesh>) {
    let mut try_configs = Vec::new();
    try_configs.push(*config);

    if config.adaptive {
        let candidate_sizes = [3.5f32, 3.0, 2.5, 2.0, 1.5];
        let min_sz = config.min_size.max(1.0);
        for &sz in &candidate_sizes {
            if sz < config.size && sz >= min_sz {
                let mut c = *config;
                c.size = sz;
                try_configs.push(c);
            }
        }
    }

    for cfg in try_configs {
        let placements = find_pin_placements(cut_result, &cfg);
        if placements.is_empty() {
            continue;
        }

        let (pos_m, neg_m, pins) = apply_sockets_for_placements(cut_result, &cfg, &placements);
        if pos_m.is_closed() && neg_m.is_closed() {
            return (pos_m, neg_m, pins);
        }
    }

    (
        cut_result.positive_mesh.clone(),
        cut_result.negative_mesh.clone(),
        Vec::new(),
    )
}

/// Applies female socket cavities and generates square pins with adaptive sizing enabled,
/// returning (positive_mesh, negative_mesh, pins, pin_size_used).
pub fn apply_female_sockets_and_generate_pins_adaptive(
    cut_result: &CutResult,
    config: &PinConfig,
    min_size: f32,
) -> (Mesh, Mesh, Vec<Mesh>, Option<f32>) {
    let mut cfg = *config;
    cfg.adaptive = true;
    cfg.min_size = min_size;
    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(cut_result, &cfg);
    let pin_sz = pins.first().map(|p| p.aabb().size().x);
    (pos, neg, pins, pin_sz)
}
