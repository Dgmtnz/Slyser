use crate::mesh::{Mesh, Plane, Triangle};
use glam::{Vec2, Vec3};
use std::collections::HashMap;

/// Result of a planar mesh cut operation.
#[derive(Debug, Clone)]
pub struct CutResult {
    /// Half-space mesh where (p - P0) . n >= 0
    pub positive_mesh: Mesh,
    /// Half-space mesh where (p - P0) . n <= 0
    pub negative_mesh: Mesh,
    /// Plane used for the cut
    pub cut_plane: Plane,
    /// Surface area of the cross-section / cap in mm²
    pub cut_area: f32,
    /// 2D closed polygon contours of the cut face in local plane coordinates
    pub cut_polygons_2d: Vec<Vec<Vec2>>,
    /// Unsimplified 2D closed boundary loops matching the exact sliced mesh vertices
    pub cut_loops_2d: Vec<Vec<Vec2>>,
    /// Tangent and bitangent orthonormal vectors defining the 2D local plane coordinate system (u_axis, v_axis)
    pub plane_basis: (Vec3, Vec3),
}

/// Computes an orthonormal basis (u_axis, v_axis) for a given plane normal.
/// Guarantees that u_axis x v_axis == normal.
pub fn compute_plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let n = normal.normalize_or_zero();
    let u_axis = if n.z.abs() < 0.9 {
        n.cross(Vec3::Z).normalize_or_zero()
    } else {
        n.cross(Vec3::Y).normalize_or_zero()
    };
    let v_axis = n.cross(u_axis).normalize_or_zero();
    (u_axis, v_axis)
}

/// Projects a 3D point onto the 2D local plane coordinate system defined by (p0, u_axis, v_axis).
#[inline]
pub fn project_to_plane_2d(p: Vec3, p0: Vec3, u_axis: Vec3, v_axis: Vec3) -> Vec2 {
    let d = p - p0;
    Vec2::new(d.dot(u_axis), d.dot(v_axis))
}

/// Unprojects a 2D local plane point back into 3D world coordinates.
#[inline]
pub fn unproject_from_plane_2d(uv: Vec2, p0: Vec3, u_axis: Vec3, v_axis: Vec3) -> Vec3 {
    p0 + uv.x * u_axis + uv.y * v_axis
}

type PointKey = (i64, i64, i64);
type DirectedEdgeKey = (PointKey, PointKey);

#[inline]
fn quantize_point(p: Vec3) -> PointKey {
    (
        (p.x * 1e4).round() as i64,
        (p.y * 1e4).round() as i64,
        (p.z * 1e4).round() as i64,
    )
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

/// Triangulates a simple 2D CCW polygon into a list of triangle index triplets using ear clipping.
fn triangulate_polygon_2d(poly: &[Vec2]) -> Vec<[usize; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![[0, 1, 2]];
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
            if cross <= 1e-7 {
                continue;
            }

            let mut has_point_inside = false;
            for &other_idx in &indices {
                if other_idx == prev_idx || other_idx == curr_idx || other_idx == next_idx {
                    continue;
                }
                let pt = poly[other_idx];
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
            triangles.push([prev_idx, curr_idx, next_idx]);
            indices.remove(best_k);
        }
    }

    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }

    triangles
}

/// Simplifies a 2D closed polygon loop by removing redundant collinear intermediate vertices.
fn simplify_collinear_2d(poly: &[Vec2]) -> Vec<Vec2> {
    if poly.len() <= 3 {
        return poly.to_vec();
    }

    let mut pts = poly.to_vec();
    let mut changed = true;
    while changed && pts.len() > 3 {
        changed = false;
        let count = pts.len();
        for i in 0..count {
            let prev = (i + count - 1) % count;
            let next = (i + 1) % count;

            let v1 = pts[i] - pts[prev];
            let v2 = pts[next] - pts[i];

            let len1 = v1.length();
            let len2 = v2.length();
            if len1 < 1e-5 || len2 < 1e-5 {
                pts.remove(i);
                changed = true;
                break;
            }

            let cross = (v1.x * v2.y - v1.y * v2.x).abs();
            let dot = v1.dot(v2);

            if cross <= 1e-4 * len1 * len2 && dot > 0.0 {
                pts.remove(i);
                changed = true;
                break;
            }
        }
    }

    pts
}


/// Slices a 3D mesh by an arbitrary plane, caps both cut surfaces to make them watertight,
/// and verifies volume conservation.
pub fn slice_mesh_by_plane(mesh: &Mesh, plane: &Plane) -> Result<CutResult, String> {

    if mesh.is_empty() {
        return Err("Mesh contains no triangles".to_string());
    }
    if plane.normal.length_squared() < 1e-6 {
        return Err("Plane normal cannot be zero".to_string());
    }

    let norm = plane.normal.normalize();
    let normalized_plane = Plane::new(plane.point, norm);
    let (u_axis, v_axis) = compute_plane_basis(norm);

    let eps = 1e-5f32;

    let mut pos_tris: Vec<Triangle> = Vec::with_capacity(mesh.triangles.len());
    let mut neg_tris: Vec<Triangle> = Vec::with_capacity(mesh.triangles.len());

    for tri in &mesh.triangles {
        let mut v = tri.vertices;
        let mut d = [
            (v[0] - normalized_plane.point).dot(norm),
            (v[1] - normalized_plane.point).dot(norm),
            (v[2] - normalized_plane.point).dot(norm),
        ];

        // Snap near-zero vertices to plane
        for i in 0..3 {
            if d[i].abs() <= eps {
                v[i] -= d[i] * norm;
                d[i] = 0.0;
            }
        }

        let s = [
            if d[0] > eps { 1 } else if d[0] < -eps { -1 } else { 0 },
            if d[1] > eps { 1 } else if d[1] < -eps { -1 } else { 0 },
            if d[2] > eps { 1 } else if d[2] < -eps { -1 } else { 0 },
        ];

        let num_pos = (s[0] == 1) as usize + (s[1] == 1) as usize + (s[2] == 1) as usize;
        let num_neg = (s[0] == -1) as usize + (s[1] == -1) as usize + (s[2] == -1) as usize;
        let num_zero = 3 - num_pos - num_neg;

        if num_zero == 3 {
            // Coplanar triangle: place in positive or negative half based on normal
            if tri.normal.dot(norm) >= 0.0 {
                pos_tris.push(Triangle::with_normal(v, tri.normal));
            } else {
                neg_tris.push(Triangle::with_normal(v, tri.normal));
            }
        } else if num_neg == 0 {
            // All vertices >= 0
            pos_tris.push(Triangle::with_normal(v, tri.normal));
        } else if num_pos == 0 {
            // All vertices <= 0
            neg_tris.push(Triangle::with_normal(v, tri.normal));
        } else if num_zero == 1 {
            // 1 zero, 1 pos, 1 neg
            let zero_idx = if s[0] == 0 { 0 } else if s[1] == 0 { 1 } else { 2 };
            let v0 = v[zero_idx];
            let v1 = v[(zero_idx + 1) % 3];
            let v2 = v[(zero_idx + 2) % 3];
            let d1 = d[(zero_idx + 1) % 3];
            let d2 = d[(zero_idx + 2) % 3];
            let s1 = s[(zero_idx + 1) % 3];

            let t = (d1 / (d1 - d2)).clamp(0.0, 1.0);
            let mut inter = v1 + t * (v2 - v1);
            inter -= (inter - normalized_plane.point).dot(norm) * norm;

            if s1 > 0 {
                pos_tris.push(Triangle::with_normal([v0, v1, inter], tri.normal));
                neg_tris.push(Triangle::with_normal([v0, inter, v2], tri.normal));
            } else {
                neg_tris.push(Triangle::with_normal([v0, v1, inter], tri.normal));
                pos_tris.push(Triangle::with_normal([v0, inter, v2], tri.normal));
            }
        } else if num_pos == 1 && num_neg == 2 {
            // 1 positive, 2 negative
            let pos_idx = if s[0] == 1 { 0 } else if s[1] == 1 { 1 } else { 2 };
            let v0 = v[pos_idx];
            let v1 = v[(pos_idx + 1) % 3];
            let v2 = v[(pos_idx + 2) % 3];
            let d0 = d[pos_idx];
            let d1 = d[(pos_idx + 1) % 3];
            let d2 = d[(pos_idx + 2) % 3];

            let t01 = (d0 / (d0 - d1)).clamp(0.0, 1.0);
            let mut i01 = v0 + t01 * (v1 - v0);
            i01 -= (i01 - normalized_plane.point).dot(norm) * norm;

            let t02 = (d0 / (d0 - d2)).clamp(0.0, 1.0);
            let mut i02 = v0 + t02 * (v2 - v0);
            i02 -= (i02 - normalized_plane.point).dot(norm) * norm;

            pos_tris.push(Triangle::with_normal([v0, i01, i02], tri.normal));
            neg_tris.push(Triangle::with_normal([i01, v1, v2], tri.normal));
            neg_tris.push(Triangle::with_normal([i01, v2, i02], tri.normal));
        } else if num_pos == 2 && num_neg == 1 {
            // 2 positive, 1 negative
            let neg_idx = if s[0] == -1 { 0 } else if s[1] == -1 { 1 } else { 2 };
            let v0 = v[neg_idx];
            let v1 = v[(neg_idx + 1) % 3];
            let v2 = v[(neg_idx + 2) % 3];
            let d0 = d[neg_idx];
            let d1 = d[(neg_idx + 1) % 3];
            let d2 = d[(neg_idx + 2) % 3];

            let t01 = (d0 / (d0 - d1)).clamp(0.0, 1.0);
            let mut i01 = v0 + t01 * (v1 - v0);
            i01 -= (i01 - normalized_plane.point).dot(norm) * norm;

            let t02 = (d0 / (d0 - d2)).clamp(0.0, 1.0);
            let mut i02 = v0 + t02 * (v2 - v0);
            i02 -= (i02 - normalized_plane.point).dot(norm) * norm;

            neg_tris.push(Triangle::with_normal([v0, i01, i02], tri.normal));
            pos_tris.push(Triangle::with_normal([i01, v1, v2], tri.normal));
            pos_tris.push(Triangle::with_normal([i01, v2, i02], tri.normal));
        }
    }

    // Collect unpaired directed edges on the cut plane from the positive mesh
    let mut edge_map: HashMap<DirectedEdgeKey, (Vec3, Vec3)> = HashMap::new();

    for tri in &pos_tris {
        for e in 0..3 {
            let a = tri.vertices[e];
            let b = tri.vertices[(e + 1) % 3];

            let da = (a - normalized_plane.point).dot(norm).abs();
            let db = (b - normalized_plane.point).dot(norm).abs();

            if da <= eps * 2.0 && db <= eps * 2.0 {
                let qa = quantize_point(a);
                let qb = quantize_point(b);
                if qa != qb {
                    let key_ab = (qa, qb);
                    let key_ba = (qb, qa);

                    if edge_map.remove(&key_ba).is_none() {
                        edge_map.insert(key_ab, (a, b));
                    }
                }
            }
        }
    }

    if edge_map.is_empty() && (pos_tris.is_empty() || neg_tris.is_empty()) {
        return Err("Cut plane does not intersect the mesh".to_string());
    }

    // Chain cut boundary edges into closed loops
    let mut adj: HashMap<PointKey, Vec<(Vec3, Vec3)>> = HashMap::with_capacity(edge_map.len());
    for (_k, (a, b)) in edge_map {
        adj.entry(quantize_point(a)).or_default().push((a, b));
    }

    let mut loops: Vec<Vec<Vec3>> = Vec::new();

    while let Some(&start_key) = adj.keys().next() {
        let edges_at_start = adj.get_mut(&start_key).unwrap();
        let (curr_a, mut curr_b) = edges_at_start.pop().unwrap();
        if edges_at_start.is_empty() {
            adj.remove(&start_key);
        }

        let mut loop_pts = vec![curr_a];
        let original_start_key = start_key;

        loop {
            let next_key = quantize_point(curr_b);
            if next_key == original_start_key || (curr_b - loop_pts[0]).length() < 1e-4 {
                break;
            }

            if let Some(next_edges) = adj.get_mut(&next_key) {
                let (next_a, next_b) = next_edges.pop().unwrap();
                if next_edges.is_empty() {
                    adj.remove(&next_key);
                }
                loop_pts.push(next_a);
                curr_b = next_b;
            } else {
                let mut best_dist = 1e-2f32;
                let mut best_key = None;
                for &k in adj.keys() {
                    if let Some(edge) = adj.get(&k).and_then(|list| list.last()) {
                        let d = (edge.0 - curr_b).length();
                        if d < best_dist {
                            best_dist = d;
                            best_key = Some(k);
                        }
                    }
                }


                if let Some(bk) = best_key {
                    let edges = adj.get_mut(&bk).unwrap();
                    let (next_a, next_b) = edges.pop().unwrap();
                    if edges.is_empty() {
                        adj.remove(&bk);
                    }
                    loop_pts.push(next_a);
                    curr_b = next_b;
                } else {
                    break;
                }
            }
        }


        let mut clean_pts: Vec<Vec3> = Vec::with_capacity(loop_pts.len());
        for p in loop_pts {
            if clean_pts.is_empty() || (*clean_pts.last().unwrap() - p).length() > 1e-5 {
                clean_pts.push(p);
            }
        }
        if clean_pts.len() >= 3 && (clean_pts[0] - *clean_pts.last().unwrap()).length() <= 1e-5 {
            clean_pts.pop();
        }

        if clean_pts.len() >= 3 {
            loops.push(clean_pts);
        }
    }

    let mut total_cut_area = 0.0f32;
    let mut cut_polygons_2d: Vec<Vec<Vec2>> = Vec::new();
    let mut cut_loops_2d: Vec<Vec<Vec2>> = Vec::new();

    for mut loop_3d in loops {
        let n = loop_3d.len();
        if n < 3 {
            continue;
        }

        let mut loop_2d: Vec<Vec2> = loop_3d
            .iter()
            .map(|&p| project_to_plane_2d(p, normalized_plane.point, u_axis, v_axis))
            .collect();

        let mut area2d = 0.0f32;
        for i in 0..n {
            let j = (i + 1) % n;
            area2d += loop_2d[i].x * loop_2d[j].y - loop_2d[j].x * loop_2d[i].y;
        }
        area2d *= 0.5;

        // Ensure CCW orientation
        if area2d < 0.0 {
            loop_3d.reverse();
            loop_2d.reverse();
            area2d = -area2d;
        }

        total_cut_area += area2d;
        let simplified_2d = simplify_collinear_2d(&loop_2d);
        cut_polygons_2d.push(simplified_2d);
        cut_loops_2d.push(loop_2d.clone());

        // Triangulate cap

        let tri_indices = triangulate_polygon_2d(&loop_2d);
        for [i0, i1, i2] in tri_indices {
            let p0 = loop_3d[i0];
            let p1 = loop_3d[i1];
            let p2 = loop_3d[i2];

            // Cap for negative mesh: outward normal +norm
            neg_tris.push(Triangle::with_normal([p0, p1, p2], norm));

            // Cap for positive mesh: outward normal -norm
            pos_tris.push(Triangle::with_normal([p0, p2, p1], -norm));
        }
    }

    let positive_mesh = Mesh::new(pos_tris);
    let negative_mesh = Mesh::new(neg_tris);

    let orig_vol = mesh.total_volume();
    let pos_vol = positive_mesh.total_volume();
    let neg_vol = negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    if orig_vol > 1e-4 {
        let diff = (sum_vol - orig_vol).abs() / orig_vol;
        if diff > 0.01 {
            return Err(format!(
                "Volume conservation failed: original {:.3} mm³, sum {:.3} mm³ (diff: {:.2}%)",
                orig_vol, sum_vol, diff * 100.0
            ));
        }
    }

    Ok(CutResult {
        positive_mesh,
        negative_mesh,
        cut_plane: normalized_plane,
        cut_area: total_cut_area,
        cut_polygons_2d,
        cut_loops_2d,
        plane_basis: (u_axis, v_axis),
    })
}

