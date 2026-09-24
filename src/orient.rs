use crate::mesh::Mesh;
use crate::support::{analyze_support, SupportConfig, SupportMetrics};
use glam::{Quat, Vec2, Vec3};
use rayon::prelude::*;

/// Candidate orientation for placing a part onto the print bed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrientationCandidate {
    /// Rotation quaternion that aligns the candidate base to the print bed.
    pub rotation: Quat,
    /// Surface normal on the original mesh corresponding to this resting face.
    pub contact_normal: Vec3,
    /// Estimated contact surface area of the planar base in mm².
    pub contact_area: f32,
}

/// The result of the orientation optimization process.
#[derive(Debug, Clone, PartialEq)]
pub struct OptimalOrientation {
    /// Rotation quaternion applied to the original mesh.
    pub rotation: Quat,
    /// Translation vector applied after rotation to place the mesh flat on the bed (z_min = 0).
    pub translation: Vec3,
    /// The reoriented and translated mesh resting flat on the print bed.
    pub mesh: Mesh,
    /// Support metrics evaluated for this orientation.
    pub metrics: SupportMetrics,
    /// Total surface area in contact with the print bed in mm².
    pub contact_area: f32,
    /// Whether the orientation is mechanically stable under gravity.
    pub is_stable: bool,
}

/// Computes the rotation quaternion that maps `normal` to downward along `-Z` (0, 0, -1).
pub fn rotation_to_down(normal: Vec3) -> Quat {
    let from = normal.normalize_or_zero();
    let to = Vec3::new(0.0, 0.0, -1.0);
    let dot = from.dot(to);

    if dot >= 1.0 - 1e-5 {
        Quat::IDENTITY
    } else if dot <= -1.0 + 1e-5 {
        // 180-degree flip around the X-axis
        Quat::from_rotation_x(std::f32::consts::PI)
    } else {
        let axis = from.cross(to).normalize();
        let angle = dot.clamp(-1.0, 1.0).acos();
        Quat::from_axis_angle(axis, angle)
    }
}

/// Represents an aggregated planar cluster of approximately coplanar triangles.
struct PlanarCluster {
    normal: Vec3,
    d: f32,
    area: f32,
}

/// Extracts candidate resting orientations from planar clusters and canonical coordinate axes.
pub fn extract_candidate_orientations(mesh: &Mesh) -> Vec<OrientationCandidate> {
    if mesh.is_empty() {
        return Vec::new();
    }

    let total_area = mesh.total_surface_area();
    let mut clusters: Vec<PlanarCluster> = Vec::new();

    let angular_threshold = 8.0f32.to_radians().cos(); // ~0.9902
    let dist_threshold = 0.2f32; // mm

    for tri in &mesh.triangles {
        let tri_area = tri.area();
        if tri_area <= 1e-6 {
            continue;
        }
        let tri_norm = tri.normal;
        let tri_d = tri_norm.dot(tri.center());

        let mut matched = false;
        for cl in &mut clusters {
            if cl.normal.dot(tri_norm) >= angular_threshold
                && (cl.d - tri_d).abs() <= dist_threshold
            {
                let new_area = cl.area + tri_area;
                cl.normal = (cl.normal * cl.area + tri_norm * tri_area) / new_area;
                cl.normal = cl.normal.normalize();
                cl.d = (cl.d * cl.area + tri_d * tri_area) / new_area;
                cl.area = new_area;
                matched = true;
                break;
            }
        }

        if !matched {
            clusters.push(PlanarCluster {
                normal: tri_norm,
                d: tri_d,
                area: tri_area,
            });
        }
    }

    // Sort clusters by surface area descending
    clusters.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let min_significant_area = total_area * 0.01; // 1% of total area
    let mut candidates: Vec<OrientationCandidate> = Vec::new();

    // Select clusters meeting significant area threshold, or at least the top 48 largest
    for (i, cl) in clusters.iter().enumerate() {
        if cl.area >= min_significant_area || i < 48 {
            let rot = rotation_to_down(cl.normal);
            candidates.push(OrientationCandidate {
                rotation: rot,
                contact_normal: cl.normal,
                contact_area: cl.area,
            });
        }
    }

    // Always include canonical coordinate axes (+-X, +-Y, +-Z)
    let canonical_axes = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
    ];

    for &axis in &canonical_axes {
        let already_present = candidates
            .iter()
            .any(|c| c.contact_normal.dot(axis) >= 0.995);
        if !already_present {
            let area: f32 = mesh
                .triangles
                .iter()
                .filter(|t| t.normal.dot(axis) >= 0.95)
                .map(|t| t.area())
                .sum();

            candidates.push(OrientationCandidate {
                rotation: rotation_to_down(axis),
                contact_normal: axis,
                contact_area: area,
            });
        }
    }

    // Deduplicate candidates that point in essentially the same direction
    let mut unique_candidates: Vec<OrientationCandidate> = Vec::new();
    for cand in candidates {
        let duplicate_idx = unique_candidates
            .iter()
            .position(|u| u.contact_normal.dot(cand.contact_normal) >= 0.995);

        if let Some(idx) = duplicate_idx {
            unique_candidates[idx].contact_area += cand.contact_area;
        } else {
            unique_candidates.push(cand);
        }
    }

    unique_candidates
}

/// Applies `rotation` to the mesh and translates it so `min.z == 0.0`.
pub fn orient_mesh_to_bed(mesh: &Mesh, rotation: Quat) -> Mesh {
    let mut rotated = mesh.rotated(rotation);
    let aabb = rotated.aabb();
    rotated.translate(Vec3::new(0.0, 0.0, -aabb.min.z));
    rotated
}

/// Computes the surface area of triangles resting flat on the print bed ($z \le 1e-3$, facing $-Z$).
pub fn compute_bed_contact_area(mesh: &Mesh) -> f32 {
    let mut area = 0.0f32;
    for tri in &mesh.triangles {
        let on_bed = tri.vertices[0].z <= 1e-3
            && tri.vertices[1].z <= 1e-3
            && tri.vertices[2].z <= 1e-3;
        if on_bed && tri.normal.z <= -0.9 {
            area += tri.area();
        }
    }
    area
}

/// 2D cross product of vectors (A - O) and (B - O).
#[inline]
fn cross_2d(o: Vec2, a: Vec2, b: Vec2) -> f32 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Computes the 2D convex hull using Andrew's monotone chain algorithm.
fn convex_hull_2d(mut pts: Vec<Vec2>) -> Vec<Vec2> {
    if pts.len() <= 2 {
        return pts;
    }

    pts.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
    });

    pts.dedup_by(|a, b| (a.x - b.x).abs() < 1e-4 && (a.y - b.y).abs() < 1e-4);
    if pts.len() <= 2 {
        return pts;
    }

    let n = pts.len();
    let mut hull = Vec::with_capacity(2 * n);

    // Lower hull
    for &p in &pts {
        while hull.len() >= 2 && cross_2d(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 1e-5 {
            hull.pop();
        }
        hull.push(p);
    }

    // Upper hull
    let lower_len = hull.len();
    for &p in pts.iter().rev().skip(1) {
        while hull.len() > lower_len
            && cross_2d(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 1e-5
        {
            hull.pop();
        }
        hull.push(p);
    }

    hull.pop();
    hull
}

/// Computes the 2D polygon area of a closed convex hull.
fn polygon_area_2d(hull: &[Vec2]) -> f32 {
    let mut area = 0.0f32;
    let n = hull.len();
    for i in 0..n {
        let j = (i + 1) % n;
        area += hull[i].x * hull[j].y - hull[j].x * hull[i].y;
    }
    (area * 0.5).abs()
}

/// Tests whether point `p` is contained inside the 2D convex polygon `hull` (oriented CCW).
fn point_in_convex_polygon_2d(p: Vec2, hull: &[Vec2]) -> bool {
    let n = hull.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        let j = (i + 1) % n;
        if cross_2d(hull[i], hull[j], p) < -1e-3 {
            return false;
        }
    }
    true
}

/// Verifies whether a mesh resting at $z=0$ is mechanically stable under gravity.
pub fn is_orientation_stable(mesh_on_bed: &Mesh) -> bool {
    if mesh_on_bed.is_empty() {
        return false;
    }

    let com = mesh_on_bed.center_of_mass();
    let com_xy = Vec2::new(com.x, com.y);

    let mut contact_points = Vec::new();
    for tri in &mesh_on_bed.triangles {
        for v in &tri.vertices {
            if v.z <= 1e-3 {
                contact_points.push(Vec2::new(v.x, v.y));
            }
        }
    }

    if contact_points.len() < 3 {
        return false;
    }

    let hull = convex_hull_2d(contact_points);
    if hull.len() < 3 {
        return false;
    }

    if polygon_area_2d(&hull) < 1e-3 {
        return false;
    }

    point_in_convex_polygon_2d(com_xy, &hull)
}

/// Evaluated candidate orientation metrics without retaining cloned meshes in memory.
#[derive(Debug, Clone, PartialEq)]
struct EvaluatedOrientation {
    rotation: Quat,
    translation: Vec3,
    metrics: SupportMetrics,
    contact_area: f32,
    is_stable: bool,
}

/// Evaluates whether candidate `a` is preferable to candidate `b` according to multi-objective fitness.
fn is_better_eval(a: &EvaluatedOrientation, b: &EvaluatedOrientation) -> bool {
    // Primary: minimize estimated_support_volume
    let vol_diff = a.metrics.estimated_support_volume - b.metrics.estimated_support_volume;
    if vol_diff.abs() > 1e-2 {
        return vol_diff < 0.0;
    }

    // Secondary: minimize support_on_model_volume (penalty for scarring part surfaces)
    let model_diff = a.metrics.support_on_model_volume - b.metrics.support_on_model_volume;
    if model_diff.abs() > 1e-2 {
        return model_diff < 0.0;
    }

    // Tertiary: maximize contact_area (bed adhesion)
    let area_diff = a.contact_area - b.contact_area;
    if area_diff.abs() > 1e-2 {
        return area_diff > 0.0;
    }

    // Quaternary: prefer stable orientations
    if a.is_stable != b.is_stable {
        return a.is_stable;
    }

    false
}

/// Evaluates whether candidate `a` is preferable to candidate `b` according to multi-objective fitness.
#[allow(dead_code)]
fn is_better(a: &OptimalOrientation, b: &OptimalOrientation) -> bool {
    let ea = EvaluatedOrientation {
        rotation: a.rotation,
        translation: a.translation,
        metrics: a.metrics.clone(),
        contact_area: a.contact_area,
        is_stable: a.is_stable,
    };
    let eb = EvaluatedOrientation {
        rotation: b.rotation,
        translation: b.translation,
        metrics: b.metrics.clone(),
        contact_area: b.contact_area,
        is_stable: b.is_stable,
    };
    is_better_eval(&ea, &eb)
}

/// Evaluates candidate orientations in parallel across all CPU threads and returns the optimal orientation.
pub fn find_optimal_orientation(
    mesh: &Mesh,
    support_config: &SupportConfig,
) -> OptimalOrientation {
    if mesh.is_empty() {
        return OptimalOrientation {
            rotation: Quat::IDENTITY,
            translation: Vec3::ZERO,
            mesh: Mesh::empty(),
            metrics: SupportMetrics::default(),
            contact_area: 0.0,
            is_stable: false,
        };
    }

    let candidates = extract_candidate_orientations(mesh);

    let eval_one = |rot: Quat| -> EvaluatedOrientation {
        let mut rotated = mesh.rotated(rot);
        let aabb = rotated.aabb();
        let translation = Vec3::new(0.0, 0.0, -aabb.min.z);
        rotated.translate(translation);

        let metrics = analyze_support(&rotated, support_config);
        let contact_area = compute_bed_contact_area(&rotated);
        let is_stable = is_orientation_stable(&rotated);

        EvaluatedOrientation {
            rotation: rot,
            translation,
            metrics,
            contact_area,
            is_stable,
        }
    };

    let mut evaluated: Vec<EvaluatedOrientation> = candidates
        .into_par_iter()
        .map(|cand| eval_one(cand.rotation))
        .collect();

    // Sort to determine the top candidate orientations (best first)
    evaluated.sort_by(|a, b| {
        if is_better_eval(a, b) {
            std::cmp::Ordering::Less
        } else if is_better_eval(b, a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    // For the top 3 candidate orientations, add fine tilt evaluations
    // (±2.5° and ±5.0° around orthogonal axes) to discover exact zero-support resting orientations.
    let top_candidates: Vec<Quat> = evaluated
        .iter()
        .take(3)
        .map(|e| e.rotation)
        .collect();

    let tilt_angles = [-5.0f32, -2.5, 2.5, 5.0];
    let mut fine_tilt_rotations: Vec<Quat> = Vec::new();

    for &base_rot in &top_candidates {
        for &deg in &tilt_angles {
            let rad = deg.to_radians();
            fine_tilt_rotations.push(Quat::from_rotation_x(rad) * base_rot);
            fine_tilt_rotations.push(Quat::from_rotation_y(rad) * base_rot);
        }
    }

    let fine_evaluated: Vec<EvaluatedOrientation> = fine_tilt_rotations
        .into_par_iter()
        .map(eval_one)
        .collect();

    evaluated.extend(fine_evaluated);

    let best = evaluated
        .into_iter()
        .reduce(|best, current| {
            if is_better_eval(&current, &best) {
                current
            } else {
                best
            }
        })
        .unwrap_or_else(|| eval_one(Quat::IDENTITY));

    let mut oriented = mesh.rotated(best.rotation);
    oriented.translate(best.translation);

    OptimalOrientation {
        rotation: best.rotation,
        translation: best.translation,
        mesh: oriented,
        metrics: best.metrics,
        contact_area: best.contact_area,
        is_stable: best.is_stable,
    }
}
