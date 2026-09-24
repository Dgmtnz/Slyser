use crate::bvh::BVH;
use crate::mesh::{Mesh, Triangle};
use glam::Vec3;
use rayon::prelude::*;

/// Configuration parameters for overhang detection and support volume estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupportConfig {
    /// Overhang angle threshold in degrees from the horizontal.
    /// Surfaces with angle to horizontal < `overhang_angle_deg` require support.
    /// Equivalent to downward normal having `-n_z >= sin(overhang_angle_deg)`.
    /// Default is 45.0 degrees.
    pub overhang_angle_deg: f32,

    /// Number of quadrature sampling points per mm² of projected XY area.
    /// Higher values yield finer integration around steps and model contours.
    /// Default is 2.0.
    pub sample_resolution: f32,
}

impl Default for SupportConfig {
    fn default() -> Self {
        Self {
            overhang_angle_deg: 45.0,
            sample_resolution: 2.0,
        }
    }
}

impl SupportConfig {
    /// Creates a new support configuration.
    pub fn new(overhang_angle_deg: f32, sample_resolution: f32) -> Self {
        Self {
            overhang_angle_deg,
            sample_resolution,
        }
    }

    /// Creates a configuration with a specific overhang threshold angle and default resolution.
    pub fn with_angle(overhang_angle_deg: f32) -> Self {
        Self {
            overhang_angle_deg,
            ..Default::default()
        }
    }
}

/// Computed metrics describing overhangs and required support structures.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SupportMetrics {
    /// Total 3D surface area of triangles in overhang in mm².
    pub overhang_area: f32,

    /// Total projected area of overhang surfaces onto the XY print bed plane in mm².
    pub projected_support_area: f32,

    /// Total estimated volume of support structures in mm³.
    pub estimated_support_volume: f32,

    /// Volume of support structures that land on the build plate (z = z_bed) in mm³.
    pub support_on_bed_volume: f32,

    /// Volume of support structures that land on lower surfaces of the mesh in mm³.
    pub support_on_model_volume: f32,

    /// Number of triangles classified as overhangs.
    pub overhang_triangle_count: usize,
}

/// Evaluates whether a triangle is an overhang requiring support structures.
///
/// Returns true if the triangle faces downward, does not rest flat on the build platform,
/// and has an angle to the horizontal plane strictly less than `config.overhang_angle_deg`.
pub fn is_overhang(tri: &Triangle, config: &SupportConfig, z_bed: f32) -> bool {
    if config.overhang_angle_deg <= 0.0 {
        return false;
    }

    // Must be pointing downward
    if tri.normal.z >= -1e-5 {
        return false;
    }

    // Faces resting on the print bed do not require support
    let all_on_bed = tri.vertices[0].z <= z_bed + 1e-4
        && tri.vertices[1].z <= z_bed + 1e-4
        && tri.vertices[2].z <= z_bed + 1e-4;
    if all_on_bed {
        return false;
    }

    // Slicer standard convention: angle to horizontal < overhang_angle_deg
    // Normal has n_z < 0 and -n_z >= sin(overhang_angle_deg)
    let threshold = config.overhang_angle_deg.to_radians().sin();
    -tri.normal.z >= threshold - 1e-4
}

/// Analyzes support requirements for a mesh in parallel across all available CPU threads.
pub fn analyze_support(mesh: &Mesh, config: &SupportConfig) -> SupportMetrics {
    if mesh.is_empty() {
        return SupportMetrics::default();
    }

    let aabb = mesh.aabb();
    let z_bed = aabb.min.z;

    let has_overhang = mesh.triangles.iter().any(|t| is_overhang(t, config, z_bed));
    if !has_overhang {
        return SupportMetrics::default();
    }

    let bvh = BVH::build(&mesh.triangles);

    mesh.triangles
        .par_iter()
        .enumerate()
        .filter_map(|(tri_idx, tri)| {
            if is_overhang(tri, config, z_bed) {
                Some(analyze_overhang_triangle(tri, tri_idx, &bvh, config, z_bed))
            } else {
                None
            }
        })
        .reduce(
            SupportMetrics::default,
            |mut a, b| {
                a.overhang_area += b.overhang_area;
                a.projected_support_area += b.projected_support_area;
                a.estimated_support_volume += b.estimated_support_volume;
                a.support_on_bed_volume += b.support_on_bed_volume;
                a.support_on_model_volume += b.support_on_model_volume;
                a.overhang_triangle_count += b.overhang_triangle_count;
                a
            },
        )
}

/// Evaluates support column volume for a single overhang triangle using quadrature integration.
fn analyze_overhang_triangle(
    tri: &Triangle,
    tri_idx: usize,
    bvh: &BVH,
    config: &SupportConfig,
    z_bed: f32,
) -> SupportMetrics {
    let area_3d = tri.area();
    let area_xy = area_3d * (-tri.normal.z);

    if area_xy <= 1e-6 {
        return SupportMetrics {
            overhang_area: area_3d,
            projected_support_area: 0.0,
            estimated_support_volume: 0.0,
            support_on_bed_volume: 0.0,
            support_on_model_volume: 0.0,
            overhang_triangle_count: 1,
        };
    }

    let res = config.sample_resolution.max(0.1);
    let target_samples = (area_xy * res).round() as usize;
    let k = ((target_samples as f32).sqrt().round() as usize).max(1);
    let total_sub = (k * k) as f32;
    let weight = area_xy / total_sub;

    let mut bed_vol = 0.0f32;
    let mut model_vol = 0.0f32;

    let inv_k = 1.0 / (k as f32);

    for i in 0..k {
        let i_f = i as f32;
        for j in 0..(k - i) {
            let j_f = j as f32;

            // Upward sub-triangle centroid
            let u1 = (i_f + 1.0 / 3.0) * inv_k;
            let v1 = (j_f + 1.0 / 3.0) * inv_k;
            let w1 = 1.0 - u1 - v1;
            let p1 = tri.vertices[0] * u1 + tri.vertices[1] * v1 + tri.vertices[2] * w1;
            eval_support_point(p1, weight, tri_idx, bvh, z_bed, &mut bed_vol, &mut model_vol);

            // Downward sub-triangle centroid
            if i + j + 1 < k {
                let u2 = (i_f + 2.0 / 3.0) * inv_k;
                let v2 = (j_f + 2.0 / 3.0) * inv_k;
                let w2 = 1.0 - u2 - v2;
                let p2 = tri.vertices[0] * u2 + tri.vertices[1] * v2 + tri.vertices[2] * w2;
                eval_support_point(p2, weight, tri_idx, bvh, z_bed, &mut bed_vol, &mut model_vol);
            }
        }
    }

    SupportMetrics {
        overhang_area: area_3d,
        projected_support_area: area_xy,
        estimated_support_volume: bed_vol + model_vol,
        support_on_bed_volume: bed_vol,
        support_on_model_volume: model_vol,
        overhang_triangle_count: 1,
    }
}

/// Casts a downward ray from a sample point on an overhang and accumulates column volume.
#[inline]
fn eval_support_point(
    p: Vec3,
    weight: f32,
    tri_idx: usize,
    bvh: &BVH,
    z_bed: f32,
    bed_vol: &mut f32,
    model_vol: &mut f32,
) {
    let dist_to_bed = p.z - z_bed;
    if dist_to_bed <= 1e-5 {
        return;
    }

    // Shoot ray directly downwards towards the build plate
    let hit = bvh.intersect_ray_with_options(
        p,
        Vec3::new(0.0, 0.0, -1.0),
        1e-4,
        dist_to_bed - 1e-4,
        Some(tri_idx),
    );

    if let Some(hit) = hit {
        *model_vol += weight * hit.t;
    } else {
        *bed_vol += weight * dist_to_bed;
    }
}
