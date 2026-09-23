use crate::cut::{compute_plane_basis, slice_mesh_by_plane};
use crate::mesh::{Mesh, Plane, Triangle, AABB};
use crate::orient::{find_optimal_orientation, OptimalOrientation};
use crate::pins::{apply_female_sockets_and_generate_pins, PinConfig};
use crate::support::{analyze_support, is_overhang, SupportConfig, SupportMetrics};
use glam::Vec3;
use rayon::prelude::*;

/// Configuration parameters for the Pareto partition optimizer.
#[derive(Debug, Clone, PartialEq)]
pub struct OptimizerConfig {
    /// Maximum number of resulting pieces (default: 2).
    pub max_pieces: usize,
    /// Support detection and volume estimation configuration.
    pub support_config: SupportConfig,
    /// Optional square pin and female socket configuration.
    pub pin_config: Option<PinConfig>,
    /// Pareto weight for support volume minimization (default: 1.0).
    pub weight_support: f32,
    /// Penalty per mm² of cut cross-sectional area (default: 0.001).
    pub weight_cut_area: f32,
    /// Minimum volume ratio for resulting pieces to reject tiny slivers (default: 0.02 = 2%).
    pub min_piece_volume_ratio: f32,
    /// Sampling resolution for candidate planes along axes (default: 20).
    pub num_candidate_planes: usize,
    /// Enforce single contiguous solid guarantee on all pieces (default: true).
    pub enforce_contiguous: bool,
    /// Minimum allowed square pin size in mm for adaptive pin sizing (default: 1.5).
    pub pin_min_size: f32,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            max_pieces: 2,
            support_config: SupportConfig::default(),
            pin_config: None,
            weight_support: 1.0,
            weight_cut_area: 0.001,
            min_piece_volume_ratio: 0.02,
            num_candidate_planes: 20,
            enforce_contiguous: true,
            pin_min_size: 1.5,
        }
    }
}

impl OptimizerConfig {
    /// Creates a default optimizer configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the maximum number of pieces.
    pub fn with_max_pieces(mut self, max_pieces: usize) -> Self {
        self.max_pieces = max_pieces;
        self
    }

    /// Sets the overhang threshold angle in degrees.
    pub fn with_support_angle(mut self, angle_deg: f32) -> Self {
        self.support_config.overhang_angle_deg = angle_deg;
        self
    }

    /// Enables square alignment pins with the given configuration.
    pub fn with_pins(mut self, pin_config: PinConfig) -> Self {
        self.pin_config = Some(pin_config);
        self
    }

    /// Sets whether to enforce that every resulting piece is a single contiguous solid.
    pub fn with_enforce_contiguous(mut self, enforce: bool) -> Self {
        self.enforce_contiguous = enforce;
        self
    }

    /// Sets the minimum pin size in mm for adaptive sizing.
    pub fn with_pin_min_size(mut self, min_size: f32) -> Self {
        self.pin_min_size = min_size;
        self
    }
}

/// Represents a partitioned piece oriented flat on the build plate (z_min = 0).
#[derive(Debug, Clone)]
pub struct PartitionPiece {
    /// 1-based unique identifier.
    pub id: usize,
    /// The reoriented mesh resting flat on the build plate.
    pub mesh: Mesh,
    /// Support metrics for this piece.
    pub metrics: SupportMetrics,
    /// Surface area in contact with the print bed in mm².
    pub contact_area: f32,
}

/// Comprehensive result of the multi-piece partition optimization process.
#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// Support metrics of the original mesh as provided.
    pub original_metrics: SupportMetrics,
    /// The resulting partition pieces oriented flat on the build plate.
    pub pieces: Vec<PartitionPiece>,
    /// Standalone square alignment pin meshes placed flat on z=0.
    pub pins: Vec<Mesh>,
    /// Total number of cuts performed.
    pub cuts_made: usize,
    /// Combined support volume of all resulting pieces in mm³.
    pub total_support_volume: f32,
    /// Percentage reduction in support volume compared to the original mesh.
    pub total_reduction_pct: f32,
    /// Total computation time in milliseconds.
    pub computation_time_ms: f64,
    /// Size of square alignment pins used in mm, if pins were generated.
    pub pin_size_used: Option<f32>,
}

/// Performs Jacobi eigenvalue decomposition on a real symmetric 3x3 matrix.
/// Returns (eigenvectors_as_columns, eigenvalues).
#[allow(clippy::needless_range_loop)]
fn jacobi_eigen_3x3(mut a: [[f32; 3]; 3]) -> ([[f32; 3]; 3], [f32; 3]) {
    let mut v = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    for _ in 0..30 {
        let mut max_val = 0.0f32;
        let mut p = 0;
        let mut q = 1;

        for i in 0..3 {
            for j in (i + 1)..3 {
                let abs_val = a[i][j].abs();
                if abs_val > max_val {
                    max_val = abs_val;
                    p = i;
                    q = j;
                }
            }
        }

        if max_val < 1e-6 {
            break;
        }

        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];

        let theta = 0.5 * (2.0 * apq).atan2(app - aqq);
        let c = theta.cos();
        let s = theta.sin();

        let mut new_a = a;
        for i in 0..3 {
            if i != p && i != q {
                new_a[i][p] = c * a[i][p] + s * a[i][q];
                new_a[p][i] = new_a[i][p];
                new_a[i][q] = -s * a[i][p] + c * a[i][q];
                new_a[q][i] = new_a[i][q];
            }
        }
        new_a[p][p] = c * c * app + 2.0 * s * c * apq + s * s * aqq;
        new_a[q][q] = s * s * app - 2.0 * s * c * apq + c * c * aqq;
        new_a[p][q] = 0.0;
        new_a[q][p] = 0.0;
        a = new_a;

        for i in 0..3 {
            let vip = v[i][p];
            let viq = v[i][q];
            v[i][p] = c * vip + s * viq;
            v[i][q] = -s * vip + c * viq;
        }
    }

    let eigenvalues = [a[0][0], a[1][1], a[2][2]];
    (v, eigenvalues)
}

/// Computes the cross-sectional area of the mesh sliced by plane Z = z using Green's Theorem.
fn compute_cross_section_area_z(mesh: &Mesh, z: f32) -> f32 {
    let mut area = 0.0f32;
    let z_test = z + 1e-4;

    for tri in &mesh.triangles {
        let v = tri.vertices;
        let d = [v[0].z - z_test, v[1].z - z_test, v[2].z - z_test];
        let s = [
            if d[0] > 0.0 { 1 } else { -1 },
            if d[1] > 0.0 { 1 } else { -1 },
            if d[2] > 0.0 { 1 } else { -1 },
        ];
        let num_pos = (s[0] == 1) as usize + (s[1] == 1) as usize + (s[2] == 1) as usize;
        if num_pos == 1 || num_pos == 2 {
            let (v_single, v_pair1, v_pair2, invert) = if num_pos == 1 {
                let single_idx = if s[0] == 1 { 0 } else if s[1] == 1 { 1 } else { 2 };
                (
                    v[single_idx],
                    v[(single_idx + 1) % 3],
                    v[(single_idx + 2) % 3],
                    false,
                )
            } else {
                let neg_idx = if s[0] == -1 { 0 } else if s[1] == -1 { 1 } else { 2 };
                (
                    v[neg_idx],
                    v[(neg_idx + 1) % 3],
                    v[(neg_idx + 2) % 3],
                    true,
                )
            };

            let denom1 = v_pair1.z - v_single.z;
            let denom2 = v_pair2.z - v_single.z;
            if denom1.abs() > 1e-6 && denom2.abs() > 1e-6 {
                let t1 = ((z_test - v_single.z) / denom1).clamp(0.0, 1.0);
                let p1 = v_single + t1 * (v_pair1 - v_single);
                let t2 = ((z_test - v_single.z) / denom2).clamp(0.0, 1.0);
                let p2 = v_single + t2 * (v_pair2 - v_single);

                let (p_start, p_end) = if invert { (p2, p1) } else { (p1, p2) };
                area += 0.5 * (p_start.x * p_end.y - p_end.x * p_start.y);
            }
        }
    }
    area.abs()
}

/// Computes the cross-sectional area of the mesh sliced by plane X = x using Green's Theorem.
fn compute_cross_section_area_x(mesh: &Mesh, x: f32) -> f32 {
    let mut area = 0.0f32;
    let x_test = x + 1e-4;

    for tri in &mesh.triangles {
        let v = tri.vertices;
        let d = [v[0].x - x_test, v[1].x - x_test, v[2].x - x_test];
        let s = [
            if d[0] > 0.0 { 1 } else { -1 },
            if d[1] > 0.0 { 1 } else { -1 },
            if d[2] > 0.0 { 1 } else { -1 },
        ];
        let num_pos = (s[0] == 1) as usize + (s[1] == 1) as usize + (s[2] == 1) as usize;
        if num_pos == 1 || num_pos == 2 {
            let (v_single, v_pair1, v_pair2, invert) = if num_pos == 1 {
                let single_idx = if s[0] == 1 { 0 } else if s[1] == 1 { 1 } else { 2 };
                (
                    v[single_idx],
                    v[(single_idx + 1) % 3],
                    v[(single_idx + 2) % 3],
                    false,
                )
            } else {
                let neg_idx = if s[0] == -1 { 0 } else if s[1] == -1 { 1 } else { 2 };
                (
                    v[neg_idx],
                    v[(neg_idx + 1) % 3],
                    v[(neg_idx + 2) % 3],
                    true,
                )
            };

            let denom1 = v_pair1.x - v_single.x;
            let denom2 = v_pair2.x - v_single.x;
            if denom1.abs() > 1e-6 && denom2.abs() > 1e-6 {
                let t1 = ((x_test - v_single.x) / denom1).clamp(0.0, 1.0);
                let p1 = v_single + t1 * (v_pair1 - v_single);
                let t2 = ((x_test - v_single.x) / denom2).clamp(0.0, 1.0);
                let p2 = v_single + t2 * (v_pair2 - v_single);

                let (p_start, p_end) = if invert { (p2, p1) } else { (p1, p2) };
                area += 0.5 * (p_start.y * p_end.z - p_end.y * p_start.z);
            }
        }
    }
    area.abs()
}

/// Computes the cross-sectional area of the mesh sliced by plane Y = y using Green's Theorem.
fn compute_cross_section_area_y(mesh: &Mesh, y: f32) -> f32 {
    let mut area = 0.0f32;
    let y_test = y + 1e-4;

    for tri in &mesh.triangles {
        let v = tri.vertices;
        let d = [v[0].y - y_test, v[1].y - y_test, v[2].y - y_test];
        let s = [
            if d[0] > 0.0 { 1 } else { -1 },
            if d[1] > 0.0 { 1 } else { -1 },
            if d[2] > 0.0 { 1 } else { -1 },
        ];
        let num_pos = (s[0] == 1) as usize + (s[1] == 1) as usize + (s[2] == 1) as usize;
        if num_pos == 1 || num_pos == 2 {
            let (v_single, v_pair1, v_pair2, invert) = if num_pos == 1 {
                let single_idx = if s[0] == 1 { 0 } else if s[1] == 1 { 1 } else { 2 };
                (
                    v[single_idx],
                    v[(single_idx + 1) % 3],
                    v[(single_idx + 2) % 3],
                    false,
                )
            } else {
                let neg_idx = if s[0] == -1 { 0 } else if s[1] == -1 { 1 } else { 2 };
                (
                    v[neg_idx],
                    v[(neg_idx + 1) % 3],
                    v[(neg_idx + 2) % 3],
                    true,
                )
            };

            let denom1 = v_pair1.y - v_single.y;
            let denom2 = v_pair2.y - v_single.y;
            if denom1.abs() > 1e-6 && denom2.abs() > 1e-6 {
                let t1 = ((y_test - v_single.y) / denom1).clamp(0.0, 1.0);
                let p1 = v_single + t1 * (v_pair1 - v_single);
                let t2 = ((y_test - v_single.y) / denom2).clamp(0.0, 1.0);
                let p2 = v_single + t2 * (v_pair2 - v_single);

                let (p_start, p_end) = if invert { (p2, p1) } else { (p1, p2) };
                area += 0.5 * (p_start.x * p_end.z - p_end.x * p_start.z);
            }
        }
    }
    area.abs()
}

/// Intelligent generator of candidate cutting planes using geometric and overhang heuristics.
pub struct CandidatePlaneGenerator;

impl CandidatePlaneGenerator {
    /// Generates candidate cutting planes based on:
    /// 1. Overhang boundary planes (Z-levels and coordinate planes at overhang junctions).
    /// 2. Geometric bottleneck planes (narrow necks and minimal cross-sections).
    /// 3. PCA / Principal axes planes of overhang clusters.
    /// 4. Planes offset along major axes between bounding box extents.
    /// Generates candidate planes tilted by +-15, +-25, +-35 degrees
    /// around base_normal (pitch, roll, and 45-degree diagonal combinations) through pivot.
    pub fn generate_tilted_planes(pivot: Vec3, base_normal: Vec3) -> Vec<Plane> {
        let (u, v) = compute_plane_basis(base_normal);
        let angles = [15.0f32, 25.0, 35.0];
        let azimuths = [0.0f32, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0];
        let mut planes = Vec::with_capacity(angles.len() * azimuths.len());

        for &ang_deg in &angles {
            let theta = ang_deg.to_radians();
            let sin_t = theta.sin();
            let cos_t = theta.cos();
            for &az_deg in &azimuths {
                let psi = az_deg.to_radians();
                let dir_plane = psi.cos() * u + psi.sin() * v;
                let normal = (cos_t * base_normal + sin_t * dir_plane).normalize_or_zero();
                if normal.length_squared() > 0.5 {
                    planes.push(Plane::new(pivot, normal));
                }
            }
        }

        planes
    }

    /// Generates candidate cutting planes based on:
    /// 1. Centroid planes and tilts around mesh center of mass and AABB center.
    /// 2. Overhang boundary planes (Z-levels and coordinate planes at overhang junctions).
    /// 3. Geometric bottleneck planes (narrow necks and minimal cross-sections).
    /// 4. PCA / Principal axes planes of overhang clusters.
    /// 5. Planes offset along major axes between bounding box extents.
    pub fn generate_candidates(mesh: &Mesh, config: &OptimizerConfig) -> Vec<Plane> {
        if mesh.is_empty() {
            return Vec::new();
        }

        let aabb = mesh.aabb();
        let com = mesh.center_of_mass();
        let aabb_center = aabb.center();
        let mut raw_planes: Vec<Plane> = Vec::new();

        // 0. Centroid planes and tilts around mesh center of mass and AABB center
        raw_planes.push(Plane::new(com, Vec3::Z));
        raw_planes.extend(Self::generate_tilted_planes(com, Vec3::Z));
        if (com - aabb_center).length() > 0.5 {
            raw_planes.push(Plane::new(aabb_center, Vec3::Z));
            raw_planes.extend(Self::generate_tilted_planes(aabb_center, Vec3::Z));
        }

        // 1. Overhang boundary planes
        let overhang_planes = Self::overhang_boundary_planes(mesh, config, &aabb);
        raw_planes.extend(overhang_planes);

        // 2. Geometric bottleneck planes
        let bottleneck_planes = Self::bottleneck_planes(mesh, &aabb);
        raw_planes.extend(bottleneck_planes);

        // 3. PCA principal axes planes
        let pca_planes = Self::pca_planes(mesh, config);
        raw_planes.extend(pca_planes);

        // 4. Planes offset along major axes
        let offset_planes = Self::bounding_box_offset_planes(&aabb, config.num_candidate_planes);
        raw_planes.extend(offset_planes);

        // Deduplicate and filter planes
        Self::deduplicate_planes(raw_planes, &aabb)
    }

    /// Heuristic 1: Overhang boundary planes where overhang triangles begin or meet vertical geometry.
    pub fn overhang_boundary_planes(
        mesh: &Mesh,
        config: &OptimizerConfig,
        aabb: &AABB,
    ) -> Vec<Plane> {
        let center = aabb.center();
        let com = mesh.center_of_mass();
        let z_bed = aabb.min.z;
        let mut planes = Vec::new();

        let overhang_tris: Vec<&Triangle> = mesh
            .triangles
            .iter()
            .filter(|t| is_overhang(t, &config.support_config, z_bed))
            .collect();

        if overhang_tris.is_empty() {
            return planes;
        }

        // Group Z heights of overhang triangles
        let mut z_levels: Vec<(f32, f32)> = Vec::new(); // (z, total_area)
        for tri in &overhang_tris {
            let tri_area = tri.area();
            for v in &tri.vertices {
                let z = v.z;
                let mut found = false;
                for entry in &mut z_levels {
                    if (entry.0 - z).abs() < 0.25 {
                        entry.1 += tri_area / 3.0;
                        found = true;
                        break;
                    }
                }
                if !found {
                    z_levels.push((z, tri_area / 3.0));
                }
            }
        }

        z_levels.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for (z, _) in z_levels.iter().take(4) {
            if *z > aabb.min.z + 0.5 && *z < aabb.max.z - 0.5 {
                let p = Vec3::new(center.x, center.y, *z);
                planes.push(Plane::new(p, Vec3::Z));
                planes.extend(Self::generate_tilted_planes(p, Vec3::Z));
                let p_com = Vec3::new(com.x, com.y, *z);
                if (p - p_com).length() > 0.5 {
                    planes.push(Plane::new(p_com, Vec3::Z));
                    planes.extend(Self::generate_tilted_planes(p_com, Vec3::Z));
                }
            }
        }

        // Distinct X and Y boundaries of overhangs
        let mut x_levels: Vec<(f32, f32)> = Vec::new();
        let mut y_levels: Vec<(f32, f32)> = Vec::new();

        for tri in &overhang_tris {
            let tri_area = tri.area();
            for v in &tri.vertices {
                let mut found_x = false;
                for entry in &mut x_levels {
                    if (entry.0 - v.x).abs() < 0.5 {
                        entry.1 += tri_area / 3.0;
                        found_x = true;
                        break;
                    }
                }
                if !found_x {
                    x_levels.push((v.x, tri_area / 3.0));
                }

                let mut found_y = false;
                for entry in &mut y_levels {
                    if (entry.0 - v.y).abs() < 0.5 {
                        entry.1 += tri_area / 3.0;
                        found_y = true;
                        break;
                    }
                }
                if !found_y {
                    y_levels.push((v.y, tri_area / 3.0));
                }
            }
        }

        x_levels.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for (x, _) in x_levels.iter().take(4) {
            if *x > aabb.min.x + 0.5 && *x < aabb.max.x - 0.5 {
                planes.push(Plane::new(Vec3::new(*x, center.y, center.z), Vec3::X));
            }
        }

        y_levels.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        for (y, _) in y_levels.iter().take(4) {
            if *y > aabb.min.y + 0.5 && *y < aabb.max.y - 0.5 {
                planes.push(Plane::new(Vec3::new(center.x, *y, center.z), Vec3::Y));
            }
        }

        planes
    }

    /// Heuristic 2: Geometric bottleneck planes swept along X, Y, Z axes.
    pub fn bottleneck_planes(mesh: &Mesh, aabb: &AABB) -> Vec<Plane> {
        let center = aabb.center();
        let com = mesh.center_of_mass();
        let mut planes = Vec::new();
        let samples = 25;

        // Sweep along Z
        let span_z = aabb.max.z - aabb.min.z;
        if span_z > 2.0 {
            let mut areas_z: Vec<(f32, f32)> = Vec::with_capacity(samples);
            for i in 1..=samples {
                let t = i as f32 / (samples + 1) as f32;
                let z = aabb.min.z + t * span_z;
                let area = compute_cross_section_area_z(mesh, z);
                areas_z.push((z, area));
            }

            // Detect local minima and step transitions
            for i in 1..(areas_z.len() - 1) {
                let prev = areas_z[i - 1].1;
                let curr = areas_z[i].1;
                let next = areas_z[i + 1].1;

                let is_local_min = curr < prev && curr < next;
                let is_step = (curr - prev).abs() > 0.25 * curr.max(prev)
                    || (next - curr).abs() > 0.25 * curr.max(next);

                if is_local_min || is_step {
                    let z_val = areas_z[i].0;
                    let p_center = Vec3::new(center.x, center.y, z_val);
                    planes.push(Plane::new(p_center, Vec3::Z));
                    planes.extend(Self::generate_tilted_planes(p_center, Vec3::Z));
                    let p_com = Vec3::new(com.x, com.y, z_val);
                    if (p_center - p_com).length() > 0.5 {
                        planes.push(Plane::new(p_com, Vec3::Z));
                        planes.extend(Self::generate_tilted_planes(p_com, Vec3::Z));
                    }
                }
            }
        }

        // Sweep along X
        let span_x = aabb.max.x - aabb.min.x;
        if span_x > 2.0 {
            let mut areas_x: Vec<(f32, f32)> = Vec::with_capacity(samples);
            for i in 1..=samples {
                let t = i as f32 / (samples + 1) as f32;
                let x = aabb.min.x + t * span_x;
                let area = compute_cross_section_area_x(mesh, x);
                areas_x.push((x, area));
            }

            for i in 1..(areas_x.len() - 1) {
                let prev = areas_x[i - 1].1;
                let curr = areas_x[i].1;
                let next = areas_x[i + 1].1;

                if (curr < prev && curr < next)
                    || (curr - prev).abs() > 0.25 * curr.max(prev)
                    || (next - curr).abs() > 0.25 * curr.max(next)
                {
                    let p_center = Vec3::new(areas_x[i].0, center.y, center.z);
                    planes.push(Plane::new(p_center, Vec3::X));
                    planes.extend(Self::generate_tilted_planes(p_center, Vec3::X));
                }
            }
        }

        // Sweep along Y
        let span_y = aabb.max.y - aabb.min.y;
        if span_y > 2.0 {
            let mut areas_y: Vec<(f32, f32)> = Vec::with_capacity(samples);
            for i in 1..=samples {
                let t = i as f32 / (samples + 1) as f32;
                let y = aabb.min.y + t * span_y;
                let area = compute_cross_section_area_y(mesh, y);
                areas_y.push((y, area));
            }

            for i in 1..(areas_y.len() - 1) {
                let prev = areas_y[i - 1].1;
                let curr = areas_y[i].1;
                let next = areas_y[i + 1].1;

                if (curr < prev && curr < next)
                    || (curr - prev).abs() > 0.25 * curr.max(prev)
                    || (next - curr).abs() > 0.25 * curr.max(next)
                {
                    let p_center = Vec3::new(center.x, areas_y[i].0, center.z);
                    planes.push(Plane::new(p_center, Vec3::Y));
                    planes.extend(Self::generate_tilted_planes(p_center, Vec3::Y));
                }
            }
        }

        planes
    }

    /// Heuristic 3: PCA principal axes planes of overhang clusters.
    #[allow(clippy::needless_range_loop)]
    pub fn pca_planes(mesh: &Mesh, config: &OptimizerConfig) -> Vec<Plane> {
        let aabb = mesh.aabb();
        let z_bed = aabb.min.z;
        let mut planes = Vec::new();

        let overhang_tris: Vec<&Triangle> = mesh
            .triangles
            .iter()
            .filter(|t| is_overhang(t, &config.support_config, z_bed))
            .collect();

        if overhang_tris.is_empty() {
            return planes;
        }

        let mut total_area = 0.0f32;
        let mut centroid = Vec3::ZERO;

        for tri in &overhang_tris {
            let a = tri.area();
            let c = (tri.vertices[0] + tri.vertices[1] + tri.vertices[2]) / 3.0;
            centroid += c * a;
            total_area += a;
        }

        if total_area <= 1e-6 {
            return planes;
        }
        centroid /= total_area;

        // Covariance matrix
        let mut cov = [[0.0f32; 3]; 3];
        for tri in &overhang_tris {
            let a = tri.area();
            let c = (tri.vertices[0] + tri.vertices[1] + tri.vertices[2]) / 3.0 - centroid;

            cov[0][0] += a * c.x * c.x;
            cov[0][1] += a * c.x * c.y;
            cov[0][2] += a * c.x * c.z;
            cov[1][0] += a * c.y * c.x;
            cov[1][1] += a * c.y * c.y;
            cov[1][2] += a * c.y * c.z;
            cov[2][0] += a * c.z * c.x;
            cov[2][1] += a * c.z * c.y;
            cov[2][2] += a * c.z * c.z;
        }

        for row in &mut cov {
            for val in row.iter_mut() {
                *val /= total_area;
            }
        }

        let (eigenvectors, _) = jacobi_eigen_3x3(cov);

        // Tilted planes around overhang centroid
        planes.push(Plane::new(centroid, Vec3::Z));
        planes.extend(Self::generate_tilted_planes(centroid, Vec3::Z));

        for col in 0..3 {
            let axis = Vec3::new(
                eigenvectors[0][col],
                eigenvectors[1][col],
                eigenvectors[2][col],
            )
            .normalize_or_zero();

            if axis.length_squared() > 0.5 {
                planes.push(Plane::new(centroid, axis));
                planes.extend(Self::generate_tilted_planes(centroid, axis));

                // Also generate plane at median projection
                let mut projs: Vec<f32> = overhang_tris
                    .iter()
                    .map(|t| {
                        let c = (t.vertices[0] + t.vertices[1] + t.vertices[2]) / 3.0;
                        c.dot(axis)
                    })
                    .collect();
                projs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                if !projs.is_empty() {
                    let median = projs[projs.len() / 2];
                    let p_med = axis * median;
                    planes.push(Plane::new(p_med, axis));
                    planes.extend(Self::generate_tilted_planes(p_med, axis));
                }
            }
        }

        planes
    }

    /// Heuristic 4: Planes offset along major axes between bounding box extents.
    pub fn bounding_box_offset_planes(aabb: &AABB, num_planes: usize) -> Vec<Plane> {
        let center = aabb.center();
        let size = aabb.size();
        let mut planes = Vec::new();

        let steps = (num_planes / 3).max(4);
        for i in 1..=steps {
            let frac = 0.1 + 0.8 * (i as f32 / (steps + 1) as f32);

            let px = aabb.min.x + frac * size.x;
            planes.push(Plane::new(Vec3::new(px, center.y, center.z), Vec3::X));

            let py = aabb.min.y + frac * size.y;
            planes.push(Plane::new(Vec3::new(center.x, py, center.z), Vec3::Y));

            let pz = aabb.min.z + frac * size.z;
            planes.push(Plane::new(Vec3::new(center.x, center.y, pz), Vec3::Z));
        }

        // Canonical diagonals
        let diag1 = Vec3::new(1.0, 1.0, 0.0).normalize();
        let diag2 = Vec3::new(1.0, -1.0, 0.0).normalize();
        let diag3 = Vec3::new(1.0, 0.0, 1.0).normalize();
        let diag4 = Vec3::new(0.0, 1.0, 1.0).normalize();

        planes.push(Plane::new(center, diag1));
        planes.push(Plane::new(center, diag2));
        planes.push(Plane::new(center, diag3));
        planes.push(Plane::new(center, diag4));

        planes
    }

    /// Filters planes to ensure intersection with AABB and removes duplicates.
    fn deduplicate_planes(raw_planes: Vec<Plane>, aabb: &AABB) -> Vec<Plane> {
        let center = aabb.center();
        let half_diag = aabb.size().length() * 0.5;
        let mut unique: Vec<Plane> = Vec::new();

        for p in raw_planes {
            let n = p.normal.normalize_or_zero();
            if n.length_squared() < 0.5 {
                continue;
            }

            // Ensure plane intersects or is very close to AABB
            let dist_to_center = (center - p.point).dot(n).abs();
            if dist_to_center > half_diag * 0.95 {
                continue;
            }

            // Canonicalize normal direction
            let (canon_n, canon_d) = if n.x < -1e-4
                || (n.x.abs() <= 1e-4 && n.y < -1e-4)
                || (n.x.abs() <= 1e-4 && n.y.abs() <= 1e-4 && n.z < -1e-4)
            {
                (-n, -p.point.dot(n))
            } else {
                (n, p.point.dot(n))
            };

            let is_dup = unique.iter().any(|existing| {
                let dot = existing.normal.dot(canon_n);
                if dot > 0.99 {
                    let d_exist = existing.point.dot(existing.normal);
                    (d_exist - canon_d).abs() < 0.4
                } else {
                    false
                }
            });

            if !is_dup {
                unique.push(Plane::new(canon_n * canon_d, canon_n));
            }
        }

        unique
    }
}

/// Intermediate result of candidate plane evaluation.
struct CandidateEvaluation {
    plane: Plane,
    score: f32,
}

/// Evaluates a single candidate plane on a mesh.
fn evaluate_candidate_plane(
    mesh: &Mesh,
    plane: &Plane,
    config: &OptimizerConfig,
    orig_vol: f32,
) -> Option<CandidateEvaluation> {
    let cut_result = slice_mesh_by_plane(mesh, plane).ok()?;
    let pos_vol = cut_result.positive_mesh.total_volume();
    let neg_vol = cut_result.negative_mesh.total_volume();
    let total_vol = pos_vol + neg_vol;

    if total_vol <= 1e-4 {
        return None;
    }

    let min_ratio = config.min_piece_volume_ratio;
    if (pos_vol / orig_vol) < min_ratio || (neg_vol / orig_vol) < min_ratio {
        return None;
    }

    let mut contiguity_penalty = 0.0f32;
    if config.enforce_contiguous {
        let is_fragmented = |m: &Mesh| -> bool {
            let comps = m.connected_components();
            if comps.len() <= 1 {
                return false;
            }
            comps.iter().filter(|c| c.total_volume() > 1e-4).count() > 1
        };

        if is_fragmented(&cut_result.positive_mesh) || is_fragmented(&cut_result.negative_mesh) {
            contiguity_penalty = 1e6;
        }
    }

    let pos_opt = find_optimal_orientation(&cut_result.positive_mesh, &config.support_config);
    let neg_opt = find_optimal_orientation(&cut_result.negative_mesh, &config.support_config);

    let support_vol =
        pos_opt.metrics.estimated_support_volume + neg_opt.metrics.estimated_support_volume;
    let score = config.weight_support * support_vol
        + config.weight_cut_area * cut_result.cut_area
        + contiguity_penalty;

    Some(CandidateEvaluation {
        plane: *plane,
        score,
    })
}

/// Enforces single contiguous solid guarantee by processing connected components.
/// Promotes secondary components if pieces budget allows, and preserves all geometry
/// by keeping every triangle without deleting anything.
fn extract_contiguous_components(mesh: Mesh, max_additional: usize) -> (Mesh, Vec<Mesh>) {
    let comps = mesh.connected_components();
    if comps.is_empty() {
        return (mesh, Vec::new());
    }
    if comps.len() == 1 {
        return (comps.into_iter().next().unwrap(), Vec::new());
    }

    let mut primary = comps[0].clone();
    let mut promoted = Vec::new();
    let mut remaining_budget = max_additional;

    for c in comps.into_iter().skip(1) {
        if remaining_budget > 0 {
            promoted.push(c);
            remaining_budget -= 1;
        } else {
            // NEVER discard geometry: keep all triangles in primary mesh!
            primary.triangles.extend(c.triangles);
        }
    }

    (primary, promoted)
}

/// Evaluates candidate cut planes in parallel and finds the optimal partition of the mesh.
pub fn optimize_partitions(mesh: &Mesh, config: &OptimizerConfig) -> OptimizationResult {
    let start_time = std::time::Instant::now();

    if mesh.is_empty() {
        return OptimizationResult {
            original_metrics: SupportMetrics::default(),
            pieces: Vec::new(),
            pins: Vec::new(),
            cuts_made: 0,
            total_support_volume: 0.0,
            total_reduction_pct: 0.0,
            computation_time_ms: 0.0,
            pin_size_used: None,
        };
    }

    let orig_metrics = analyze_support(mesh, &config.support_config);

    // If already requires 0 support or max_pieces <= 1, do not cut
    if orig_metrics.estimated_support_volume <= 1e-4 || config.max_pieces <= 1 {
        let (primary, promoted) =
            extract_contiguous_components(mesh.clone(), config.max_pieces.saturating_sub(1));
        let mut pieces_to_orient = vec![primary];
        pieces_to_orient.extend(promoted);

        let mut pieces = Vec::with_capacity(pieces_to_orient.len());
        let mut final_support = 0.0f32;

        for (i, m) in pieces_to_orient.into_iter().enumerate() {
            let optimal = find_optimal_orientation(&m, &config.support_config);
            final_support += optimal.metrics.estimated_support_volume;
            pieces.push(PartitionPiece {
                id: i + 1,
                mesh: optimal.mesh,
                metrics: optimal.metrics,
                contact_area: optimal.contact_area,
            });
        }

        let orig_sup = orig_metrics.estimated_support_volume;
        let reduction = if orig_sup > 1e-6 {
            ((orig_sup - final_support) / orig_sup * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };

        return OptimizationResult {
            original_metrics: orig_metrics,
            pieces,
            pins: Vec::new(),
            cuts_made: 0,
            total_support_volume: final_support,
            total_reduction_pct: reduction,
            computation_time_ms: start_time.elapsed().as_secs_f64() * 1000.0,
            pin_size_used: None,
        };
    }

    struct PieceCandidate {
        mesh: Mesh,
        opt: OptimalOrientation,
        current_support: f32,
        can_cut: bool,
    }

    let (initial_clean, _) = extract_contiguous_components(mesh.clone(), 0);
    let mut pieces_state = vec![PieceCandidate {
        mesh: initial_clean.clone(),
        opt: OptimalOrientation {
            rotation: glam::Quat::IDENTITY,
            translation: Vec3::ZERO,
            mesh: initial_clean,
            metrics: orig_metrics.clone(),
            contact_area: 0.0,
            is_stable: true,
        },
        current_support: orig_metrics.estimated_support_volume,
        can_cut: true,
    }];

    let mut all_pins: Vec<Mesh> = Vec::new();
    let mut cuts_made = 0;

    while pieces_state.len() < config.max_pieces {
        let mut best_idx = None;
        let mut max_sup = 1e-3f32;

        for (i, p) in pieces_state.iter().enumerate() {
            if p.can_cut && p.current_support > max_sup {
                max_sup = p.current_support;
                best_idx = Some(i);
            }
        }

        let piece_idx = match best_idx {
            Some(idx) => idx,
            None => break,
        };

        let target_mesh = pieces_state[piece_idx].mesh.clone();
        let target_vol = target_mesh.total_volume();
        let uncut_score = config.weight_support * pieces_state[piece_idx].current_support;

        let candidates = CandidatePlaneGenerator::generate_candidates(&target_mesh, config);

        let evaluated: Vec<CandidateEvaluation> = candidates
            .par_iter()
            .filter_map(|plane| evaluate_candidate_plane(&target_mesh, plane, config, target_vol))
            .collect();

        let best_candidate = evaluated
            .into_iter()
            .min_by(|a, b| a.score.partial_cmp(&b.score).unwrap_or(std::cmp::Ordering::Equal));

        match best_candidate {
            Some(best) if best.score < uncut_score - 1e-4 => {
                cuts_made += 1;
                let (raw_pos, raw_neg) = if let Some(ref pin_cfg) = config.pin_config {
                    if let Ok(cut_res) = slice_mesh_by_plane(&target_mesh, &best.plane) {
                        let mut adaptive_pin_cfg = *pin_cfg;
                        adaptive_pin_cfg.adaptive = true;
                        adaptive_pin_cfg.min_size = config.pin_min_size;
                        let (pos_s, neg_s, mut p_meshes) =
                            apply_female_sockets_and_generate_pins(&cut_res, &adaptive_pin_cfg);
                        all_pins.append(&mut p_meshes);
                        (pos_s, neg_s)
                    } else {
                        pieces_state[piece_idx].can_cut = false;
                        continue;
                    }
                } else if let Ok(cut_res) = slice_mesh_by_plane(&target_mesh, &best.plane) {
                    (cut_res.positive_mesh, cut_res.negative_mesh)
                } else {
                    pieces_state[piece_idx].can_cut = false;
                    continue;
                };

                pieces_state.remove(piece_idx);

                let current_len = pieces_state.len();
                let budget = config.max_pieces.saturating_sub(current_len + 2);
                let (pos_primary, pos_promoted) = extract_contiguous_components(raw_pos, budget);
                let budget = config
                    .max_pieces
                    .saturating_sub(current_len + 2 + pos_promoted.len());
                let (neg_primary, neg_promoted) = extract_contiguous_components(raw_neg, budget);

                let mut to_add = Vec::new();
                to_add.push(pos_primary);
                to_add.extend(pos_promoted);
                to_add.push(neg_primary);
                to_add.extend(neg_promoted);

                for m in to_add {
                    let opt = find_optimal_orientation(&m, &config.support_config);
                    let sup = opt.metrics.estimated_support_volume;
                    pieces_state.push(PieceCandidate {
                        mesh: m,
                        opt,
                        current_support: sup,
                        can_cut: sup > 1e-3,
                    });
                }
            }
            _ => {
                pieces_state[piece_idx].can_cut = false;
            }
        }
    }

    if cuts_made == 0 {
        let (primary, promoted) =
            extract_contiguous_components(mesh.clone(), config.max_pieces.saturating_sub(1));
        let mut pieces_to_orient = vec![primary];
        pieces_to_orient.extend(promoted);

        let mut pieces = Vec::with_capacity(pieces_to_orient.len());
        let mut final_support = 0.0f32;

        for (i, m) in pieces_to_orient.into_iter().enumerate() {
            let optimal = find_optimal_orientation(&m, &config.support_config);
            final_support += optimal.metrics.estimated_support_volume;
            pieces.push(PartitionPiece {
                id: i + 1,
                mesh: optimal.mesh,
                metrics: optimal.metrics,
                contact_area: optimal.contact_area,
            });
        }

        let orig_sup = orig_metrics.estimated_support_volume;
        let reduction = if orig_sup > 1e-6 {
            ((orig_sup - final_support) / orig_sup * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };

        OptimizationResult {
            original_metrics: orig_metrics,
            pieces,
            pins: Vec::new(),
            cuts_made: 0,
            total_support_volume: final_support,
            total_reduction_pct: reduction,
            computation_time_ms: start_time.elapsed().as_secs_f64() * 1000.0,
            pin_size_used: None,
        }
    } else {
        let mut final_pieces = Vec::with_capacity(pieces_state.len());
        let mut total_support_volume = 0.0f32;

        for (i, p) in pieces_state.into_iter().enumerate() {
            total_support_volume += p.opt.metrics.estimated_support_volume;
            final_pieces.push(PartitionPiece {
                id: i + 1,
                mesh: p.opt.mesh,
                metrics: p.opt.metrics,
                contact_area: p.opt.contact_area,
            });
        }

        let orig_sup = orig_metrics.estimated_support_volume;
        let reduction = if orig_sup > 1e-6 {
            ((orig_sup - total_support_volume) / orig_sup * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };

        let pin_size_used = all_pins.first().map(|p| p.aabb().size().x);

        OptimizationResult {
            original_metrics: orig_metrics,
            pieces: final_pieces,
            pins: all_pins,
            cuts_made,
            total_support_volume,
            total_reduction_pct: reduction,
            computation_time_ms: start_time.elapsed().as_secs_f64() * 1000.0,
            pin_size_used,
        }
    }
}
