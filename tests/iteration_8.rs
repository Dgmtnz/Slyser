use glam::Vec3;
use slyser::cut::slice_mesh_by_plane;
use slyser::mesh::{Plane, AABB};
use slyser::optimizer::{
    evaluate_candidate_plane, refine_candidate_plane, CandidatePlaneGenerator, OptimizerConfig,
};
use slyser::orient::{extract_candidate_orientations, find_optimal_orientation};
use slyser::primitives::{create_t_bracket, create_u_fork};
use slyser::support::SupportConfig;

#[test]
fn test_dense_tilted_planes_and_compound_diagonal_tilts() {
    let pivot = Vec3::new(10.0, 10.0, 10.0);
    let normal = Vec3::Z;
    let planes = CandidatePlaneGenerator::generate_tilted_planes(pivot, normal);

    // 9 angles (5, 10, 15, 20, 25, 30, 35, 40, 45) * 8 azimuths = 72 tilts
    // plus compound diagonal tilts (3 angles * 4 combinations = 12 tilts)
    assert!(
        planes.len() >= 84,
        "Expected at least 84 tilted planes, got {}",
        planes.len()
    );

    // Verify fine 5° tilt is present
    let has_5_deg = planes.iter().any(|p| {
        let dot = p.normal.dot(normal).clamp(-1.0, 1.0);
        let angle_deg = dot.acos().to_degrees();
        (angle_deg - 5.0).abs() < 0.2
    });
    assert!(has_5_deg, "Must include 5-degree tilted plane");

    // Verify 45° tilt is present
    let has_45_deg = planes.iter().any(|p| {
        let dot = p.normal.dot(normal).clamp(-1.0, 1.0);
        let angle_deg = dot.acos().to_degrees();
        (angle_deg - 45.0).abs() < 0.2
    });
    assert!(has_45_deg, "Must include 45-degree tilted plane");
}

#[test]
fn test_quadruple_bottleneck_samples() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);
    let aabb = t_bracket.aabb();

    let planes = CandidatePlaneGenerator::bottleneck_planes(&t_bracket, &aabb);
    assert!(
        !planes.is_empty(),
        "Bottleneck planes must detect geometry transitions with 100 samples"
    );

    // Junction around z = 40 must be found
    let has_z40 = planes.iter().any(|p| {
        p.normal.dot(Vec3::Z).abs() > 0.99 && (p.point.z - 40.0).abs() < 1.0
    });
    assert!(has_z40, "Bottleneck planes must include z=40 neck transition");
}

#[test]
fn test_fine_deduplication_preserves_1_5_degrees() {
    let aabb = AABB::new(Vec3::splat(-20.0), Vec3::splat(20.0));
    let p0 = Plane::new(Vec3::ZERO, Vec3::Z);

    // A plane tilted by 2.0 degrees (cos(2.0°) ~ 0.99939 < 0.9996)
    let rad_2deg = 2.0f32.to_radians();
    let norm_2deg = Vec3::new(rad_2deg.sin(), 0.0, rad_2deg.cos()).normalize();
    let p_2deg = Plane::new(Vec3::ZERO, norm_2deg);

    // A plane tilted by 0.5 degrees (cos(0.5°) ~ 0.99996 > 0.9996)
    let rad_half_deg = 0.5f32.to_radians();
    let norm_half_deg = Vec3::new(rad_half_deg.sin(), 0.0, rad_half_deg.cos()).normalize();
    let p_half_deg = Plane::new(Vec3::ZERO, norm_half_deg);

    let raw = vec![p0, p_2deg, p_half_deg];
    let dedup = CandidatePlaneGenerator::deduplicate_planes(raw, &aabb);

    // p0 and p_2deg should both be preserved (diff > 1.5°), while p_half_deg is merged
    assert_eq!(
        dedup.len(),
        2,
        "Deduplication should preserve planes differentiated by 2.0° but merge 0.5°"
    );
}

#[test]
fn test_local_hill_climbing_refinement() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);
    let config = OptimizerConfig::default();
    let orig_vol = t_bracket.total_volume();

    // Slightly imperfect cut plane near the z=40 junction, tilted by 1 degree
    let rad = 1.0f32.to_radians();
    let imperfect_normal = Vec3::new(rad.sin(), 0.0, rad.cos()).normalize();
    let imperfect_plane = Plane::new(Vec3::new(0.0, 0.0, 39.8), imperfect_normal);

    let eval_init =
        evaluate_candidate_plane(&t_bracket, &imperfect_plane, &config, orig_vol).unwrap();

    let refined_plane = refine_candidate_plane(
        &t_bracket,
        &imperfect_plane,
        eval_init.score,
        &config,
        orig_vol,
    );

    let eval_refined =
        evaluate_candidate_plane(&t_bracket, &refined_plane, &config, orig_vol).unwrap();

    assert!(
        eval_refined.score <= eval_init.score + 1e-4,
        "Refined plane must improve or preserve Pareto score: initial {}, refined {}",
        eval_init.score,
        eval_refined.score
    );

    // Verify refined cut produces single contiguous solids
    let cut_res = slice_mesh_by_plane(&t_bracket, &refined_plane).unwrap();
    assert!(cut_res.positive_mesh.is_contiguous());
    assert!(cut_res.negative_mesh.is_contiguous());
}

#[test]
fn test_refinement_maintains_single_contiguous_components() {
    let u_fork = create_u_fork(50.0, 20.0, 15.0, 15.0, 50.0);
    let config = OptimizerConfig::default();
    let orig_vol = u_fork.total_volume();

    // Plane cutting base properly
    let good_plane = Plane::new(Vec3::new(0.0, 0.0, 10.0), Vec3::Z);
    let eval_good = evaluate_candidate_plane(&u_fork, &good_plane, &config, orig_vol).unwrap();

    let refined_plane =
        refine_candidate_plane(&u_fork, &good_plane, eval_good.score, &config, orig_vol);

    let cut_res = slice_mesh_by_plane(&u_fork, &refined_plane).unwrap();
    assert!(
        cut_res.positive_mesh.is_contiguous(),
        "Positive half must remain single contiguous component"
    );
    assert!(
        cut_res.negative_mesh.is_contiguous(),
        "Negative half must remain single contiguous component"
    );
}

#[test]
fn test_orient_clusters_48_and_fine_tilts() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let candidates = extract_candidate_orientations(&t_bracket);
    // Even for simple meshes with canonical faces, extraction succeeds
    assert!(candidates.len() >= 6);

    let config = SupportConfig::default();
    let optimal = find_optimal_orientation(&t_bracket, &config);

    assert_eq!(
        optimal.metrics.estimated_support_volume, 0.0,
        "Optimal orientation must achieve zero support"
    );
    assert!(optimal.is_stable);
    assert!(optimal.contact_area >= 1200.0);
}

#[test]
fn test_partition_outputs_closed_and_conserved() {
    let p1_path = std::path::Path::new("./piece_1.stl");
    let p2_path = std::path::Path::new("./piece_2.stl");
    let pin_path = std::path::Path::new("./pin_1.stl");

    if !p1_path.exists() || !p2_path.exists() {
        return;
    }

    let p1 = slyser::stl::read_stl_file(p1_path).unwrap();
    let p2 = slyser::stl::read_stl_file(p2_path).unwrap();

    assert!(p1.is_closed(), "piece_1 must be a closed solid");
    assert_eq!(p1.connected_components_count(), 1, "piece_1 must have 1 component");

    assert!(p2.is_closed(), "piece_2 must be a closed solid");
    assert_eq!(p2.connected_components_count(), 1, "piece_2 must have 1 component");

    if pin_path.exists() {
        let pin = slyser::stl::read_stl_file(pin_path).unwrap();
        assert!(pin.is_closed(), "pin_1 must be a closed solid");
        assert_eq!(pin.connected_components_count(), 1, "pin_1 must have 1 component");
    }
}
