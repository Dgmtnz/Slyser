use glam::Vec3;
use slyser::mesh::Plane;
use slyser::optimizer::{optimize_partitions, CandidatePlaneGenerator, OptimizerConfig};
use slyser::pins::PinConfig;
use slyser::primitives::{create_cube, create_t_bracket};

#[test]
fn test_t_bracket_partition_optimal_reduction() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let config = OptimizerConfig::default();
    let result = optimize_partitions(&t_bracket, &config);

    // Verify original support volume is around 32,000 mm³
    assert!(
        result.original_metrics.estimated_support_volume > 30000.0,
        "Original T-bracket must require ~32000 mm³ support, got {}",
        result.original_metrics.estimated_support_volume
    );

    // Verify optimizer made exactly 1 cut to partition into stem and overhang bar
    assert_eq!(
        result.cuts_made, 1,
        "Optimizer should perform 1 optimal cut on T-bracket"
    );
    assert_eq!(
        result.pieces.len(),
        2,
        "Optimizer should produce exactly 2 pieces"
    );

    // Verify support volume reduction is >= 90% (specifically 100%)
    assert!(
        result.total_reduction_pct >= 90.0,
        "Expected support reduction >= 90%, got {:.1}%",
        result.total_reduction_pct
    );
    assert_eq!(
        result.total_support_volume, 0.0,
        "Both reoriented pieces should require 0.0 mm³ support volume"
    );

    // Verify both pieces are placed flat on the print bed (z = 0)
    for piece in &result.pieces {
        assert_eq!(piece.metrics.estimated_support_volume, 0.0);
        assert!(
            (piece.mesh.aabb().min.z - 0.0).abs() < 1e-4,
            "Piece {} must be flat on build plate at z=0",
            piece.id
        );
        assert!(
            piece.mesh.is_closed() && piece.mesh.is_watertight(),
            "Piece {} must be closed and watertight",
            piece.id
        );
    }

    // Verify volume conservation
    let total_piece_vol: f32 = result.pieces.iter().map(|p| p.mesh.total_volume()).sum();
    let orig_vol = t_bracket.total_volume();
    assert!(
        (total_piece_vol - orig_vol).abs() / orig_vol < 0.01,
        "Volume must be conserved within 1%: orig {:.1}, pieces {:.1}",
        orig_vol,
        total_piece_vol
    );
}

#[test]
fn test_t_bracket_partition_with_pins() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let pin_config = PinConfig::default();
    let config = OptimizerConfig::default().with_pins(pin_config);
    let result = optimize_partitions(&t_bracket, &config);

    assert_eq!(result.cuts_made, 1);
    assert_eq!(result.pieces.len(), 2);
    assert!(
        !result.pins.is_empty(),
        "Square pins must be generated when pin_config is enabled"
    );

    for pin in &result.pins {
        assert!(
            pin.is_closed() && pin.is_watertight(),
            "Pin must be closed and watertight"
        );
        assert!(
            (pin.aabb().min.z - 0.0).abs() < 1e-4,
            "Pin must be flat on the bed at z=0"
        );
    }

    for piece in &result.pieces {
        assert!(
            piece.mesh.is_closed() && piece.mesh.is_watertight(),
            "Piece with female sockets must remain watertight"
        );
        assert!(
            (piece.mesh.aabb().min.z - 0.0).abs() < 1e-4,
            "Piece must rest flat on the bed at z=0"
        );
    }
}

#[test]
fn test_cube_pareto_tradeoff_no_cut_zero_support() {
    // A flat cube already has 0 support volume.
    // Any cut would incur cut penalties (cut_area * weight_cut_area > 0).
    // Therefore, the Pareto optimizer must decide NOT to cut the cube!
    let cube = create_cube(Vec3::splat(20.0));
    let config = OptimizerConfig::default();
    let result = optimize_partitions(&cube, &config);

    assert_eq!(
        result.cuts_made, 0,
        "Cube with 0 support should NOT be cut"
    );
    assert_eq!(
        result.pieces.len(),
        1,
        "Result should contain only the original uncut cube"
    );
    assert_eq!(result.total_support_volume, 0.0);
    assert!(result.pins.is_empty());
}

#[test]
fn test_candidate_plane_generator_heuristics() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let config = OptimizerConfig::default();
    let candidates = CandidatePlaneGenerator::generate_candidates(&t_bracket, &config);

    assert!(
        !candidates.is_empty(),
        "Candidate plane generator must return candidate planes"
    );

    // Check that the critical overhang junction plane at z = 40 with normal along Z was generated
    let has_z40_plane = candidates.iter().any(|plane| {
        let is_z_norm = plane.normal.dot(Vec3::Z).abs() > 0.99;
        let d = plane.point.dot(plane.normal);
        is_z_norm && (d.abs() - 40.0).abs() < 1.0
    });

    assert!(
        has_z40_plane,
        "Candidate planes must include the critical overhang junction plane at z=40"
    );
}

#[test]
fn test_max_pieces_1_does_not_cut() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let config = OptimizerConfig::default().with_max_pieces(1);
    let result = optimize_partitions(&t_bracket, &config);

    assert_eq!(result.cuts_made, 0);
    assert_eq!(result.pieces.len(), 1);
}

#[test]
fn test_min_piece_volume_ratio_rejection() {
    let cube = create_cube(Vec3::splat(20.0));
    // Plane slicing off a 0.1 mm sliver (0.5% volume)
    let sliver_plane = Plane::new(Vec3::new(0.0, 0.0, 0.1), Vec3::Z);

    let config = OptimizerConfig {
        min_piece_volume_ratio: 0.05, // 5% minimum
        ..Default::default()
    };

    let cut_res = slyser::cut::slice_mesh_by_plane(&cube, &sliver_plane).unwrap();
    let total_vol = cube.total_volume();
    let neg_vol = cut_res.negative_mesh.total_volume();
    let ratio = neg_vol / total_vol;

    assert!(
        ratio < config.min_piece_volume_ratio,
        "Sliver volume ratio {:.4} should be below threshold {:.4}",
        ratio,
        config.min_piece_volume_ratio
    );
}
