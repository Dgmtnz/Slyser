use glam::Vec3;
use slyser::cut::slice_mesh_by_plane;
use slyser::mesh::{Mesh, Plane};
use slyser::optimizer::{optimize_partitions, OptimizerConfig};
use slyser::pins::{apply_female_sockets_and_generate_pins, find_pin_placements, PinConfig};
use slyser::primitives::{create_box, create_cube, create_t_bracket, create_u_fork};

#[test]
fn test_connected_components_two_separate_boxes() {
    let box1 = create_box(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0));
    let box2 = create_box(Vec3::new(30.0, 30.0, 30.0), Vec3::new(40.0, 40.0, 40.0));

    let mut combined_tris = box1.triangles;
    combined_tris.extend(box2.triangles);
    let combined_mesh = Mesh::new(combined_tris);

    assert_eq!(
        combined_mesh.connected_components_count(),
        2,
        "Two disconnected boxes must have exactly 2 components"
    );
    assert!(
        !combined_mesh.is_contiguous(),
        "Two disconnected boxes must not be contiguous"
    );

    let components = combined_mesh.connected_components();
    assert_eq!(components.len(), 2);

    for comp in &components {
        assert_eq!(comp.triangle_count(), 12);
        assert!((comp.total_volume() - 1000.0).abs() < 1e-2);
        assert!(comp.is_closed() && comp.is_watertight());
    }

    let largest = combined_mesh.largest_connected_component();
    assert_eq!(largest.triangle_count(), 12);
    assert!((largest.total_volume() - 1000.0).abs() < 1e-2);
}

#[test]
fn test_connected_components_single_solid_cube_and_t_bracket() {
    let cube = create_cube(Vec3::splat(20.0));
    assert_eq!(cube.connected_components_count(), 1);
    assert!(cube.is_contiguous());
    let cube_comps = cube.connected_components();
    assert_eq!(cube_comps.len(), 1);
    assert!((cube_comps[0].total_volume() - 8000.0).abs() < 1e-2);

    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);
    assert_eq!(t_bracket.connected_components_count(), 1);
    assert!(t_bracket.is_contiguous());
    let t_comps = t_bracket.connected_components();
    assert_eq!(t_comps.len(), 1);
    assert!(t_comps[0].is_closed() && t_comps[0].is_watertight());
}

#[test]
fn test_contiguity_enforcement_u_fork_model() {
    // U-fork with base 50x20x15 and two vertical prongs of width 15, height 50
    let u_fork = create_u_fork(50.0, 20.0, 15.0, 15.0, 50.0);
    assert!(u_fork.is_closed() && u_fork.is_watertight());
    assert!(u_fork.is_contiguous());
    assert_eq!(u_fork.connected_components_count(), 1);

    // Cutting horizontally across both prongs at Z = 30
    let plane_across_prongs = Plane::new(Vec3::new(0.0, 0.0, 30.0), Vec3::Z);
    let cut = slice_mesh_by_plane(&u_fork, &plane_across_prongs)
        .expect("Planar cut across prongs must succeed");

    // Positive mesh consists of 2 disconnected prong tops
    assert_eq!(
        cut.positive_mesh.connected_components_count(),
        2,
        "Cutting across both prongs should produce 2 disconnected fragments in positive half"
    );
    assert!(!cut.positive_mesh.is_contiguous());

    // Negative mesh has the continuous base connecting both prongs
    assert_eq!(cut.negative_mesh.connected_components_count(), 1);
    assert!(cut.negative_mesh.is_contiguous());

    // Optimizer with enforce_contiguous: true must reject prong-fragmenting cuts
    let config_enforce = OptimizerConfig {
        max_pieces: 2,
        enforce_contiguous: true,
        ..OptimizerConfig::default()
    };
    let result_enforce = optimize_partitions(&u_fork, &config_enforce);

    for piece in &result_enforce.pieces {
        assert!(
            piece.mesh.is_contiguous(),
            "Piece {} must be a single contiguous solid",
            piece.id
        );
        assert_eq!(
            piece.mesh.connected_components_count(),
            1,
            "Piece {} must have exactly 1 connected component",
            piece.id
        );
        assert!(
            piece.mesh.is_closed() && piece.mesh.is_watertight(),
            "Piece {} must be closed and watertight",
            piece.id
        );
    }
}

#[test]
fn test_adaptive_pin_sizing_narrow_section() {
    // 6x6x20 mm narrow bar cut at Z = 10.0 (cross-section is 6x6 mm)
    let narrow_bar = create_box(Vec3::ZERO, Vec3::new(6.0, 6.0, 20.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 10.0), Vec3::Z);
    let cut = slice_mesh_by_plane(&narrow_bar, &plane).expect("Cut must succeed");

    // 1. Default 4.0 mm pin (+ 2*0.2 + 2*2.0 = 8.4 mm) does NOT fit in 6x6 mm section
    let default_config = PinConfig::default();
    let default_placements = find_pin_placements(&cut, &default_config);
    assert!(
        default_placements.is_empty(),
        "4.0 mm pin cannot fit in 6x6 mm section"
    );
    let (_, _, default_pins) = apply_female_sockets_and_generate_pins(&cut, &default_config);
    assert_eq!(
        default_pins.len(),
        0,
        "No pins should be generated with default fixed size"
    );

    // 2. Adaptive pin sizing selects a smaller candidate pin (e.g. 2.0 or 2.5 mm)
    let adaptive_config = PinConfig {
        adaptive: true,
        min_size: 1.5,
        ..PinConfig::default()
    };
    let adaptive_placements = find_pin_placements(&cut, &adaptive_config);
    assert_eq!(
        adaptive_placements.len(),
        1,
        "Adaptive sizing must place 1 pin"
    );

    let adapted_size = adaptive_placements[0].size;
    assert!(
        (1.5..4.0).contains(&adapted_size),
        "Adapted size must be smaller than 4.0 and >= 1.5 mm, got {}",
        adapted_size
    );

    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(&cut, &adaptive_config);
    assert_eq!(pins.len(), 1, "Exactly 1 pin must be generated");

    // Verify watertightness of all output meshes
    assert!(pos.is_closed(), "Positive mesh must be closed 2-manifold");
    assert!(pos.is_watertight(), "Positive mesh must be watertight");
    assert!(neg.is_closed(), "Negative mesh must be closed 2-manifold");
    assert!(neg.is_watertight(), "Negative mesh must be watertight");
    assert!(pins[0].is_closed(), "Pin mesh must be closed 2-manifold");
    assert!(pins[0].is_watertight(), "Pin mesh must be watertight");

    // Verify pin dimensions match adapted size
    let pin_dim = pins[0].aabb().size();
    assert!(
        (pin_dim.x - adapted_size).abs() < 1e-3,
        "Pin X dimension should match adapted size {}, got {}",
        adapted_size,
        pin_dim.x
    );
    assert!(
        (pin_dim.y - adapted_size).abs() < 1e-3,
        "Pin Y dimension should match adapted size {}, got {}",
        adapted_size,
        pin_dim.y
    );
    let expected_pin_len = 2.0 * adaptive_config.depth - 0.4;
    assert!(
        (pin_dim.z - expected_pin_len).abs() < 1e-3,
        "Pin Z dimension should match expected length {}, got {}",
        expected_pin_len,
        pin_dim.z
    );
}

#[test]
fn test_all_pieces_watertight_and_closed_under_partition() {
    let t_bracket = create_t_bracket(Vec3::new(20.0, 20.0, 40.0), Vec3::new(60.0, 20.0, 15.0));
    let config = OptimizerConfig {
        max_pieces: 2,
        enforce_contiguous: true,
        pin_config: Some(PinConfig {
            size: 4.0,
            depth: 5.0,
            clearance: 0.2,
            edge_margin: 2.0,
            adaptive: true,
            min_size: 1.5,
        }),
        ..OptimizerConfig::default()
    };

    let result = optimize_partitions(&t_bracket, &config);
    assert_eq!(result.cuts_made, 1);
    assert_eq!(result.pieces.len(), 2);

    for piece in &result.pieces {
        assert!(piece.mesh.is_closed());
        assert!(piece.mesh.is_watertight());
        assert!(piece.mesh.is_contiguous());
    }

    for pin in &result.pins {
        assert!(pin.is_closed());
        assert!(pin.is_watertight());
    }
}

#[test]
fn test_debris_removal_guarantees_contiguous_solid() {
    let main_box = create_box(Vec3::ZERO, Vec3::new(30.0, 30.0, 30.0));
    // Tiny floating fragment of volume 1.0 mm³ (main box is 27,000 mm³, fragment is ~0.0037%)
    let tiny_fragment = create_box(Vec3::new(40.0, 40.0, 40.0), Vec3::new(41.0, 41.0, 41.0));
    let mut tris = main_box.triangles;
    tris.extend(tiny_fragment.triangles);
    let mesh_with_debris = Mesh::new(tris);

    assert_eq!(mesh_with_debris.connected_components_count(), 2);
    assert!(!mesh_with_debris.is_contiguous());

    let config = OptimizerConfig {
        max_pieces: 2,
        enforce_contiguous: true,
        ..OptimizerConfig::default()
    };
    let result = optimize_partitions(&mesh_with_debris, &config);
    for piece in &result.pieces {
        assert!(piece.mesh.is_contiguous());
        assert_eq!(piece.mesh.connected_components_count(), 1);
        assert!(piece.mesh.is_closed());
    }
}

