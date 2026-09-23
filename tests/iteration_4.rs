use glam::Vec3;
use slyser::cut::slice_mesh_by_plane;
use slyser::mesh::Plane;
use slyser::pins::{
    apply_female_sockets_and_generate_pins, find_pin_placements, generate_square_pin, PinConfig,
};
use slyser::primitives::{create_box, create_cube, create_t_bracket};

#[test]
fn test_square_pin_mesh_dimensions_and_watertightness() {
    let config = PinConfig {
        size: 4.0,
        depth: 5.0,
        clearance: 0.2,
        edge_margin: 2.0,
    };

    let pin = generate_square_pin(&config);

    assert!(pin.is_closed(), "Pin mesh must be closed 2-manifold");
    assert!(pin.is_watertight(), "Pin mesh must be watertight");
    assert_eq!(pin.triangle_count(), 12);

    let aabb = pin.aabb();
    let expected_len = 2.0 * config.depth - 0.4; // 9.6 mm

    assert!(
        (aabb.size().x - config.size).abs() < 1e-4,
        "Pin X size expected {:.2}, got {:.2}",
        config.size,
        aabb.size().x
    );
    assert!(
        (aabb.size().y - config.size).abs() < 1e-4,
        "Pin Y size expected {:.2}, got {:.2}",
        config.size,
        aabb.size().y
    );
    assert!(
        (aabb.size().z - expected_len).abs() < 1e-4,
        "Pin Z size expected {:.2}, got {:.2}",
        expected_len,
        aabb.size().z
    );

    // Ground plane check: placed flat on Z=0
    assert!(
        aabb.min.z.abs() < 1e-4,
        "Pin must be placed flat on Z=0 (min.z = {:.4})",
        aabb.min.z
    );

    let expected_volume = config.size * config.size * expected_len; // 4 * 4 * 9.6 = 153.6 mm³
    let vol = pin.total_volume();
    assert!(
        (vol - expected_volume).abs() < 1e-3,
        "Pin volume expected {:.3}, got {:.3}",
        expected_volume,
        vol
    );
}

#[test]
fn test_cube_30x30x30_cut_and_female_sockets() {
    let cube = create_cube(Vec3::splat(30.0));
    let orig_vol = cube.total_volume();
    assert!((orig_vol - 27000.0).abs() < 1e-2);

    let plane = Plane::new(Vec3::new(0.0, 0.0, 15.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&cube, &plane).expect("Cutting 30x30x30 cube must succeed");

    let pre_pos_vol = cut.positive_mesh.total_volume();
    let pre_neg_vol = cut.negative_mesh.total_volume();
    assert!((pre_pos_vol - 13500.0).abs() < 1e-1);
    assert!((pre_neg_vol - 13500.0).abs() < 1e-1);

    let config = PinConfig::default(); // size: 4.0, depth: 5.0, clearance: 0.2, margin: 2.0
    let placements = find_pin_placements(&cut, &config);
    assert_eq!(
        placements.len(),
        1,
        "30x30 cut face should have 1 pin at centroid"
    );

    let socket_side = config.size + 2.0 * config.clearance; // 4.4 mm
    let socket_vol = socket_side * socket_side * config.depth; // 4.4 * 4.4 * 5.0 = 96.8 mm³

    let (pos_with_sock, neg_with_sock, pins) =
        apply_female_sockets_and_generate_pins(&cut, &config);

    assert_eq!(pins.len(), 1, "Should generate exactly 1 pin mesh");
    assert!(
        pins[0].is_closed() && pins[0].is_watertight(),
        "Pin mesh must be watertight"
    );

    // Watertightness check
    assert!(
        pos_with_sock.is_closed(),
        "Positive mesh with sockets must be closed"
    );
    assert!(
        pos_with_sock.is_watertight(),
        "Positive mesh with sockets must be watertight"
    );
    assert!(
        neg_with_sock.is_closed(),
        "Negative mesh with sockets must be closed"
    );
    assert!(
        neg_with_sock.is_watertight(),
        "Negative mesh with sockets must be watertight"
    );

    // Volume check: volume must decrease by exactly the pocket volume
    let post_pos_vol = pos_with_sock.total_volume();
    let post_neg_vol = neg_with_sock.total_volume();
    let expected_half_vol = 13500.0 - socket_vol; // 13403.2 mm³

    assert!(
        (post_pos_vol - expected_half_vol).abs() < 1.0,
        "Positive mesh volume expected {:.3}, got {:.3} (diff: {:.3})",
        expected_half_vol,
        post_pos_vol,
        (post_pos_vol - expected_half_vol).abs()
    );
    assert!(
        (post_neg_vol - expected_half_vol).abs() < 1.0,
        "Negative mesh volume expected {:.3}, got {:.3} (diff: {:.3})",
        expected_half_vol,
        post_neg_vol,
        (post_neg_vol - expected_half_vol).abs()
    );

    // Pin fit check: pin width must be smaller than socket opening by clearance
    let pin_size = pins[0].aabb().size();
    assert!(
        pin_size.x < socket_side,
        "Pin X ({:.2}) must be smaller than socket ({:.2})",
        pin_size.x,
        socket_side
    );
    assert!(
        pin_size.y < socket_side,
        "Pin Y ({:.2}) must be smaller than socket ({:.2})",
        pin_size.y,
        socket_side
    );
    assert!(
        (socket_side - pin_size.x - 2.0 * config.clearance).abs() < 1e-4,
        "Clearance must match config"
    );
}

#[test]
fn test_cut_section_too_small_for_pin() {
    // 6x6x6 cube cut at Z = 3.0
    let tiny_cube = create_cube(Vec3::splat(6.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 3.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&tiny_cube, &plane).expect("Cut must succeed");

    let config = PinConfig::default(); // requires 4.0 + 2*0.2 + 2*2.0 = 8.4 mm footprint
    let placements = find_pin_placements(&cut, &config);
    assert!(
        placements.is_empty(),
        "6x6 section cannot fit 8.4mm pin footprint; placements must be empty"
    );

    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(&cut, &config);
    assert_eq!(pins.len(), 0, "No pins should be generated");

    // Meshes must remain completely uncorrupted and watertight
    assert!(pos.is_closed() && pos.is_watertight());
    assert!(neg.is_closed() && neg.is_watertight());

    let pos_vol = pos.total_volume();
    let neg_vol = neg.total_volume();
    assert!(
        (pos_vol - 108.0).abs() < 1e-2,
        "Volume must remain exactly 108.0, got {:.3}",
        pos_vol
    );
    assert!(
        (neg_vol - 108.0).abs() < 1e-2,
        "Volume must remain exactly 108.0, got {:.3}",
        neg_vol
    );
}

#[test]
fn test_elongated_section_multiple_pins() {
    // 80x20x20 box cut horizontally
    let box_mesh = create_box(Vec3::ZERO, Vec3::new(80.0, 20.0, 20.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&box_mesh, &plane).expect("Cut must succeed");

    let config = PinConfig::default();
    let placements = find_pin_placements(&cut, &config);
    assert!(
        placements.len() >= 2,
        "Elongated section (80x20) should have at least 2 pins, got {}",
        placements.len()
    );

    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(&cut, &config);
    assert_eq!(pins.len(), placements.len());

    assert!(pos.is_closed() && pos.is_watertight(), "Positive must be watertight");
    assert!(neg.is_closed() && neg.is_watertight(), "Negative must be watertight");

    let socket_side = config.size + 2.0 * config.clearance;
    let single_socket_vol = socket_side * socket_side * config.depth;
    let total_socket_vol = single_socket_vol * (placements.len() as f32);

    let expected_half_vol = 16000.0 - total_socket_vol;
    let pos_vol = pos.total_volume();
    let neg_vol = neg.total_volume();

    assert!(
        (pos_vol - expected_half_vol).abs() < 2.0,
        "Expected pos vol ~{:.3}, got {:.3}",
        expected_half_vol,
        pos_vol
    );
    assert!(
        (neg_vol - expected_half_vol).abs() < 2.0,
        "Expected neg vol ~{:.3}, got {:.3}",
        expected_half_vol,
        neg_vol
    );
}

#[test]
fn test_t_bracket_cut_across_wings_with_pins() {
    let t_bracket = create_t_bracket(Vec3::new(20.0, 20.0, 40.0), Vec3::new(60.0, 20.0, 15.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 30.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&t_bracket, &plane).expect("Stem cut must succeed");

    let config = PinConfig::default();
    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(&cut, &config);

    assert!(pos.is_closed() && pos.is_watertight());
    assert!(neg.is_closed() && neg.is_watertight());

    for pin in &pins {
        assert!(pin.is_closed() && pin.is_watertight());
    }
}

#[test]
fn test_large_section_four_pins() {
    // 100x100x20 box cut horizontally
    let box_mesh = create_box(Vec3::ZERO, Vec3::new(100.0, 100.0, 20.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&box_mesh, &plane).expect("Cut must succeed");

    let config = PinConfig::default();
    let placements = find_pin_placements(&cut, &config);
    assert_eq!(
        placements.len(),
        4,
        "Large 100x100 section should place a 2x2 grid of 4 pins"
    );

    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(&cut, &config);
    assert_eq!(pins.len(), 4);

    assert!(pos.is_closed() && pos.is_watertight());
    assert!(neg.is_closed() && neg.is_watertight());

    for pin in &pins {
        assert!(pin.is_closed() && pin.is_watertight());
    }

    let socket_side = config.size + 2.0 * config.clearance;
    let single_socket_vol = socket_side * socket_side * config.depth;
    let expected_half_vol = 100000.0 - 4.0 * single_socket_vol;

    assert!(
        (pos.total_volume() - expected_half_vol).abs() < 5.0,
        "Positive mesh volume expected ~{:.2}, got {:.2}",
        expected_half_vol,
        pos.total_volume()
    );
    assert!(
        (neg.total_volume() - expected_half_vol).abs() < 5.0,
        "Negative mesh volume expected ~{:.2}, got {:.2}",
        expected_half_vol,
        neg.total_volume()
    );
}

#[test]
fn test_diagonal_cut_with_sockets() {
    let cube = create_cube(Vec3::splat(40.0));
    let center = Vec3::new(20.0, 20.0, 20.0);
    let plane = Plane::new(center, Vec3::new(1.0, 1.0, 0.0));
    let cut = slice_mesh_by_plane(&cube, &plane).expect("Diagonal cut must succeed");

    let config = PinConfig {
        size: 3.0,
        depth: 4.0,
        clearance: 0.15,
        edge_margin: 2.5,
    };

    let placements = find_pin_placements(&cut, &config);
    assert!(!placements.is_empty(), "Diagonal cut must fit at least 1 pin");

    let (pos, neg, pins) = apply_female_sockets_and_generate_pins(&cut, &config);
    assert_eq!(pins.len(), placements.len());

    assert!(pos.is_closed() && pos.is_watertight());
    assert!(neg.is_closed() && neg.is_watertight());

    for pin in &pins {
        assert!(pin.is_closed() && pin.is_watertight());
    }
}

