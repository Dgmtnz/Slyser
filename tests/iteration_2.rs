use glam::{Quat, Vec3};
use slyser::mesh::{Mesh, Triangle};
use slyser::orient::{
    compute_bed_contact_area, extract_candidate_orientations, find_optimal_orientation,
    is_orientation_stable, orient_mesh_to_bed,
};
use slyser::primitives::{create_box, create_cube, create_t_bracket};
use slyser::support::{analyze_support, SupportConfig};

/// Helper to generate a watertight right triangular prism (wedge).
/// Dimensions: w along X, d along Y, h along Z.
fn create_wedge(w: f32, d: f32, h: f32) -> Mesh {
    let triangles = vec![
        // 1. Bottom face (at z = 0, normal -Z)
        Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, d, 0.0),
            Vec3::new(w, d, 0.0),
        ),
        Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(w, d, 0.0),
            Vec3::new(w, 0.0, 0.0),
        ),
        // 2. Back vertical wall (at x = w, normal +X)
        Triangle::new(
            Vec3::new(w, 0.0, 0.0),
            Vec3::new(w, d, 0.0),
            Vec3::new(w, d, h),
        ),
        Triangle::new(
            Vec3::new(w, 0.0, 0.0),
            Vec3::new(w, d, h),
            Vec3::new(w, 0.0, h),
        ),
        // 3. Left triangle (at y = 0, normal -Y)
        Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(w, 0.0, 0.0),
            Vec3::new(w, 0.0, h),
        ),
        // 4. Right triangle (at y = d, normal +Y)
        Triangle::new(
            Vec3::new(0.0, d, 0.0),
            Vec3::new(w, d, h),
            Vec3::new(w, d, 0.0),
        ),
        // 5. Slanted hypotenuse face (normal: (-h, 0, w).normalize())
        Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(w, 0.0, h),
            Vec3::new(w, d, h),
        ),
        Triangle::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(w, d, h),
            Vec3::new(0.0, d, 0.0),
        ),
    ];

    Mesh::new(triangles)
}

#[test]
fn test_cube_canonical_orientations_zero_support() {
    let size = Vec3::new(20.0, 20.0, 20.0);
    let cube = create_cube(size);
    let config = SupportConfig::default();

    // In original orientation, cube has 0 support
    let orig_metrics = analyze_support(&cube, &config);
    assert_eq!(orig_metrics.estimated_support_volume, 0.0);

    let optimal = find_optimal_orientation(&cube, &config);

    assert_eq!(
        optimal.metrics.estimated_support_volume, 0.0,
        "Optimal orientation of a cube must require 0.0 mm³ support"
    );
    assert_eq!(optimal.metrics.support_on_model_volume, 0.0);
    assert!(
        (optimal.contact_area - 400.0).abs() < 1e-2,
        "Contact area should be 400 mm², got {}",
        optimal.contact_area
    );
    assert!(optimal.is_stable, "Cube orientation must be stable");
    assert!(
        (optimal.mesh.aabb().min.z - 0.0).abs() < 1e-5,
        "Reoriented mesh must rest at z = 0"
    );
}

#[test]
fn test_tilted_cube_reoriented_to_flat_base() {
    let size = Vec3::new(20.0, 20.0, 20.0);
    let cube = create_cube(size);

    // Rotate cube by 45 degrees around Y-axis so it stands on an edge
    let rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_4);
    let tilted_cube = orient_mesh_to_bed(&cube, rot);

    let config_45 = SupportConfig::new(45.0, 2.0);
    let tilted_metrics = analyze_support(&tilted_cube, &config_45);
    assert!(
        tilted_metrics.estimated_support_volume > 0.0,
        "Tilted cube standing on an edge must generate positive support volume"
    );

    // Run orientation optimizer
    let optimal = find_optimal_orientation(&tilted_cube, &config_45);

    assert_eq!(
        optimal.metrics.estimated_support_volume, 0.0,
        "Optimal orientation must recover 0.0 mm³ support for cube"
    );
    assert!(
        (optimal.contact_area - 400.0).abs() < 1e-2,
        "Expected 400 mm² bed contact area, got {}",
        optimal.contact_area
    );
    assert!(optimal.is_stable, "Recovered flat cube must be stable");
    assert!((optimal.mesh.aabb().min.z - 0.0).abs() < 1e-5);
}

#[test]
fn test_t_bracket_upside_down_100_percent_reduction() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let config = SupportConfig::default();
    let orig_metrics = analyze_support(&t_bracket, &config);

    // Upright T-bracket has ~32,000 mm³ support volume under wings
    assert!(
        orig_metrics.estimated_support_volume > 30000.0,
        "Original T-bracket must require ~32000 mm³ support, got {}",
        orig_metrics.estimated_support_volume
    );

    // 1. Verify directly that upside-down orientation gives 0 support volume and 1200 contact area
    let upside_down = orient_mesh_to_bed(&t_bracket, Quat::from_rotation_x(std::f32::consts::PI));
    let upside_down_metrics = analyze_support(&upside_down, &config);
    assert_eq!(
        upside_down_metrics.estimated_support_volume, 0.0,
        "Upside-down T-bracket must require 0.0 mm³ support"
    );
    assert_eq!(upside_down_metrics.overhang_triangle_count, 0);
    let upside_down_contact_area = compute_bed_contact_area(&upside_down);
    assert!(
        (upside_down_contact_area - 1200.0).abs() < 1e-2,
        "Expected upside-down contact area 1200 mm², got {}",
        upside_down_contact_area
    );
    assert!(is_orientation_stable(&upside_down));

    // 2. Verify find_optimal_orientation finds an orientation with 0.0 mm³ support and 100% reduction
    let optimal = find_optimal_orientation(&t_bracket, &config);

    assert_eq!(
        optimal.metrics.estimated_support_volume, 0.0,
        "Optimal orientation must have 0.0 mm³ support volume, got {}",
        optimal.metrics.estimated_support_volume
    );
    assert_eq!(
        optimal.metrics.overhang_triangle_count, 0,
        "Optimal orientation must have 0 overhang triangles"
    );
    assert_eq!(optimal.metrics.support_on_model_volume, 0.0);

    // Optimal contact area is at least 1200 mm² (1200 on top face or 1700 on flat side)
    assert!(
        optimal.contact_area >= 1200.0,
        "Expected contact area >= 1200 mm², got {}",
        optimal.contact_area
    );

    assert!(optimal.is_stable, "Optimal T-bracket orientation must be stable");
    assert!(
        (optimal.mesh.aabb().min.z - 0.0).abs() < 1e-5,
        "Reoriented mesh must rest at z = 0"
    );

    let reduction_pct = (orig_metrics.estimated_support_volume
        - optimal.metrics.estimated_support_volume)
        / orig_metrics.estimated_support_volume
        * 100.0;
    assert!(
        (reduction_pct - 100.0).abs() < 1e-4,
        "Expected 100% support reduction, got {:.2}%",
        reduction_pct
    );
}

#[test]
fn test_asymmetric_t_bracket_uniquely_selects_top_face() {
    // Overhang is wider and deeper than stem, so side faces are not flat
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 30.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let config = SupportConfig::default();
    let orig_metrics = analyze_support(&t_bracket, &config);
    assert!(orig_metrics.estimated_support_volume > 0.0);

    let optimal = find_optimal_orientation(&t_bracket, &config);

    // Support volume drops to 0.0 mm³
    assert_eq!(
        optimal.metrics.estimated_support_volume, 0.0,
        "Optimal orientation must have 0.0 mm³ support volume"
    );
    // Uniquely resting on the 60x30 top face = 1800 mm²
    assert!(
        (optimal.contact_area - 1800.0).abs() < 1e-2,
        "Expected contact area 1800 mm², got {}",
        optimal.contact_area
    );
    assert!(optimal.is_stable);
}

#[test]
fn test_slanted_wedge_model_optimization() {
    // 40x30x40 wedge
    let wedge = create_wedge(40.0, 30.0, 40.0);

    // Invert wedge 180° around X axis so it balances on its thin edge at z=0,
    // with its 45° slanted face pointing downwards and requiring support
    let rot = Quat::from_rotation_x(std::f32::consts::PI);
    let inverted_wedge = orient_mesh_to_bed(&wedge, rot);

    // With a 40° overhang threshold, the downward 45° face requires support
    let config = SupportConfig::new(40.0, 2.0);
    let inverted_metrics = analyze_support(&inverted_wedge, &config);
    assert!(
        inverted_metrics.estimated_support_volume > 0.0,
        "Inverted wedge must generate overhangs requiring support, got {}",
        inverted_metrics.estimated_support_volume
    );

    let optimal = find_optimal_orientation(&inverted_wedge, &config);

    // The optimizer should place it either on its bottom base (40x30 = 1200 mm²)
    // or on its hypotenuse face, eliminating support.
    assert_eq!(
        optimal.metrics.estimated_support_volume, 0.0,
        "Optimal orientation for wedge must have 0.0 mm³ support volume, got {}",
        optimal.metrics.estimated_support_volume
    );
    assert!(optimal.contact_area >= 1200.0);
    assert!(optimal.is_stable);
}

#[test]
fn test_mechanical_stability_evaluation() {
    // 1. Stable cube
    let cube = create_cube(Vec3::new(20.0, 20.0, 20.0));
    assert!(is_orientation_stable(&cube));

    // 2. Unstable inverted / leaning structure
    // Narrow base: [0..4, 0..4, 0..10]
    let base = create_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 4.0, 10.0));
    // Heavy cantilever block hanging far off to +X: [4..80, 0..4, 10..30]
    let arm = create_box(Vec3::new(4.0, 0.0, 10.0), Vec3::new(80.0, 4.0, 30.0));

    let mut combined = Vec::new();
    combined.extend(base.triangles);
    combined.extend(arm.triangles);
    let cantilever = Mesh::new(combined);

    // Center of mass is far outside [0..4] in X
    let com = cantilever.center_of_mass();
    assert!(com.x > 10.0, "COM x should be > 10, got {}", com.x);

    // Only the narrow base touches the bed at z=0 (x in [0..4], y in [0..4])
    assert!(
        !is_orientation_stable(&cantilever),
        "Cantilever with COM outside footprint must be evaluated as unstable"
    );
}

#[test]
fn test_candidate_orientations_extraction() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    let candidates = extract_candidate_orientations(&t_bracket);

    // Must include canonical axes (+-X, +-Y, +-Z)
    assert!(
        candidates.len() >= 6,
        "Must have at least 6 canonical candidates, got {}",
        candidates.len()
    );

    // Verify +Z normal candidate is present with ~1200 mm² area (the top overhang face)
    let top_face_cand = candidates
        .iter()
        .find(|c| c.contact_normal.dot(Vec3::new(0.0, 0.0, 1.0)) >= 0.99)
        .expect("Must have candidate corresponding to +Z face");

    assert!(
        (top_face_cand.contact_area - 1200.0).abs() < 1e-2,
        "Top face candidate area should be 1200 mm², got {}",
        top_face_cand.contact_area
    );
}
