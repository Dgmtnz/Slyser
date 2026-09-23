use glam::{Quat, Vec3};
use slyser::bvh::BVH;
use slyser::mesh::Triangle;
use slyser::primitives::{create_box, create_cube, create_t_bracket};
use slyser::support::{analyze_support, SupportConfig};
use std::time::Instant;

#[test]
fn test_cube_flat_on_bed_zero_support() {
    let cube = create_cube(Vec3::new(20.0, 20.0, 20.0));
    assert_eq!(cube.aabb().min.z, 0.0);

    let config = SupportConfig::default();
    let metrics = analyze_support(&cube, &config);

    assert_eq!(
        metrics.overhang_triangle_count, 0,
        "A flat cube on the bed should have 0 overhang triangles"
    );
    assert_eq!(metrics.overhang_area, 0.0);
    assert_eq!(metrics.projected_support_area, 0.0);
    assert_eq!(
        metrics.estimated_support_volume, 0.0,
        "A flat cube on the bed should have 0 mm³ support volume"
    );
    assert_eq!(metrics.support_on_bed_volume, 0.0);
    assert_eq!(metrics.support_on_model_volume, 0.0);
}

#[test]
fn test_cube_rotated_45_deg_overhang_detection() {
    let size = Vec3::new(20.0, 20.0, 20.0);
    let cube = create_cube(size);

    // Rotate 45 degrees around Y axis
    let rot = Quat::from_rotation_y(std::f32::consts::FRAC_PI_4);
    let rotated = cube.rotated(rot);

    // Translate so bottom edge touches the bed (z = 0)
    let aabb = rotated.aabb();
    let cube_on_bed = rotated.translated(Vec3::new(0.0, 0.0, -aabb.min.z));
    assert!((cube_on_bed.aabb().min.z - 0.0).abs() < 1e-5);

    // At 45° threshold, faces at 45° angle are classified as overhangs needing support
    let config_45 = SupportConfig::new(45.0, 2.0);
    let metrics_45 = analyze_support(&cube_on_bed, &config_45);

    // The cube has 2 downward-sloping faces (each 2 triangles) => 4 overhang triangles
    assert_eq!(
        metrics_45.overhang_triangle_count, 4,
        "Expected 4 overhang triangles on 45° tilted cube"
    );
    assert!(metrics_45.overhang_area > 0.0);
    assert!(metrics_45.projected_support_area > 0.0);
    assert!(
        metrics_45.estimated_support_volume > 0.0,
        "Expected positive support volume for elevated overhangs"
    );
    assert!(metrics_45.support_on_bed_volume > 0.0);
    assert_eq!(metrics_45.support_on_model_volume, 0.0);

    // At 50° threshold (steeper overhang limit), surfaces at 45° to horizontal do not need support
    let config_50 = SupportConfig::new(50.0, 2.0);
    let metrics_50 = analyze_support(&cube_on_bed, &config_50);
    assert_eq!(
        metrics_50.overhang_triangle_count, 0,
        "At 50° threshold, 45° faces should not require support"
    );
    assert_eq!(metrics_50.estimated_support_volume, 0.0);
}

#[test]
fn test_t_bracket_analytical_support_volume() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    // Analytical values:
    // Left wing: 20 mm wide x 20 mm deep = 400 mm²
    // Right wing: 20 mm wide x 20 mm deep = 400 mm²
    // Total wing area = 800 mm²
    // Height to bed = stem.z = 40 mm
    // Exact analytical support volume = 800 mm² * 40 mm = 32000 mm³
    let expected_area = 800.0f32;
    let expected_volume = 32000.0f32;

    let config = SupportConfig::default();
    let metrics = analyze_support(&t_bracket, &config);

    assert_eq!(
        metrics.overhang_triangle_count, 4,
        "Expected 4 overhang triangles (2 per wing quad)"
    );
    assert!(
        (metrics.overhang_area - expected_area).abs() < 1e-2,
        "Expected overhang area {}, got {}",
        expected_area,
        metrics.overhang_area
    );
    assert!(
        (metrics.projected_support_area - expected_area).abs() < 1e-2,
        "Expected projected support area {}, got {}",
        expected_area,
        metrics.projected_support_area
    );

    // Verify estimated support volume matches analytical within 2% tolerance
    let vol_diff = (metrics.estimated_support_volume - expected_volume).abs();
    let rel_error = vol_diff / expected_volume;
    assert!(
        rel_error < 0.02,
        "Estimated support volume {} deviates from analytical {} by {:.4}% (must be < 2%)",
        metrics.estimated_support_volume,
        expected_volume,
        rel_error * 100.0
    );

    // Support must land entirely on the build platform
    let bed_diff = (metrics.support_on_bed_volume - expected_volume).abs();
    let bed_rel_error = bed_diff / expected_volume;
    assert!(
        bed_rel_error < 0.02,
        "Support on bed {} deviates by {:.4}%",
        metrics.support_on_bed_volume,
        bed_rel_error * 100.0
    );
    assert!(
        metrics.support_on_model_volume < 1e-3,
        "Expected 0 support landing on model, got {}",
        metrics.support_on_model_volume
    );
}

#[test]
fn test_support_on_model_detection() {
    // Construct a stepped model where an overhang sits directly above a lower base
    // Lower base: (0..100, 0..40, 0..20)
    let base = create_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 40.0, 20.0));
    // Upright column: (0..20, 0..40, 20..60)
    let column = create_box(Vec3::new(0.0, 0.0, 20.0), Vec3::new(20.0, 40.0, 60.0));
    // Overhang beam: (20..80, 0..40, 40..60) - elevated above base (z=20 to z=40 is empty air)
    let beam = create_box(Vec3::new(20.0, 0.0, 40.0), Vec3::new(80.0, 40.0, 60.0));

    let mut combined_triangles = Vec::new();
    combined_triangles.extend(base.triangles);
    combined_triangles.extend(column.triangles);
    combined_triangles.extend(beam.triangles);

    let stepped_mesh = slyser::Mesh::new(combined_triangles);
    let config = SupportConfig::default();
    let metrics = analyze_support(&stepped_mesh, &config);

    // The bottom face of the beam at z = 40 (X from 20..80, Y from 0..40) is an overhang.
    // Area = 60 * 40 = 2400 mm²
    // Distance to lower base top (at z = 20) = 40 - 20 = 20 mm.
    // Expected volume on model = 2400 * 20 = 48000 mm³.
    assert!(
        metrics.support_on_model_volume > 0.0,
        "Support landing on model must be detected"
    );
    let expected_on_model = 2400.0 * 20.0;
    let rel_err = (metrics.support_on_model_volume - expected_on_model).abs() / expected_on_model;
    assert!(
        rel_err < 0.02,
        "Expected ~{} support on model, got {} (rel err: {:.4})",
        expected_on_model,
        metrics.support_on_model_volume,
        rel_err
    );
}

#[test]
fn test_bvh_raycasting_primitives() {
    let t1 = Triangle::new(
        Vec3::new(0.0, 0.0, 10.0),
        Vec3::new(10.0, 0.0, 10.0),
        Vec3::new(0.0, 10.0, 10.0),
    );
    let t2 = Triangle::new(
        Vec3::new(0.0, 0.0, 20.0),
        Vec3::new(10.0, 0.0, 20.0),
        Vec3::new(0.0, 10.0, 20.0),
    );

    let bvh = BVH::build(&[t1, t2]);

    // Ray shooting upwards from (2, 2, 0) should hit t1 first at z = 10
    let hit = bvh
        .intersect_ray(Vec3::new(2.0, 2.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
        .expect("Should hit t1");

    assert_eq!(hit.triangle_index, 0);
    assert!((hit.t - 10.0).abs() < 1e-4);
    assert!((hit.hit_point - Vec3::new(2.0, 2.0, 10.0)).length() < 1e-4);
    assert!((hit.hit_normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-4);

    // Ray starting between t1 and t2 shooting upwards should hit t2 at distance 5
    let hit2 = bvh
        .intersect_ray(Vec3::new(2.0, 2.0, 15.0), Vec3::new(0.0, 0.0, 1.0))
        .expect("Should hit t2");
    assert_eq!(hit2.triangle_index, 1);
    assert!((hit2.t - 5.0).abs() < 1e-4);

    // Ray shooting away should hit nothing
    let miss = bvh.intersect_ray(Vec3::new(2.0, 2.0, 0.0), Vec3::new(0.0, 0.0, -1.0));
    assert!(miss.is_none());
}

#[test]
fn test_multithreaded_performance() {
    // Generate an array of 16 T-brackets to form a multi-part scene
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let single = create_t_bracket(stem, overhang);

    let mut combined = Vec::new();
    let count = 16;
    for i in 0..count {
        let offset = Vec3::new((i % 4) as f32 * 80.0, (i / 4) as f32 * 80.0, 0.0);
        let shifted = single.translated(offset);
        combined.extend(shifted.triangles);
    }
    let scene = slyser::Mesh::new(combined);

    let config = SupportConfig::default();
    let start = Instant::now();
    let metrics = analyze_support(&scene, &config);
    let duration = start.elapsed();

    // Verify linear scaling of analytical support volume
    let expected_total_vol = 32000.0 * (count as f32);
    let rel_err = (metrics.estimated_support_volume - expected_total_vol).abs() / expected_total_vol;
    assert!(
        rel_err < 0.02,
        "Expected total volume {}, got {}, error {:.4}%",
        expected_total_vol,
        metrics.estimated_support_volume,
        rel_err * 100.0
    );

    assert_eq!(metrics.overhang_triangle_count, 4 * count);
    assert!(
        duration.as_millis() < 500,
        "Multithreaded support analysis took {:?}, expected < 500ms",
        duration
    );
}
