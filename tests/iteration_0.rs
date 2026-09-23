use glam::{Quat, Vec3};
use slyser::mesh::{Mesh, Plane, Triangle, AABB};
use slyser::primitives::{create_box, create_cube, create_t_bracket};
use slyser::stl::{read_stl, write_stl_ascii, write_stl_binary};
use std::io::Cursor;

#[test]
fn test_triangle_area_and_normal() {
    // Right triangle in XY plane with legs of length 3 and 4 -> Area should be 6.0
    let v0 = Vec3::new(0.0, 0.0, 0.0);
    let v1 = Vec3::new(3.0, 0.0, 0.0);
    let v2 = Vec3::new(0.0, 4.0, 0.0);

    let tri = Triangle::new(v0, v1, v2);
    assert!((tri.area() - 6.0).abs() < 1e-5);
    assert!((tri.normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-5);

    // Reversed winding should flip normal
    let tri_reversed = Triangle::new(v0, v2, v1);
    assert!((tri_reversed.area() - 6.0).abs() < 1e-5);
    assert!((tri_reversed.normal - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-5);

    // 45-degree slanted triangle
    let s0 = Vec3::new(0.0, 0.0, 0.0);
    let s1 = Vec3::new(10.0, 0.0, 0.0);
    let s2 = Vec3::new(0.0, 10.0, 10.0);
    let tri_slanted = Triangle::new(s0, s1, s2);
    let expected_normal = Vec3::new(0.0, -1.0, 1.0).normalize();
    assert!((tri_slanted.normal - expected_normal).length() < 1e-5);
}

#[test]
fn test_plane_distance_and_ray_intersection() {
    let plane_pt = Vec3::new(0.0, 0.0, 10.0);
    let plane_n = Vec3::new(0.0, 0.0, 1.0);
    let plane = Plane::new(plane_pt, plane_n);

    // Point above plane
    let p_above = Vec3::new(5.0, 3.0, 15.0);
    assert!((plane.signed_distance(p_above) - 5.0).abs() < 1e-5);
    assert!((plane.distance(p_above) - 5.0).abs() < 1e-5);

    // Point below plane
    let p_below = Vec3::new(-2.0, 4.0, 3.0);
    assert!((plane.signed_distance(p_below) - (-7.0)).abs() < 1e-5);
    assert!((plane.distance(p_below) - 7.0).abs() < 1e-5);

    // Orthogonal projection
    let proj = plane.project_point(p_above);
    assert!((proj - Vec3::new(5.0, 3.0, 10.0)).length() < 1e-5);

    // Ray intersection hitting plane
    let ray_origin = Vec3::new(2.0, 2.0, 0.0);
    let ray_dir = Vec3::new(0.0, 0.0, 1.0);
    let t = plane.intersect_ray(ray_origin, ray_dir);
    assert_eq!(t, Some(10.0));

    // Ray pointing away from plane
    let ray_away_dir = Vec3::new(0.0, 0.0, -1.0);
    assert_eq!(plane.intersect_ray(ray_origin, ray_away_dir), None);

    // Ray parallel to plane
    let ray_parallel_dir = Vec3::new(1.0, 0.0, 0.0);
    assert_eq!(plane.intersect_ray(ray_origin, ray_parallel_dir), None);
}

#[test]
fn test_aabb_operations() {
    let mut aabb = AABB::empty();
    assert!(aabb.is_empty());

    aabb.extend(Vec3::new(-5.0, 2.0, 10.0));
    aabb.extend(Vec3::new(15.0, -3.0, 4.0));

    assert_eq!(aabb.min, Vec3::new(-5.0, -3.0, 4.0));
    assert_eq!(aabb.max, Vec3::new(15.0, 2.0, 10.0));
    assert_eq!(aabb.size(), Vec3::new(20.0, 5.0, 6.0));
    assert_eq!(aabb.center(), Vec3::new(5.0, -0.5, 7.0));

    assert!(aabb.contains(Vec3::new(0.0, 0.0, 5.0)));
    assert!(!aabb.contains(Vec3::new(25.0, 0.0, 5.0)));
}

#[test]
fn test_mesh_volume_area_and_com() {
    let size = Vec3::new(10.0, 20.0, 30.0);
    let cube = create_cube(size);

    // Expected volume: 10 * 20 * 30 = 6000
    let vol = cube.total_volume();
    assert!((vol - 6000.0).abs() < 1e-2, "Expected 6000.0, got {vol}");

    // Expected surface area: 2 * (10*20 + 20*30 + 30*10) = 2 * (200 + 600 + 300) = 2200
    let area = cube.total_surface_area();
    assert!((area - 2200.0).abs() < 1e-2, "Expected 2200.0, got {area}");

    // Expected center of mass: (5, 10, 15)
    let com = cube.center_of_mass();
    let expected_com = size * 0.5;
    assert!(
        (com - expected_com).length() < 1e-2,
        "Expected {:?}, got {:?}",
        expected_com,
        com
    );

    // Test translation
    let mut translated_cube = cube.clone();
    let translation = Vec3::new(15.0, -5.0, 2.5);
    translated_cube.translate(translation);
    let new_com = translated_cube.center_of_mass();
    assert!(
        (new_com - (expected_com + translation)).length() < 1e-2,
        "Expected {:?}, got {:?}",
        expected_com + translation,
        new_com
    );
}

#[test]
fn test_mesh_rotation() {
    let cube = create_cube(Vec3::new(10.0, 20.0, 30.0));
    // Rotate 90 degrees around Z axis: (x, y, z) -> (-y, x, z)
    let rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let rotated = cube.rotated(rotation);

    let aabb = rotated.aabb();
    // After 90 deg rot around Z, X extent is [-20, 0], Y extent is [0, 10], Z extent is [0, 30]
    assert!((aabb.min.x - (-20.0)).abs() < 1e-4);
    assert!((aabb.max.x - 0.0).abs() < 1e-4);
    assert!((aabb.min.y - 0.0).abs() < 1e-4);
    assert!((aabb.max.y - 10.0).abs() < 1e-4);
    assert!((aabb.min.z - 0.0).abs() < 1e-4);
    assert!((aabb.max.z - 30.0).abs() < 1e-4);

    // Volume should be invariant under rotation
    assert!((rotated.total_volume() - 6000.0).abs() < 1e-2);
}

#[test]
fn test_stl_binary_roundtrip_cube() {
    let original = create_cube(Vec3::new(12.5, 24.0, 36.5));
    let mut buffer = Vec::new();

    // Write binary
    write_stl_binary(&mut buffer, &original).expect("Failed to write binary STL");

    // Read back
    let mut cursor = Cursor::new(buffer);
    let loaded = read_stl(&mut cursor).expect("Failed to read binary STL");

    assert_eq!(loaded.triangle_count(), original.triangle_count());
    assert_eq!(loaded.triangle_count(), 12);

    let orig_aabb = original.aabb();
    let load_aabb = loaded.aabb();
    assert!((orig_aabb.min - load_aabb.min).length() < 1e-4);
    assert!((orig_aabb.max - load_aabb.max).length() < 1e-4);

    assert!((original.total_volume() - loaded.total_volume()).abs() < 1e-2);
    assert!((original.total_surface_area() - loaded.total_surface_area()).abs() < 1e-2);
}

#[test]
fn test_stl_binary_roundtrip_t_bracket() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let original = create_t_bracket(stem, overhang);

    let mut buffer = Vec::new();
    write_stl_binary(&mut buffer, &original).expect("Failed to write binary STL");

    let mut cursor = Cursor::new(buffer);
    let loaded = read_stl(&mut cursor).expect("Failed to read binary STL");

    assert_eq!(loaded.triangle_count(), original.triangle_count());

    let orig_aabb = original.aabb();
    let load_aabb = loaded.aabb();
    assert!((orig_aabb.min - load_aabb.min).length() < 1e-4);
    assert!((orig_aabb.max - load_aabb.max).length() < 1e-4);

    // Expected volume = stem volume + overhang volume = (20*20*40) + (60*20*15) = 16000 + 18000 = 34000
    let expected_vol = 20.0 * 20.0 * 40.0 + 60.0 * 20.0 * 15.0;
    assert!(
        (loaded.total_volume() - expected_vol).abs() < 1e-1,
        "Expected vol {}, got {}",
        expected_vol,
        loaded.total_volume()
    );
}

#[test]
fn test_stl_ascii_roundtrip() {
    let original = create_cube(Vec3::new(5.0, 5.0, 5.0));
    let mut buffer = Vec::new();

    write_stl_ascii(&mut buffer, &original, "test_cube").expect("Failed to write ASCII STL");

    let mut cursor = Cursor::new(buffer);
    let loaded = read_stl(&mut cursor).expect("Failed to read ASCII STL");

    assert_eq!(loaded.triangle_count(), 12);
    let orig_aabb = original.aabb();
    let load_aabb = loaded.aabb();
    assert!((orig_aabb.min - load_aabb.min).length() < 1e-4);
    assert!((orig_aabb.max - load_aabb.max).length() < 1e-4);
}

#[test]
fn test_binary_stl_with_solid_in_header() {
    let original = create_cube(Vec3::new(10.0, 10.0, 10.0));
    let mut buffer = Vec::new();
    write_stl_binary(&mut buffer, &original).unwrap();

    // Overwrite the first bytes of the binary header with "solid"
    buffer[0..5].copy_from_slice(b"solid");

    let mut cursor = Cursor::new(buffer);
    let loaded = read_stl(&mut cursor).expect("Should correctly detect binary STL despite 'solid' in header");

    assert_eq!(loaded.triangle_count(), 12);
    assert!((loaded.total_volume() - 1000.0).abs() < 1e-2);
}

#[test]
fn test_t_bracket_guaranteed_90_deg_overhangs() {
    let stem = Vec3::new(20.0, 20.0, 40.0);
    let overhang = Vec3::new(60.0, 20.0, 15.0);
    let t_bracket = create_t_bracket(stem, overhang);

    // Overhang extends on X from 0..20 (left wing) and 40..60 (right wing) at z = 40.
    // Total downward overhang area should be: (60 - 20) * 20 = 800 mm²
    let mut overhang_down_area = 0.0f32;
    let down_normal = Vec3::new(0.0, 0.0, -1.0);

    for tri in &t_bracket.triangles {
        // Face points directly down (-Z)
        if (tri.normal - down_normal).length() < 1e-4 {
            // Check that it is elevated above the bed (at z = 40)
            if (tri.vertices[0].z - 40.0).abs() < 1e-4
                && (tri.vertices[1].z - 40.0).abs() < 1e-4
                && (tri.vertices[2].z - 40.0).abs() < 1e-4
            {
                overhang_down_area += tri.area();
            }
        }
    }

    assert!(
        (overhang_down_area - 800.0).abs() < 1e-2,
        "Expected 800.0 mm² of 90-degree overhang area at z=40, got {overhang_down_area}"
    );
}

#[test]
fn test_create_box_and_affine_transform() {
    use glam::Affine3A;

    let b = create_box(Vec3::new(1.0, 2.0, 3.0), Vec3::new(4.0, 6.0, 8.0));
    assert_eq!(b.triangle_count(), 12);
    // Dimensions: (3, 4, 5) => volume = 60
    assert!((b.total_volume() - 60.0).abs() < 1e-2);

    let affine = Affine3A::from_scale_rotation_translation(
        Vec3::splat(2.0),
        Quat::IDENTITY,
        Vec3::new(10.0, 20.0, 30.0),
    );
    let transformed: Mesh = b.transformed(affine);
    // Scaled by 2 in all dimensions: volume should be 60 * 8 = 480
    assert!((transformed.total_volume() - 480.0).abs() < 1e-1);
    let aabb = transformed.aabb();
    assert_eq!(aabb.min, Vec3::new(12.0, 24.0, 36.0));
    assert_eq!(aabb.max, Vec3::new(18.0, 32.0, 46.0));
}

