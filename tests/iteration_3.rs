use glam::Vec3;
use slyser::cut::{compute_plane_basis, slice_mesh_by_plane};
use slyser::mesh::Plane;
use slyser::primitives::{create_cube, create_t_bracket};

#[test]
fn test_cube_horizontal_cut_20x20x20() {
    let cube = create_cube(Vec3::splat(20.0));
    assert_eq!(cube.triangle_count(), 12);
    let orig_vol = cube.total_volume();
    assert!((orig_vol - 8000.0).abs() < 1e-2);

    let plane = Plane::new(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&cube, &plane).expect("Horizontal plane cut must succeed");

    let pos_vol = cut.positive_mesh.total_volume();
    let neg_vol = cut.negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    assert!(
        (pos_vol - 4000.0).abs() < 1e-1,
        "Positive mesh volume expected 4000.0, got {:.3}",
        pos_vol
    );
    assert!(
        (neg_vol - 4000.0).abs() < 1e-1,
        "Negative mesh volume expected 4000.0, got {:.3}",
        neg_vol
    );
    assert!(
        (sum_vol - 8000.0).abs() < 1e-2,
        "Sum of volumes expected 8000.0, got {:.3}",
        sum_vol
    );

    assert!(
        (cut.cut_area - 400.0).abs() < 1e-1,
        "Cut area expected 400.0, got {:.3}",
        cut.cut_area
    );

    assert!(cut.positive_mesh.is_closed(), "Positive mesh must be watertight");
    assert!(cut.negative_mesh.is_closed(), "Negative mesh must be watertight");
    assert!(cut.positive_mesh.is_watertight());
    assert!(cut.negative_mesh.is_watertight());

    assert_eq!(cut.cut_polygons_2d.len(), 1);
    assert_eq!(cut.cut_polygons_2d[0].len(), 4);

    let pos_aabb = cut.positive_mesh.aabb();
    assert!((pos_aabb.min.z - 10.0).abs() < 1e-3);
    assert!((pos_aabb.max.z - 20.0).abs() < 1e-3);

    let neg_aabb = cut.negative_mesh.aabb();
    assert!((neg_aabb.min.z - 0.0).abs() < 1e-3);
    assert!((neg_aabb.max.z - 10.0).abs() < 1e-3);
}

#[test]
fn test_cube_diagonal_cut_through_center() {
    let cube = create_cube(Vec3::splat(20.0));
    let center = Vec3::new(10.0, 10.0, 10.0);
    let plane = Plane::new(center, Vec3::new(1.0, 1.0, 0.0));

    let cut = slice_mesh_by_plane(&cube, &plane).expect("Diagonal plane cut must succeed");

    let pos_vol = cut.positive_mesh.total_volume();
    let neg_vol = cut.negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    assert!(
        (pos_vol - 4000.0).abs() < 1.0,
        "Positive volume expected ~4000.0, got {:.3}",
        pos_vol
    );
    assert!(
        (neg_vol - 4000.0).abs() < 1.0,
        "Negative volume expected ~4000.0, got {:.3}",
        neg_vol
    );
    assert!(
        (sum_vol - 8000.0).abs() < 1e-1,
        "Sum volume expected 8000.0, got {:.3}",
        sum_vol
    );

    let expected_area = 20.0 * 20.0 * std::f32::consts::SQRT_2;
    assert!(
        (cut.cut_area - expected_area).abs() < 1.0,
        "Cut area expected ~{:.3}, got {:.3}",
        expected_area,
        cut.cut_area
    );

    assert!(cut.positive_mesh.is_closed(), "Positive mesh must be watertight");
    assert!(cut.negative_mesh.is_closed(), "Negative mesh must be watertight");
    assert_eq!(cut.cut_polygons_2d.len(), 1);
}

#[test]
fn test_cube_oblique_cut_3d() {
    let cube = create_cube(Vec3::splat(20.0));
    let center = Vec3::new(10.0, 10.0, 10.0);
    let plane = Plane::new(center, Vec3::new(1.0, 1.0, 1.0));

    let cut = slice_mesh_by_plane(&cube, &plane).expect("3D oblique plane cut must succeed");

    let pos_vol = cut.positive_mesh.total_volume();
    let neg_vol = cut.negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    assert!(
        (sum_vol - 8000.0).abs() < 1.0,
        "Sum volume expected 8000.0, got {:.3}",
        sum_vol
    );
    assert!(
        (pos_vol - 4000.0).abs() < 1.0,
        "Positive volume expected 4000.0, got {:.3}",
        pos_vol
    );
    assert!(
        (neg_vol - 4000.0).abs() < 1.0,
        "Negative volume expected 4000.0, got {:.3}",
        neg_vol
    );

    assert!(cut.positive_mesh.is_closed(), "Positive mesh must be watertight");
    assert!(cut.negative_mesh.is_closed(), "Negative mesh must be watertight");
}

#[test]
fn test_t_bracket_cut_across_stem() {
    let t_bracket = create_t_bracket(
        Vec3::new(20.0, 20.0, 40.0),
        Vec3::new(60.0, 20.0, 15.0),
    );
    let orig_vol = t_bracket.total_volume();
    // Stem is 20x20x40 = 16000 mm³, overhang bar on top is 60x20x15 = 18000 mm³. Total = 34000 mm³.
    assert!((orig_vol - 34000.0).abs() < 1e-1);

    let plane = Plane::new(Vec3::new(0.0, 0.0, 20.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&t_bracket, &plane).expect("Stem cut must succeed");

    let pos_vol = cut.positive_mesh.total_volume();
    let neg_vol = cut.negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    // Negative half is lower stem 20x20x20 = 8000 mm³
    assert!(
        (neg_vol - 8000.0).abs() < 1.0,
        "Negative half volume expected 8000.0, got {:.3}",
        neg_vol
    );
    // Positive half is upper stem (20x20x20 = 8000) + overhang (18000) = 26000 mm³
    assert!(
        (pos_vol - 26000.0).abs() < 1.0,
        "Positive half volume expected 26000.0, got {:.3}",
        pos_vol
    );
    assert!(
        (sum_vol - 34000.0).abs() < 1.0,
        "Volume sum expected 34000.0, got {:.3}",
        sum_vol
    );

    assert!(
        (cut.cut_area - 400.0).abs() < 1.0,
        "Stem cut area expected 400.0, got {:.3}",
        cut.cut_area
    );

    assert!(cut.positive_mesh.is_closed());
    assert!(cut.negative_mesh.is_closed());
    assert_eq!(cut.cut_polygons_2d.len(), 1);
}

#[test]
fn test_t_bracket_cut_below_overhang_wings() {
    let t_bracket = create_t_bracket(
        Vec3::new(20.0, 20.0, 40.0),
        Vec3::new(60.0, 20.0, 15.0),
    );

    // Overhang wings sit at z = 40mm. Cut exactly at z = 40mm.
    let plane = Plane::new(Vec3::new(0.0, 0.0, 40.0), Vec3::new(0.0, 0.0, 1.0));
    let cut = slice_mesh_by_plane(&t_bracket, &plane).expect("Cut at wing base must succeed");

    let pos_vol = cut.positive_mesh.total_volume();
    let neg_vol = cut.negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    // Negative half is stem: 20x20x40 = 16000 mm³
    assert!(
        (neg_vol - 16000.0).abs() < 1.0,
        "Negative half expected 16000.0, got {:.3}",
        neg_vol
    );
    // Positive half is overhang bar: 60x20x15 = 18000 mm³
    assert!(
        (pos_vol - 18000.0).abs() < 1.0,
        "Positive half expected 18000.0, got {:.3}",
        pos_vol
    );
    assert!((sum_vol - 34000.0).abs() < 1.0);

    assert!(
        (cut.cut_area - 1200.0).abs() < 1.0,
        "Overhang base cut area expected 1200.0, got {:.3}",
        cut.cut_area
    );

    assert!(cut.positive_mesh.is_closed());
    assert!(cut.negative_mesh.is_closed());
}


#[test]
fn test_t_bracket_vertical_symmetry_cut() {
    let t_bracket = create_t_bracket(
        Vec3::new(20.0, 20.0, 40.0),
        Vec3::new(60.0, 20.0, 15.0),
    );

    // Overhang spans X from 0 to 60. Center is at X = 30.
    let plane = Plane::new(Vec3::new(30.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let cut = slice_mesh_by_plane(&t_bracket, &plane).expect("Vertical symmetry cut must succeed");

    let pos_vol = cut.positive_mesh.total_volume();
    let neg_vol = cut.negative_mesh.total_volume();
    let sum_vol = pos_vol + neg_vol;

    // Each symmetric half is 34000 / 2 = 17000 mm³
    assert!(
        (pos_vol - 17000.0).abs() < 1.0,
        "Positive volume expected 17000.0, got {:.3}",
        pos_vol
    );
    assert!(
        (neg_vol - 17000.0).abs() < 1.0,
        "Negative volume expected 17000.0, got {:.3}",
        neg_vol
    );
    assert!((sum_vol - 34000.0).abs() < 1.0);

    assert!(cut.positive_mesh.is_closed());
    assert!(cut.negative_mesh.is_closed());
}


#[test]
fn test_plane_outside_mesh_returns_err() {
    let cube = create_cube(Vec3::splat(20.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 100.0), Vec3::new(0.0, 0.0, 1.0));
    let result = slice_mesh_by_plane(&cube, &plane);
    assert!(result.is_err(), "Plane outside mesh must return Err");
}

#[test]
fn test_zero_normal_returns_err() {
    let cube = create_cube(Vec3::splat(20.0));
    let plane = Plane::new(Vec3::new(0.0, 0.0, 10.0), Vec3::ZERO);
    let result = slice_mesh_by_plane(&cube, &plane);
    assert!(result.is_err(), "Zero plane normal must return Err");
}

#[test]
fn test_plane_basis_orthonormality() {
    let normals = vec![
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 1.0, 1.0).normalize(),
        Vec3::new(0.5, -0.3, 0.8).normalize(),
    ];

    for n in normals {
        let (u, v) = compute_plane_basis(n);

        assert!((u.length() - 1.0).abs() < 1e-5, "u must be unit length");
        assert!((v.length() - 1.0).abs() < 1e-5, "v must be unit length");
        assert!(u.dot(n).abs() < 1e-5, "u must be orthogonal to n");
        assert!(v.dot(n).abs() < 1e-5, "v must be orthogonal to n");
        assert!(u.dot(v).abs() < 1e-5, "u must be orthogonal to v");

        let cross = u.cross(v);
        assert!((cross - n).length() < 1e-5, "u x v must equal n");
    }
}
