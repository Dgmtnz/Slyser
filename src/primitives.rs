use crate::mesh::{Mesh, Triangle};
use glam::Vec3;

/// Helper to add a planar quadrilateral to the triangle list with outward CCW winding.
fn add_quad(triangles: &mut Vec<Triangle>, v0: Vec3, v1: Vec3, v2: Vec3, v3: Vec3) {
    triangles.push(Triangle::new(v0, v1, v2));
    triangles.push(Triangle::new(v0, v2, v3));
}

/// Helper to add a quad facing -Z at a fixed z coordinate.
fn add_quad_down(triangles: &mut Vec<Triangle>, x0: f32, x1: f32, y0: f32, y1: f32, z: f32) {
    if x1 > x0 && y1 > y0 {
        let v0 = Vec3::new(x0, y0, z);
        let v1 = Vec3::new(x0, y1, z);
        let v2 = Vec3::new(x1, y1, z);
        let v3 = Vec3::new(x1, y0, z);
        add_quad(triangles, v0, v1, v2, v3);
    }
}

/// Helper to add a quad facing +Z at a fixed z coordinate.
fn add_quad_up(triangles: &mut Vec<Triangle>, x0: f32, x1: f32, y0: f32, y1: f32, z: f32) {
    if x1 > x0 && y1 > y0 {
        let v0 = Vec3::new(x0, y0, z);
        let v1 = Vec3::new(x1, y0, z);
        let v2 = Vec3::new(x1, y1, z);
        let v3 = Vec3::new(x0, y1, z);
        add_quad(triangles, v0, v1, v2, v3);
    }
}

/// Helper to add a quad facing -Y at a fixed y coordinate.
fn add_quad_front(triangles: &mut Vec<Triangle>, x0: f32, x1: f32, z0: f32, z1: f32, y: f32) {
    if x1 > x0 && z1 > z0 {
        let v0 = Vec3::new(x0, y, z0);
        let v1 = Vec3::new(x1, y, z0);
        let v2 = Vec3::new(x1, y, z1);
        let v3 = Vec3::new(x0, y, z1);
        add_quad(triangles, v0, v1, v2, v3);
    }
}

/// Helper to add a quad facing +Y at a fixed y coordinate.
fn add_quad_back(triangles: &mut Vec<Triangle>, x0: f32, x1: f32, z0: f32, z1: f32, y: f32) {
    if x1 > x0 && z1 > z0 {
        let v0 = Vec3::new(x1, y, z0);
        let v1 = Vec3::new(x0, y, z0);
        let v2 = Vec3::new(x0, y, z1);
        let v3 = Vec3::new(x1, y, z1);
        add_quad(triangles, v0, v1, v2, v3);
    }
}

/// Helper to add a quad facing -X at a fixed x coordinate.
fn add_quad_left(triangles: &mut Vec<Triangle>, y0: f32, y1: f32, z0: f32, z1: f32, x: f32) {
    if y1 > y0 && z1 > z0 {
        let v0 = Vec3::new(x, y1, z0);
        let v1 = Vec3::new(x, y0, z0);
        let v2 = Vec3::new(x, y0, z1);
        let v3 = Vec3::new(x, y1, z1);
        add_quad(triangles, v0, v1, v2, v3);
    }
}

/// Helper to add a quad facing +X at a fixed x coordinate.
fn add_quad_right(triangles: &mut Vec<Triangle>, y0: f32, y1: f32, z0: f32, z1: f32, x: f32) {
    if y1 > y0 && z1 > z0 {
        let v0 = Vec3::new(x, y0, z0);
        let v1 = Vec3::new(x, y1, z0);
        let v2 = Vec3::new(x, y1, z1);
        let v3 = Vec3::new(x, y0, z1);
        add_quad(triangles, v0, v1, v2, v3);
    }
}

/// Creates an axis-aligned box between `min` and `max`.
pub fn create_box(min: Vec3, max: Vec3) -> Mesh {
    let mut triangles = Vec::with_capacity(12);

    add_quad_down(&mut triangles, min.x, max.x, min.y, max.y, min.z);
    add_quad_up(&mut triangles, min.x, max.x, min.y, max.y, max.z);
    add_quad_front(&mut triangles, min.x, max.x, min.z, max.z, min.y);
    add_quad_back(&mut triangles, min.x, max.x, min.z, max.z, max.y);
    add_quad_left(&mut triangles, min.y, max.y, min.z, max.z, min.x);
    add_quad_right(&mut triangles, min.y, max.y, min.z, max.z, max.x);

    Mesh::new(triangles)
}

/// Generates a solid cube of dimensions `size`, placed on the ground plane (min at (0, 0, 0)).
pub fn create_cube(size: Vec3) -> Mesh {
    create_box(Vec3::ZERO, size)
}

/// Generates a centered solid cube of dimensions `size` centered at origin.
pub fn create_cube_centered(size: Vec3) -> Mesh {
    let half = size * 0.5;
    create_box(-half, half)
}

/// Generates a solid T-bracket test model consisting of a vertical stem and a horizontal overhang.
///
/// The model is placed on the build platform ($Z \ge 0$), with the stem centered in XY
/// underneath the overhang bar. Downward-facing faces underneath the overhang wings
/// form guaranteed 90-degree overhangs relative to vertical walls (pointing directly along -Z).
/// The resulting mesh is a single watertight 2-manifold polyhedron with no internal faces.
pub fn create_t_bracket(stem: Vec3, overhang: Vec3) -> Mesh {
    let w = overhang.x.max(stem.x);
    let d = overhang.y.max(stem.y);

    let x_stem_min = (w - stem.x) * 0.5;
    let x_stem_max = x_stem_min + stem.x;
    let y_stem_min = (d - stem.y) * 0.5;
    let y_stem_max = y_stem_min + stem.y;
    let z_stem_min = 0.0f32;
    let z_stem_max = stem.z;

    let x_over_min = (w - overhang.x) * 0.5;
    let x_over_max = x_over_min + overhang.x;
    let y_over_min = (d - overhang.y) * 0.5;
    let y_over_max = y_over_min + overhang.y;
    let z_over_min = stem.z;
    let z_over_max = stem.z + overhang.z;

    let mut triangles = Vec::new();

    // 1. Bottom of the stem (at z = 0, facing -Z)
    add_quad_down(
        &mut triangles,
        x_stem_min,
        x_stem_max,
        y_stem_min,
        y_stem_max,
        z_stem_min,
    );

    // 2. Stem vertical walls (from z = 0 to stem.z)
    add_quad_front(
        &mut triangles,
        x_stem_min,
        x_stem_max,
        z_stem_min,
        z_stem_max,
        y_stem_min,
    );
    add_quad_back(
        &mut triangles,
        x_stem_min,
        x_stem_max,
        z_stem_min,
        z_stem_max,
        y_stem_max,
    );
    add_quad_left(
        &mut triangles,
        y_stem_min,
        y_stem_max,
        z_stem_min,
        z_stem_max,
        x_stem_min,
    );
    add_quad_right(
        &mut triangles,
        y_stem_min,
        y_stem_max,
        z_stem_min,
        z_stem_max,
        x_stem_max,
    );

    // 3. Overhang bottom wings / ceiling (at z = stem.z, facing -Z)
    // Left wing
    if x_stem_min > x_over_min {
        add_quad_down(
            &mut triangles,
            x_over_min,
            x_stem_min,
            y_over_min,
            y_over_max,
            z_over_min,
        );
    }
    // Right wing
    if x_over_max > x_stem_max {
        add_quad_down(
            &mut triangles,
            x_stem_max,
            x_over_max,
            y_over_min,
            y_over_max,
            z_over_min,
        );
    }
    // Front strip between stem x bounds
    if y_stem_min > y_over_min {
        add_quad_down(
            &mut triangles,
            x_stem_min.max(x_over_min),
            x_stem_max.min(x_over_max),
            y_over_min,
            y_stem_min,
            z_over_min,
        );
    }
    // Back strip between stem x bounds
    if y_over_max > y_stem_max {
        add_quad_down(
            &mut triangles,
            x_stem_min.max(x_over_min),
            x_stem_max.min(x_over_max),
            y_stem_max,
            y_over_max,
            z_over_min,
        );
    }

    // Stem upward shoulders (if stem is larger than overhang in any dimension)
    if x_over_min > x_stem_min {
        add_quad_up(
            &mut triangles,
            x_stem_min,
            x_over_min,
            y_stem_min,
            y_stem_max,
            z_over_min,
        );
    }
    if x_stem_max > x_over_max {
        add_quad_up(
            &mut triangles,
            x_over_max,
            x_stem_max,
            y_stem_min,
            y_stem_max,
            z_over_min,
        );
    }
    if y_over_min > y_stem_min {
        add_quad_up(
            &mut triangles,
            x_stem_min,
            x_stem_max,
            y_stem_min,
            y_over_min,
            z_over_min,
        );
    }
    if y_stem_max > y_over_max {
        add_quad_up(
            &mut triangles,
            x_stem_min,
            x_stem_max,
            y_over_max,
            y_stem_max,
            z_over_min,
        );
    }

    // 4. Overhang vertical walls and top face partitioned along stem intervals to maintain watertight manifold
    let mut x_splits = vec![x_over_min, x_over_max];
    if x_stem_min > x_over_min && x_stem_min < x_over_max {
        x_splits.push(x_stem_min);
    }
    if x_stem_max > x_over_min && x_stem_max < x_over_max {
        x_splits.push(x_stem_max);
    }
    x_splits.sort_by(|a, b| a.partial_cmp(b).unwrap());
    x_splits.dedup();

    let mut y_splits = vec![y_over_min, y_over_max];
    if y_stem_min > y_over_min && y_stem_min < y_over_max {
        y_splits.push(y_stem_min);
    }
    if y_stem_max > y_over_min && y_stem_max < y_over_max {
        y_splits.push(y_stem_max);
    }
    y_splits.sort_by(|a, b| a.partial_cmp(b).unwrap());
    y_splits.dedup();

    for i in 0..x_splits.len() - 1 {
        add_quad_front(
            &mut triangles,
            x_splits[i],
            x_splits[i + 1],
            z_over_min,
            z_over_max,
            y_over_min,
        );
        add_quad_back(
            &mut triangles,
            x_splits[i],
            x_splits[i + 1],
            z_over_min,
            z_over_max,
            y_over_max,
        );
    }

    for j in 0..y_splits.len() - 1 {
        add_quad_left(
            &mut triangles,
            y_splits[j],
            y_splits[j + 1],
            z_over_min,
            z_over_max,
            x_over_min,
        );
        add_quad_right(
            &mut triangles,
            y_splits[j],
            y_splits[j + 1],
            z_over_min,
            z_over_max,
            x_over_max,
        );
    }

    // 5. Top of the overhang (at z = stem.z + overhang.z, facing +Z)
    for i in 0..x_splits.len() - 1 {
        for j in 0..y_splits.len() - 1 {
            add_quad_up(
                &mut triangles,
                x_splits[i],
                x_splits[i + 1],
                y_splits[j],
                y_splits[j + 1],
                z_over_max,
            );
        }
    }


    Mesh::new(triangles)
}
