use glam::{Affine3A, Mat3, Quat, Vec3};
use rayon::prelude::*;

/// 3D vertex position.
pub type Vertex = Vec3;

/// Axis-Aligned Bounding Box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
}

impl AABB {
    /// Creates a new AABB with the specified min and max bounds.
    #[inline]
    pub const fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Creates an empty (inverted) AABB.
    #[inline]
    pub fn empty() -> Self {
        Self {
            min: Vec3::splat(f32::INFINITY),
            max: Vec3::splat(f32::NEG_INFINITY),
        }
    }

    /// Computes the AABB enclosing an array/slice of points.
    pub fn from_points(points: &[Vec3]) -> Self {
        let mut aabb = Self::empty();
        for &p in points {
            aabb.extend(p);
        }
        aabb
    }

    /// Extends the bounding box to include a point.
    #[inline]
    pub fn extend(&mut self, point: Vec3) {
        self.min = self.min.min(point);
        self.max = self.max.max(point);
    }

    /// Extends the bounding box to enclose another AABB.
    #[inline]
    pub fn extend_aabb(&mut self, other: &Self) {
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }

    /// Returns the dimensional size (width, depth, height) of the bounding box.
    #[inline]
    pub fn size(&self) -> Vec3 {
        if self.is_empty() {
            Vec3::ZERO
        } else {
            (self.max - self.min).max(Vec3::ZERO)
        }
    }

    /// Returns the center point of the bounding box.
    #[inline]
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Checks if the bounding box has valid extents.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// Checks if a point is contained within the bounding box (inclusive).
    #[inline]
    pub fn contains(&self, point: Vec3) -> bool {
        point.cmpge(self.min).all() && point.cmple(self.max).all()
    }
}

/// A 3D planar surface defined by a point and a unit normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
}

impl Plane {
    /// Creates a new plane with the given point and normal (normalized).
    pub fn new(point: Vec3, normal: Vec3) -> Self {
        Self {
            point,
            normal: normal.normalize_or_zero(),
        }
    }

    /// Creates a plane passing through three non-collinear points.
    pub fn from_points(p0: Vec3, p1: Vec3, p2: Vec3) -> Option<Self> {
        let normal = (p1 - p0).cross(p2 - p0).normalize_or_zero();
        if normal.length_squared() < 1e-6 {
            None
        } else {
            Some(Self { point: p0, normal })
        }
    }

    /// Computes the signed distance from a point to the plane.
    /// Positive if the point lies in the half-space pointed by `normal`.
    #[inline]
    pub fn signed_distance(&self, p: Vec3) -> f32 {
        (p - self.point).dot(self.normal)
    }

    /// Computes the absolute distance from a point to the plane.
    #[inline]
    pub fn distance(&self, p: Vec3) -> f32 {
        self.signed_distance(p).abs()
    }

    /// Projects a point orthogonally onto the plane.
    #[inline]
    pub fn project_point(&self, p: Vec3) -> Vec3 {
        p - self.normal * self.signed_distance(p)
    }

    /// Intersects a ray `(origin + t * dir)` with the plane.
    /// Returns the distance `t >= 0` along the ray if an intersection occurs.
    pub fn intersect_ray(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<f32> {
        let denom = self.normal.dot(ray_dir);
        if denom.abs() < 1e-6 {
            return None; // Ray is parallel to plane
        }
        let t = (self.point - ray_origin).dot(self.normal) / denom;
        if t >= 0.0 {
            Some(t)
        } else {
            None
        }
    }
}

/// A 3D triangle defined by 3 vertices and a surface normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle {
    pub vertices: [Vec3; 3],
    pub normal: Vec3,
}

impl Triangle {
    /// Creates a new triangle from 3 vertices, computing the normal automatically.
    pub fn new(v0: Vec3, v1: Vec3, v2: Vec3) -> Self {
        let mut tri = Self {
            vertices: [v0, v1, v2],
            normal: Vec3::ZERO,
        };
        tri.normal = tri.computed_normal();
        tri
    }

    /// Creates a triangle with an explicit normal. Recomputes if the provided normal is zero.
    pub fn with_normal(vertices: [Vec3; 3], normal: Vec3) -> Self {
        let mut tri = Self { vertices, normal };
        if tri.normal.length_squared() < 1e-8 {
            tri.normal = tri.computed_normal();
        } else {
            tri.normal = tri.normal.normalize();
        }
        tri
    }

    /// Computes the surface normal using counter-clockwise vertex winding.
    #[inline]
    pub fn computed_normal(&self) -> Vec3 {
        let edge1 = self.vertices[1] - self.vertices[0];
        let edge2 = self.vertices[2] - self.vertices[0];
        edge1.cross(edge2).normalize_or_zero()
    }

    /// Computes the surface area of the triangle.
    #[inline]
    pub fn area(&self) -> f32 {
        let edge1 = self.vertices[1] - self.vertices[0];
        let edge2 = self.vertices[2] - self.vertices[0];
        edge1.cross(edge2).length() * 0.5
    }

    /// Alias for area calculation.
    #[inline]
    pub fn computed_area(&self) -> f32 {
        self.area()
    }

    /// Computes the arithmetic centroid of the triangle vertices.
    #[inline]
    pub fn center(&self) -> Vec3 {
        (self.vertices[0] + self.vertices[1] + self.vertices[2]) / 3.0
    }

    /// Computes the AABB of the triangle.
    pub fn aabb(&self) -> AABB {
        AABB::from_points(&self.vertices)
    }

    /// Computes the signed volume contribution of this triangle to the origin.
    #[inline]
    pub fn signed_volume_to_origin(&self) -> f32 {
        self.vertices[0].dot(self.vertices[1].cross(self.vertices[2])) / 6.0
    }
}

/// Triangular surface mesh.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Mesh {
    pub triangles: Vec<Triangle>,
}

impl Mesh {
    /// Creates a mesh from a list of triangles.
    pub fn new(triangles: Vec<Triangle>) -> Self {
        Self { triangles }
    }

    /// Creates an empty mesh.
    pub fn empty() -> Self {
        Self {
            triangles: Vec::new(),
        }
    }

    /// Returns the number of triangles in the mesh.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Checks if the mesh contains zero triangles.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    /// Computes the tight axis-aligned bounding box enclosing all triangles.
    pub fn aabb(&self) -> AABB {
        if self.triangles.len() > 2048 {
            self.triangles
                .par_iter()
                .fold(AABB::empty, |mut aabb, tri| {
                    aabb.extend(tri.vertices[0]);
                    aabb.extend(tri.vertices[1]);
                    aabb.extend(tri.vertices[2]);
                    aabb
                })
                .reduce(AABB::empty, |mut a, b| {
                    a.extend_aabb(&b);
                    a
                })
        } else {
            let mut aabb = AABB::empty();
            for tri in &self.triangles {
                aabb.extend(tri.vertices[0]);
                aabb.extend(tri.vertices[1]);
                aabb.extend(tri.vertices[2]);
            }
            aabb
        }
    }

    /// Computes the total surface area by summing individual triangle areas.
    pub fn total_surface_area(&self) -> f32 {
        if self.triangles.len() > 2048 {
            self.triangles.par_iter().map(|tri| tri.area()).sum()
        } else {
            self.triangles.iter().map(|tri| tri.area()).sum()
        }
    }

    /// Computes the signed volume of a closed watertight mesh using the divergence theorem.
    pub fn signed_volume(&self) -> f32 {
        if self.triangles.len() > 2048 {
            self.triangles
                .par_iter()
                .map(|tri| tri.signed_volume_to_origin())
                .sum()
        } else {
            self.triangles
                .iter()
                .map(|tri| tri.signed_volume_to_origin())
                .sum()
        }
    }

    /// Computes the total absolute volume of a closed watertight mesh.
    pub fn total_volume(&self) -> f32 {
        self.signed_volume().abs()
    }

    /// Computes the center of mass (centroid of solid volume) for a closed mesh.
    /// Falls back to area-weighted surface centroid if volume is near zero.
    pub fn center_of_mass(&self) -> Vec3 {
        let mut total_vol = 0.0f32;
        let mut weighted_center = Vec3::ZERO;

        for tri in &self.triangles {
            let vol = tri.signed_volume_to_origin();
            let tet_center = (tri.vertices[0] + tri.vertices[1] + tri.vertices[2]) * 0.25;
            weighted_center += tet_center * vol;
            total_vol += vol;
        }

        if total_vol.abs() > 1e-6 {
            weighted_center / total_vol
        } else {
            // Fallback to surface area-weighted center
            let mut total_area = 0.0f32;
            let mut area_center = Vec3::ZERO;
            for tri in &self.triangles {
                let a = tri.area();
                area_center += tri.center() * a;
                total_area += a;
            }
            if total_area > 1e-6 {
                area_center / total_area
            } else {
                self.aabb().center()
            }
        }
    }

    /// Translates all vertices of the mesh in place.
    pub fn translate(&mut self, translation: Vec3) {
        if self.triangles.len() > 2048 {
            self.triangles.par_iter_mut().for_each(|tri| {
                tri.vertices[0] += translation;
                tri.vertices[1] += translation;
                tri.vertices[2] += translation;
            });
        } else {
            for tri in &mut self.triangles {
                tri.vertices[0] += translation;
                tri.vertices[1] += translation;
                tri.vertices[2] += translation;
            }
        }
    }

    /// Returns a new translated mesh.
    pub fn translated(&self, translation: Vec3) -> Self {
        let mut copy = self.clone();
        copy.translate(translation);
        copy
    }

    /// Rotates the mesh around the origin using a quaternion.
    pub fn rotate(&mut self, rotation: Quat) {
        let rot_mat = Mat3::from_quat(rotation);
        self.transform_mat3(&rot_mat);
    }

    /// Returns a new rotated mesh.
    pub fn rotated(&self, rotation: Quat) -> Self {
        let mut copy = self.clone();
        copy.rotate(rotation);
        copy
    }

    /// Transforms the mesh using a 3x3 linear matrix (rotates vertices and updates normals).
    pub fn transform_mat3(&mut self, mat: &Mat3) {
        let normal_mat = mat.inverse().transpose();
        if self.triangles.len() > 2048 {
            self.triangles.par_iter_mut().for_each(|tri| {
                tri.vertices[0] = *mat * tri.vertices[0];
                tri.vertices[1] = *mat * tri.vertices[1];
                tri.vertices[2] = *mat * tri.vertices[2];
                tri.normal = (normal_mat * tri.normal).normalize_or_zero();
            });
        } else {
            for tri in &mut self.triangles {
                tri.vertices[0] = *mat * tri.vertices[0];
                tri.vertices[1] = *mat * tri.vertices[1];
                tri.vertices[2] = *mat * tri.vertices[2];
                tri.normal = (normal_mat * tri.normal).normalize_or_zero();
            }
        }
    }

    /// Transforms the mesh using an Affine3A transform (rotation, scale, translation).
    pub fn transform(&mut self, affine: Affine3A) {
        let linear = Mat3::from_cols(
            affine.matrix3.x_axis.into(),
            affine.matrix3.y_axis.into(),
            affine.matrix3.z_axis.into(),
        );
        let normal_mat = linear.inverse().transpose();

        if self.triangles.len() > 2048 {
            self.triangles.par_iter_mut().for_each(|tri| {
                tri.vertices[0] = affine.transform_point3(tri.vertices[0]);
                tri.vertices[1] = affine.transform_point3(tri.vertices[1]);
                tri.vertices[2] = affine.transform_point3(tri.vertices[2]);
                tri.normal = (normal_mat * tri.normal).normalize_or_zero();
            });
        } else {
            for tri in &mut self.triangles {
                tri.vertices[0] = affine.transform_point3(tri.vertices[0]);
                tri.vertices[1] = affine.transform_point3(tri.vertices[1]);
                tri.vertices[2] = affine.transform_point3(tri.vertices[2]);
                tri.normal = (normal_mat * tri.normal).normalize_or_zero();
            }
        }
    }

    /// Returns a new transformed mesh.
    pub fn transformed(&self, affine: Affine3A) -> Self {
        let mut copy = self.clone();
        copy.transform(affine);
        copy
    }

    /// Checks whether the mesh is a closed watertight 2-manifold without open boundary edges.
    pub fn is_closed(&self) -> bool {
        if self.triangles.is_empty() {
            return false;
        }

        type PointKey = (i64, i64, i64);
        type DirectedEdgeKey = (PointKey, PointKey);

        fn quantize(v: Vec3) -> PointKey {
            (
                (v.x * 1e4).round() as i64,
                (v.y * 1e4).round() as i64,
                (v.z * 1e4).round() as i64,
            )
        }

        let mut edge_counts: std::collections::HashMap<DirectedEdgeKey, i32> =
            std::collections::HashMap::with_capacity(self.triangles.len() * 3);


        for tri in &self.triangles {
            let k0 = quantize(tri.vertices[0]);
            let k1 = quantize(tri.vertices[1]);
            let k2 = quantize(tri.vertices[2]);

            if k0 != k1 {
                *edge_counts.entry((k0, k1)).or_insert(0) += 1;
            }
            if k1 != k2 {
                *edge_counts.entry((k1, k2)).or_insert(0) += 1;
            }
            if k2 != k0 {
                *edge_counts.entry((k2, k0)).or_insert(0) += 1;
            }
        }

        for (&(a, b), &count) in &edge_counts {
            let twin_count = edge_counts.get(&(b, a)).copied().unwrap_or(0);
            if count != twin_count {
                return false;
            }
        }

        true
    }

    /// Alias for is_closed.
    #[inline]
    pub fn is_watertight(&self) -> bool {
        self.is_closed()
    }

    /// Partitions the mesh into separate connected surface components.
    /// Triangles sharing vertices (quantized with 1e-4 mm tolerance) are grouped together.
    /// Components are returned sorted in descending order of total volume.
    pub fn connected_components(&self) -> Vec<Mesh> {
        if self.triangles.is_empty() {
            return Vec::new();
        }

        let n = self.triangles.len();
        let mut dsu = DisjointSet::new(n);
        let mut vertex_to_tri: std::collections::HashMap<(i64, i64, i64), usize> =
            std::collections::HashMap::with_capacity(n * 3);

        let quantize = |v: Vec3| -> (i64, i64, i64) {
            (
                (v.x * 1e4).round() as i64,
                (v.y * 1e4).round() as i64,
                (v.z * 1e4).round() as i64,
            )
        };

        for (i, tri) in self.triangles.iter().enumerate() {
            for v in &tri.vertices {
                let key = quantize(*v);
                match vertex_to_tri.entry(key) {
                    std::collections::hash_map::Entry::Occupied(entry) => {
                        dsu.union(i, *entry.get());
                    }
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        entry.insert(i);
                    }
                }
            }
        }

        let mut groups: std::collections::HashMap<usize, Vec<Triangle>> =
            std::collections::HashMap::new();
        for (i, tri) in self.triangles.iter().enumerate() {
            let root = dsu.find(i);
            groups.entry(root).or_default().push(*tri);
        }

        let mut components: Vec<Mesh> = groups.into_values().map(Mesh::new).collect();
        components.sort_by(|a, b| {
            b.total_volume()
                .total_cmp(&a.total_volume())
                .then_with(|| b.triangles.len().cmp(&a.triangles.len()))
        });

        components
    }

    /// Returns the number of disconnected components in the mesh.
    pub fn connected_components_count(&self) -> usize {
        if self.triangles.is_empty() {
            return 0;
        }

        let n = self.triangles.len();
        let mut dsu = DisjointSet::new(n);
        let mut vertex_to_tri: std::collections::HashMap<(i64, i64, i64), usize> =
            std::collections::HashMap::with_capacity(n * 3);

        let quantize = |v: Vec3| -> (i64, i64, i64) {
            (
                (v.x * 1e4).round() as i64,
                (v.y * 1e4).round() as i64,
                (v.z * 1e4).round() as i64,
            )
        };

        for (i, tri) in self.triangles.iter().enumerate() {
            for v in &tri.vertices {
                let key = quantize(*v);
                match vertex_to_tri.entry(key) {
                    std::collections::hash_map::Entry::Occupied(entry) => {
                        dsu.union(i, *entry.get());
                    }
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        entry.insert(i);
                    }
                }
            }
        }

        let mut roots = std::collections::HashSet::new();
        for i in 0..n {
            roots.insert(dsu.find(i));
        }
        roots.len()
    }

    /// Returns true if the mesh is a single contiguous solid body (exactly 1 connected component).
    #[inline]
    pub fn is_contiguous(&self) -> bool {
        self.connected_components_count() == 1
    }

    /// Returns the largest connected component by volume, or empty mesh if empty.
    pub fn largest_connected_component(&self) -> Mesh {
        let mut comps = self.connected_components();
        if comps.is_empty() {
            Mesh::empty()
        } else {
            comps.swap_remove(0)
        }
    }
}

struct DisjointSet {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl DisjointSet {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        let mut root = x;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        while self.parent[x] != root {
            let next = self.parent[x];
            self.parent[x] = root;
            x = next;
        }
        root
    }

    fn union(&mut self, x: usize, y: usize) {
        let mut root_x = self.find(x);
        let mut root_y = self.find(y);
        if root_x != root_y {
            if self.rank[root_x] < self.rank[root_y] {
                std::mem::swap(&mut root_x, &mut root_y);
            }
            self.parent[root_y] = root_x;
            if self.rank[root_x] == self.rank[root_y] {
                self.rank[root_x] += 1;
            }
        }
    }
}

