use crate::mesh::{Triangle, AABB};
use glam::Vec3;

/// Result of a ray-surface intersection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// Distance `t` along the ray from the ray origin.
    pub t: f32,
    /// 3D coordinate of the intersection point.
    pub hit_point: Vec3,
    /// Surface normal vector at the intersection point.
    pub hit_normal: Vec3,
    /// Index of the intersected triangle in the source mesh.
    pub triangle_index: usize,
}

impl RayHit {
    /// Creates a new ray intersection record.
    #[inline]
    pub const fn new(t: f32, hit_point: Vec3, hit_normal: Vec3, triangle_index: usize) -> Self {
        Self {
            t,
            hit_point,
            hit_normal,
            triangle_index,
        }
    }

    /// Alias returning the hit point coordinate.
    #[inline]
    pub fn point(&self) -> Vec3 {
        self.hit_point
    }

    /// Alias returning the hit normal vector.
    #[inline]
    pub fn normal(&self) -> Vec3 {
        self.hit_normal
    }
}

/// A node in the flattened BVH tree.
#[derive(Debug, Clone, PartialEq)]
pub struct BVHNode {
    /// Tight bounding box enclosing all primitives in this subtree.
    pub aabb: AABB,
    /// Index of left child node (0 if leaf).
    pub left_child: u32,
    /// Index of right child node (0 if leaf).
    pub right_child: u32,
    /// Starting offset into the primitive index buffer.
    pub prim_start: u32,
    /// Number of primitives in this leaf node (0 for internal nodes).
    pub prim_count: u32,
}

impl BVHNode {
    /// Returns true if this node is a leaf containing primitives.
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.prim_count > 0
    }
}

/// Robust slab-based ray-AABB intersection test.
///
/// Returns `Some(distance_to_entry)` if the ray intersects the bounding box
/// within `[0.0, t_max]`, or `None` if it misses. Handles rays parallel to axes correctly.
#[inline]
pub fn aabb_intersect_distance(
    aabb: &AABB,
    origin: Vec3,
    inv_dir: Vec3,
    t_max: f32,
) -> Option<f32> {
    let mut tmin = 0.0f32;
    let mut tmax = t_max;

    // X axis
    if inv_dir.x.is_finite() {
        let t1 = (aabb.min.x - origin.x) * inv_dir.x;
        let t2 = (aabb.max.x - origin.x) * inv_dir.x;
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));
        if tmin > tmax {
            return None;
        }
    } else if origin.x < aabb.min.x || origin.x > aabb.max.x {
        return None;
    }

    // Y axis
    if inv_dir.y.is_finite() {
        let t1 = (aabb.min.y - origin.y) * inv_dir.y;
        let t2 = (aabb.max.y - origin.y) * inv_dir.y;
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));
        if tmin > tmax {
            return None;
        }
    } else if origin.y < aabb.min.y || origin.y > aabb.max.y {
        return None;
    }

    // Z axis
    if inv_dir.z.is_finite() {
        let t1 = (aabb.min.z - origin.z) * inv_dir.z;
        let t2 = (aabb.max.z - origin.z) * inv_dir.z;
        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));
        if tmin > tmax {
            return None;
        }
    } else if origin.z < aabb.min.z || origin.z > aabb.max.z {
        return None;
    }

    Some(tmin)
}

/// Möller-Trumbore ray-triangle intersection test.
///
/// Evaluates whether ray `R(t) = origin + t * dir` intersects `tri`
/// within distance bounds `[t_min, t_max]`.
#[inline]
pub fn intersect_triangle(
    tri: &Triangle,
    tri_idx: usize,
    origin: Vec3,
    dir: Vec3,
    t_min: f32,
    t_max: f32,
) -> Option<RayHit> {
    let edge1 = tri.vertices[1] - tri.vertices[0];
    let edge2 = tri.vertices[2] - tri.vertices[0];
    let pvec = dir.cross(edge2);
    let det = edge1.dot(pvec);

    if det.abs() < 1e-8 {
        return None; // Ray is parallel to triangle plane
    }
    let inv_det = 1.0 / det;

    let tvec = origin - tri.vertices[0];
    let u = tvec.dot(pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let qvec = tvec.cross(edge1);
    let v = dir.dot(qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = edge2.dot(qvec) * inv_det;
    if t < t_min || t > t_max {
        return None;
    }

    let hit_point = origin + dir * t;
    Some(RayHit {
        t,
        hit_point,
        hit_normal: tri.normal,
        triangle_index: tri_idx,
    })
}

/// Fast Bounding Volume Hierarchy for spatial raycasting queries.
#[derive(Debug, Clone, PartialEq)]
pub struct BVH {
    /// Linear buffer of BVH nodes.
    pub nodes: Vec<BVHNode>,
    /// Permuted indices referring to the source triangles.
    pub prim_indices: Vec<usize>,
    /// Stored copy of mesh triangles.
    pub triangles: Vec<Triangle>,
}

impl BVH {
    /// Builds a balanced BVH from an array of triangles using spatial median splitting.
    pub fn build(triangles: &[Triangle]) -> Self {
        if triangles.is_empty() {
            return Self {
                nodes: Vec::new(),
                prim_indices: Vec::new(),
                triangles: Vec::new(),
            };
        }

        let mut prim_indices: Vec<usize> = (0..triangles.len()).collect();
        let centroids: Vec<Vec3> = triangles.iter().map(|t| t.center()).collect();
        let aabbs: Vec<AABB> = triangles.iter().map(|t| t.aabb()).collect();

        let mut nodes = Vec::with_capacity(triangles.len().max(1) * 2);
        let n = triangles.len();
        build_recursive(
            &mut nodes,
            &mut prim_indices,
            &centroids,
            &aabbs,
            0,
            n,
        );

        Self {
            nodes,
            prim_indices,
            triangles: triangles.to_vec(),
        }
    }

    /// Tests a ray against the scene triangles, returning the closest hit.
    pub fn intersect_ray(&self, origin: Vec3, dir: Vec3) -> Option<RayHit> {
        self.intersect_ray_with_options(origin, dir, 1e-5, f32::INFINITY, None)
    }

    /// Tests a ray with custom distance limits and an optional triangle to ignore (avoid self-hits).
    pub fn intersect_ray_with_options(
        &self,
        origin: Vec3,
        dir: Vec3,
        t_min: f32,
        t_max: f32,
        ignore_triangle: Option<usize>,
    ) -> Option<RayHit> {
        if self.nodes.is_empty() {
            return None;
        }

        let inv_dir = Vec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);

        // Fixed-size stack for non-recursive tree traversal (depth <= 64)
        let mut stack = [(0u32, 0.0f32); 64];
        let mut stack_ptr = 0;

        if let Some(t) = aabb_intersect_distance(&self.nodes[0].aabb, origin, inv_dir, t_max) {
            stack[stack_ptr] = (0, t);
            stack_ptr += 1;
        } else {
            return None;
        }

        let mut closest_hit: Option<RayHit> = None;
        let mut current_t_max = t_max;

        while stack_ptr > 0 {
            stack_ptr -= 1;
            let (node_idx, node_t) = stack[stack_ptr];

            if node_t > current_t_max {
                continue;
            }

            let node = &self.nodes[node_idx as usize];

            if node.is_leaf() {
                let start = node.prim_start as usize;
                let end = start + node.prim_count as usize;
                for &tri_idx in &self.prim_indices[start..end] {
                    if Some(tri_idx) == ignore_triangle {
                        continue;
                    }
                    let tri = &self.triangles[tri_idx];
                    if let Some(hit) =
                        intersect_triangle(tri, tri_idx, origin, dir, t_min, current_t_max)
                    {
                        current_t_max = hit.t;
                        closest_hit = Some(hit);
                    }
                }
            } else {
                let left_node = &self.nodes[node.left_child as usize];
                let right_node = &self.nodes[node.right_child as usize];

                let hit_left =
                    aabb_intersect_distance(&left_node.aabb, origin, inv_dir, current_t_max);
                let hit_right =
                    aabb_intersect_distance(&right_node.aabb, origin, inv_dir, current_t_max);

                match (hit_left, hit_right) {
                    (Some(tl), Some(tr)) => {
                        // Push farther child first so closer child is evaluated first
                        if tl < tr {
                            stack[stack_ptr] = (node.right_child, tr);
                            stack_ptr += 1;
                            stack[stack_ptr] = (node.left_child, tl);
                            stack_ptr += 1;
                        } else {
                            stack[stack_ptr] = (node.left_child, tl);
                            stack_ptr += 1;
                            stack[stack_ptr] = (node.right_child, tr);
                            stack_ptr += 1;
                        }
                    }
                    (Some(tl), None) => {
                        stack[stack_ptr] = (node.left_child, tl);
                        stack_ptr += 1;
                    }
                    (None, Some(tr)) => {
                        stack[stack_ptr] = (node.right_child, tr);
                        stack_ptr += 1;
                    }
                    (None, None) => {}
                }
            }
        }

        closest_hit
    }
}

/// Recursively builds the BVH using median centroid splitting.
fn build_recursive(
    nodes: &mut Vec<BVHNode>,
    prim_indices: &mut [usize],
    centroids: &[Vec3],
    aabbs: &[AABB],
    start: usize,
    end: usize,
) -> u32 {
    let count = end - start;
    let node_idx = nodes.len() as u32;

    // Allocate node slot
    nodes.push(BVHNode {
        aabb: AABB::empty(),
        left_child: 0,
        right_child: 0,
        prim_start: start as u32,
        prim_count: 0,
    });

    // Enclosing AABB for all primitives in this span
    let mut node_aabb = AABB::empty();
    for &idx in &prim_indices[start..end] {
        node_aabb.extend_aabb(&aabbs[idx]);
    }
    nodes[node_idx as usize].aabb = node_aabb;

    const MAX_LEAF_PRIMS: usize = 4;
    if count <= MAX_LEAF_PRIMS {
        nodes[node_idx as usize].prim_count = count as u32;
        return node_idx;
    }

    // Centroid bounding box
    let mut centroid_aabb = AABB::empty();
    for &idx in &prim_indices[start..end] {
        centroid_aabb.extend(centroids[idx]);
    }
    let extent = centroid_aabb.size();

    // If primitives are coincident or degenerate, create leaf
    if extent.max_element() < 1e-6 {
        nodes[node_idx as usize].prim_count = count as u32;
        return node_idx;
    }

    // Split along the axis with the largest centroid span
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };

    let mid = start + count / 2;
    prim_indices[start..end].select_nth_unstable_by(count / 2, |&a, &b| {
        let ca = centroids[a][axis];
        let cb = centroids[b][axis];
        ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
    });

    let left_child = build_recursive(nodes, prim_indices, centroids, aabbs, start, mid);
    let right_child = build_recursive(nodes, prim_indices, centroids, aabbs, mid, end);

    nodes[node_idx as usize].left_child = left_child;
    nodes[node_idx as usize].right_child = right_child;
    node_idx
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_moller_trumbore_direct_hit() {
        let v0 = Vec3::new(-1.0, -1.0, 5.0);
        let v1 = Vec3::new(1.0, -1.0, 5.0);
        let v2 = Vec3::new(0.0, 1.0, 5.0);
        let tri = Triangle::new(v0, v1, v2);

        let origin = Vec3::new(0.0, 0.0, 0.0);
        let dir = Vec3::new(0.0, 0.0, 1.0);

        let hit = intersect_triangle(&tri, 42, origin, dir, 0.0, 10.0).expect("Should hit");
        assert_eq!(hit.triangle_index, 42);
        assert!((hit.t - 5.0).abs() < 1e-5);
        assert!((hit.hit_point - Vec3::new(0.0, 0.0, 5.0)).length() < 1e-5);
        assert!((hit.hit_normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-5);
    }

    #[test]
    fn test_moller_trumbore_miss() {
        let v0 = Vec3::new(1.0, 1.0, 5.0);
        let v1 = Vec3::new(3.0, 1.0, 5.0);
        let v2 = Vec3::new(2.0, 3.0, 5.0);
        let tri = Triangle::new(v0, v1, v2);

        let origin = Vec3::new(0.0, 0.0, 0.0);
        let dir = Vec3::new(0.0, 0.0, 1.0);

        assert!(intersect_triangle(&tri, 0, origin, dir, 0.0, 10.0).is_none());
    }

    #[test]
    fn test_bvh_closest_hit_selection() {
        // Two parallel triangles at z = 4 and z = 8
        let t1 = Triangle::new(
            Vec3::new(-2.0, -2.0, 8.0),
            Vec3::new(2.0, -2.0, 8.0),
            Vec3::new(0.0, 2.0, 8.0),
        );
        let t2 = Triangle::new(
            Vec3::new(-2.0, -2.0, 4.0),
            Vec3::new(2.0, -2.0, 4.0),
            Vec3::new(0.0, 2.0, 4.0),
        );

        let bvh = BVH::build(&[t1, t2]);
        let hit = bvh
            .intersect_ray(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
            .expect("Should hit closest triangle");

        assert_eq!(hit.triangle_index, 1);
        assert!((hit.t - 4.0).abs() < 1e-5);
    }
}
