pub mod bvh;
pub mod cut;
pub mod mesh;
pub mod orient;
pub mod pins;
pub mod optimizer;
pub mod primitives;
pub mod stl;
pub mod support;

pub use bvh::{RayHit, BVH};
pub use cut::{compute_plane_basis, slice_mesh_by_plane, CutResult};
pub use mesh::{Mesh, Plane, Triangle, Vertex, AABB};
pub use optimizer::{
    optimize_partitions, CandidatePlaneGenerator, OptimizationResult, OptimizerConfig,
    PartitionPiece,
};
pub use orient::{
    compute_bed_contact_area, extract_candidate_orientations, find_optimal_orientation,
    is_orientation_stable, orient_mesh_to_bed, rotation_to_down, OptimalOrientation,
    OrientationCandidate,
};
pub use pins::{
    apply_female_sockets_and_generate_pins, find_pin_placements, generate_square_pin, PinConfig,
    PinPlacement,
};
pub use primitives::{create_box, create_cube, create_cube_centered, create_t_bracket};
pub use stl::{read_stl, read_stl_file, write_stl_ascii, write_stl_binary, write_stl_file};
pub use support::{analyze_support, is_overhang, SupportConfig, SupportMetrics};


