use clap::{Parser, Subcommand};
use glam::Vec3;
use slyser::cut::slice_mesh_by_plane;
use slyser::mesh::Plane;
use slyser::optimizer::{optimize_partitions, OptimizerConfig};
use slyser::orient::find_optimal_orientation;
use slyser::pins::{apply_female_sockets_and_generate_pins, PinConfig};
use slyser::primitives::{create_cube, create_t_bracket};
use slyser::stl::{read_stl_file, write_stl_file};
use slyser::support::{analyze_support, SupportConfig};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "slyser")]
#[command(author, version, about = "Intelligent STL mesh partitioning tool for support-free 3D printing", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// Inspect an STL mesh (triangle count, bounding box, volume, surface area, center of mass)
    Info {
        /// Path to input STL file
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Analyze mesh overhangs and estimated support material requirements
    Analyze {
        /// Path to input STL file
        #[arg(short, long)]
        input: PathBuf,
        /// Overhang threshold angle in degrees from horizontal (default: 45.0)
        #[arg(short, long, default_value_t = 45.0)]
        angle: f32,
        /// Sampling resolution in quadrature points per mm² (default: 2.0)
        #[arg(short, long, default_value_t = 2.0)]
        resolution: f32,
    },
    /// Generate a synthetic test mesh
    Generate {
        /// Path to output STL file
        #[arg(short, long)]
        output: PathBuf,
        /// Model type: 'cube' or 't_bracket'
        #[arg(short, long, default_value = "cube")]
        model: String,
        /// Dimensions (X Y Z) for cube or stem (e.g. "20.0 20.0 40.0")
        #[arg(long, default_value = "20.0,20.0,20.0")]
        dim: String,
        /// Overhang dimensions (X Y Z) if generating t_bracket
        #[arg(long, default_value = "60.0,20.0,15.0")]
        overhang: String,
    },
    /// Automatically find optimal part orientation to minimize support structures
    Orient {
        /// Path to input STL file
        #[arg(short, long)]
        input: PathBuf,
        /// Path to output reoriented STL file
        #[arg(short, long)]
        output: PathBuf,
        /// Overhang threshold angle in degrees from horizontal (default: 45.0)
        #[arg(short, long, default_value_t = 45.0)]
        angle: f32,
    },
    /// Cut an STL mesh by a plane and export watertight halves
    Cut {
        /// Path to input STL file
        #[arg(short, long)]
        input: PathBuf,
        /// Point on the cut plane: "X,Y,Z"
        #[arg(short, long)]
        point: String,
        /// Normal vector of the cut plane: "NX,NY,NZ"
        #[arg(short, long)]
        normal: String,
        /// Output prefix for exported meshes (<prefix>_pos.stl, <prefix>_neg.stl, <prefix>_pin_1.stl...)
        #[arg(short = 'o', long)]
        output_prefix: String,
        /// Generate female sockets and square alignment pins
        #[arg(long, default_value_t = false)]
        pins: bool,
        /// Square pin width/height in mm (default: 4.0)
        #[arg(long, default_value_t = 4.0)]
        pin_size: f32,
        /// Pin insertion depth per half in mm (default: 5.0)
        #[arg(long, default_value_t = 5.0)]
        pin_depth: f32,
        /// XY clearance per side for FDM fit in mm (default: 0.2)
        #[arg(long, default_value_t = 0.2)]
        pin_clearance: f32,
    },
    /// Partition an STL mesh into optimal support-free pieces with optional alignment pins
    Partition {
        /// Path to input STL file
        #[arg(short, long)]
        input: PathBuf,
        /// Output directory for exported pieces and pins
        #[arg(short, long)]
        output_dir: PathBuf,
        /// Maximum number of pieces (default: 2)
        #[arg(short, long, default_value_t = 2)]
        max_pieces: usize,
        /// Overhang threshold angle in degrees from horizontal (default: 45.0)
        #[arg(short, long, default_value_t = 45.0)]
        angle: f32,
        /// Generate female sockets and square alignment pins
        #[arg(long, default_value_t = false)]
        pins: bool,
        /// Square pin width/height in mm (default: 4.0)
        #[arg(long, default_value_t = 4.0)]
        pin_size: f32,
        /// Pin insertion depth per half in mm (default: 5.0)
        #[arg(long, default_value_t = 5.0)]
        pin_depth: f32,
        /// XY clearance per side for FDM fit in mm (default: 0.2)
        #[arg(long, default_value_t = 0.2)]
        pin_clearance: f32,
        /// Allow cuts that split pieces into disjoint/floating islands
        #[arg(long, default_value_t = false)]
        allow_disjoint: bool,
        /// Minimum pin size in mm for adaptive sizing (default: 1.5)
        #[arg(long, default_value_t = 1.5)]
        pin_min_size: f32,
    },
}


fn parse_vec3(s: &str) -> Result<Vec3, String> {
    let parts: Vec<&str> = s.split([',', ' ']).filter(|s| !s.is_empty()).collect();
    if parts.len() != 3 {
        return Err(format!("Expected 3 float values, got {}: '{}'", parts.len(), s));
    }
    let x = parts[0].parse::<f32>().map_err(|e| e.to_string())?;
    let y = parts[1].parse::<f32>().map_err(|e| e.to_string())?;
    let z = parts[2].parse::<f32>().map_err(|e| e.to_string())?;
    Ok(Vec3::new(x, y, z))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    match cli.command {
        Some(Commands::Info { input }) => {
            println!("Reading STL: {}", input.display());
            let mesh = read_stl_file(&input)?;
            let aabb = mesh.aabb();
            let size = aabb.size();
            let com = mesh.center_of_mass();

            println!("--- Mesh Information ---");
            println!("Triangles:       {}", mesh.triangle_count());
            println!("AABB Min:        [{:.3}, {:.3}, {:.3}]", aabb.min.x, aabb.min.y, aabb.min.z);
            println!("AABB Max:        [{:.3}, {:.3}, {:.3}]", aabb.max.x, aabb.max.y, aabb.max.z);
            println!("Dimensions:      {:.3} x {:.3} x {:.3} mm", size.x, size.y, size.z);
            println!("Total Surface:   {:.3} mm²", mesh.total_surface_area());
            println!("Total Volume:    {:.3} mm³", mesh.total_volume());
            println!("Center of Mass:  [{:.3}, {:.3}, {:.3}]", com.x, com.y, com.z);
            println!("Is Closed:       {}", mesh.is_closed());
            println!("Components:      {}", mesh.connected_components_count());
        }
        Some(Commands::Analyze {
            input,
            angle,
            resolution,
        }) => {
            println!("Reading STL: {}", input.display());
            let mesh = read_stl_file(&input)?;
            let config = SupportConfig {
                overhang_angle_deg: angle,
                sample_resolution: resolution,
            };

            let start = std::time::Instant::now();
            let metrics = analyze_support(&mesh, &config);
            let elapsed = start.elapsed();

            println!("--- Support Analysis ---");
            println!("Threshold Angle:         {:.1}°", config.overhang_angle_deg);
            println!("Sampling Resolution:     {:.2} samples/mm²", config.sample_resolution);
            println!("Overhang Triangles:      {}", metrics.overhang_triangle_count);
            println!("Overhang Area:           {:.3} mm²", metrics.overhang_area);
            println!("Projected Support Area:  {:.3} mm²", metrics.projected_support_area);
            println!("Total Support Volume:    {:.3} mm³", metrics.estimated_support_volume);
            println!("Support on Bed:          {:.3} mm³", metrics.support_on_bed_volume);
            println!("Support on Model:        {:.3} mm³", metrics.support_on_model_volume);
            println!("Analysis Time:           {:.2?}", elapsed);
        }
        Some(Commands::Generate {
            output,
            model,
            dim,
            overhang,
        }) => {
            let dim_vec = parse_vec3(&dim)?;
            let mesh = match model.to_lowercase().as_str() {
                "cube" => {
                    println!("Generating cube of dimensions {:?}", dim_vec);
                    create_cube(dim_vec)
                }
                "t_bracket" | "t-bracket" => {
                    let over_vec = parse_vec3(&overhang)?;
                    println!(
                        "Generating T-bracket with stem {:?} and overhang {:?}",
                        dim_vec, over_vec
                    );
                    create_t_bracket(dim_vec, over_vec)
                }
                other => {
                    eprintln!("Unknown model type '{}'. Supported: cube, t_bracket", other);
                    std::process::exit(1);
                }
            };

            write_stl_file(&output, &mesh)?;
            println!(
                "Successfully exported {} triangles to {}",
                mesh.triangle_count(),
                output.display()
            );
        }
        Some(Commands::Orient {
            input,
            output,
            angle,
        }) => {
            println!("Reading STL: {}", input.display());
            let mesh = read_stl_file(&input)?;
            let config = SupportConfig::with_angle(angle);

            let orig_metrics = analyze_support(&mesh, &config);

            let start = std::time::Instant::now();
            let optimal = find_optimal_orientation(&mesh, &config);
            let elapsed = start.elapsed();

            let orig_vol = orig_metrics.estimated_support_volume;
            let new_vol = optimal.metrics.estimated_support_volume;
            let reduction_pct = if orig_vol > 1e-6 {
                ((orig_vol - new_vol) / orig_vol * 100.0).clamp(0.0, 100.0)
            } else {
                0.0
            };

            println!("--- Orientation Optimization Results ---");
            println!("Original Support Volume:     {:.3} mm³", orig_vol);
            println!("Reoriented Support Volume:   {:.3} mm³", new_vol);
            println!("Support Volume Reduction:    {:.1}%", reduction_pct);
            println!("Bed Contact Area:            {:.3} mm²", optimal.contact_area);
            println!("Mechanically Stable:         {}", optimal.is_stable);
            println!("Optimization Time:           {:.2?}", elapsed);

            write_stl_file(&output, &optimal.mesh)?;
            println!("Saved reoriented mesh to:    {}", output.display());
        }
        Some(Commands::Cut {
            input,
            point,
            normal,
            output_prefix,
            pins,
            pin_size,
            pin_depth,
            pin_clearance,
        }) => {
            println!("Reading STL: {}", input.display());
            let mesh = read_stl_file(&input)?;
            let pt = parse_vec3(&point)?;
            let norm = parse_vec3(&normal)?;
            let plane = Plane::new(pt, norm);

            let start = std::time::Instant::now();
            let result = slice_mesh_by_plane(&mesh, &plane)?;

            let (pos_final, neg_final, pin_meshes) = if pins {
                let config = PinConfig {
                    size: pin_size,
                    depth: pin_depth,
                    clearance: pin_clearance,
                    edge_margin: 2.0,
                    adaptive: true,
                    min_size: 1.5,
                };
                let (pos_sock, neg_sock, pins_list) =
                    apply_female_sockets_and_generate_pins(&result, &config);
                println!("Alignment Pins Generated: {}", pins_list.len());
                (pos_sock, neg_sock, pins_list)
            } else {
                (result.positive_mesh.clone(), result.negative_mesh.clone(), Vec::new())
            };
            let elapsed = start.elapsed();

            let orig_vol = mesh.total_volume();
            let pos_vol = pos_final.total_volume();
            let neg_vol = neg_final.total_volume();
            let sum_vol = pos_vol + neg_vol;
            let diff_pct = if orig_vol > 1e-6 {
                ((sum_vol - orig_vol).abs() / orig_vol) * 100.0
            } else {
                0.0
            };

            println!("--- Planar Mesh Cut Results ---");
            println!("Original Volume:         {:.3} mm³", orig_vol);
            println!("Positive Half Volume:    {:.3} mm³", pos_vol);
            println!("Negative Half Volume:    {:.3} mm³", neg_vol);
            println!("Combined Volume:         {:.3} mm³ (diff: {:.3}%)", sum_vol, diff_pct);
            println!("Cut Cap Area:            {:.3} mm²", result.cut_area);
            println!("Cut Polygon Loops:       {}", result.cut_polygons_2d.len());
            println!("Positive Mesh Closed:    {}", pos_final.is_closed());
            println!("Negative Mesh Closed:    {}", neg_final.is_closed());
            println!("Cutting Time:            {:.2?}", elapsed);

            let pos_path = PathBuf::from(format!("{}_pos.stl", output_prefix));
            let neg_path = PathBuf::from(format!("{}_neg.stl", output_prefix));

            write_stl_file(&pos_path, &pos_final)?;
            println!("Saved positive mesh to:  {}", pos_path.display());

            write_stl_file(&neg_path, &neg_final)?;
            println!("Saved negative mesh to:  {}", neg_path.display());

            for (idx, pin_mesh) in pin_meshes.iter().enumerate() {
                let pin_path = PathBuf::from(format!("{}_pin_{}.stl", output_prefix, idx + 1));
                write_stl_file(&pin_path, pin_mesh)?;
                println!("Saved pin {} mesh to:     {}", idx + 1, pin_path.display());
            }
        }
        Some(Commands::Partition {
            input,
            output_dir,
            max_pieces,
            angle,
            pins,
            pin_size,
            pin_depth,
            pin_clearance,
            allow_disjoint,
            pin_min_size,
        }) => {
            println!("Reading STL: {}", input.display());
            let mesh = read_stl_file(&input)?;

            let pin_config = if pins {
                Some(PinConfig {
                    size: pin_size,
                    depth: pin_depth,
                    clearance: pin_clearance,
                    edge_margin: 2.0,
                    adaptive: true,
                    min_size: pin_min_size,
                })
            } else {
                None
            };

            let config = OptimizerConfig {
                max_pieces,
                support_config: SupportConfig::with_angle(angle),
                pin_config,
                weight_support: 1.0,
                weight_cut_area: 0.001,
                min_piece_volume_ratio: 0.02,
                num_candidate_planes: 20,
                enforce_contiguous: !allow_disjoint,
                pin_min_size,
            };

            let start = std::time::Instant::now();
            let result = optimize_partitions(&mesh, &config);
            let elapsed = start.elapsed();

            std::fs::create_dir_all(&output_dir)?;

            println!("================================================================================");
            println!("                     SLYSER PARETO PARTITION OPTIMIZATION                      ");
            println!("================================================================================");
            println!("Original Support Volume:     {:.3} mm³", result.original_metrics.estimated_support_volume);
            println!("Final Combined Support Vol:  {:.3} mm³", result.total_support_volume);
            println!("Support Volume Reduction:    {:.1}%", result.total_reduction_pct);
            println!("Cuts Made:                   {}", result.cuts_made);
            println!("Pieces Generated:            {}", result.pieces.len());
            let all_contiguous = result.pieces.iter().all(|p| p.mesh.is_contiguous());
            println!("Contiguous Solid:            {}", if all_contiguous { "true" } else { "false" });
            println!("Square Pins Generated:       {}", result.pins.len());
            if let Some(pin_sz) = result.pin_size_used {
                println!("Pin Size:                    {:.1} mm", pin_sz);
            }
            println!("Execution Time:              {:.2?}", elapsed);
            println!("--------------------------------------------------------------------------------");
            println!("Piece Breakdown:");
            for piece in &result.pieces {
                let aabb = piece.mesh.aabb();
                let size = aabb.size();
                println!("  Piece {}:", piece.id);
                println!("    Dimensions:              {:.3} x {:.3} x {:.3} mm", size.x, size.y, size.z);
                println!("    Support Volume:          {:.3} mm³", piece.metrics.estimated_support_volume);
                println!("    Bed Contact Area:        {:.3} mm²", piece.contact_area);
            }
            println!("================================================================================");

            for piece in &result.pieces {
                let piece_path = output_dir.join(format!("piece_{}.stl", piece.id));
                write_stl_file(&piece_path, &piece.mesh)?;
                println!("Saved piece {} to:            {}", piece.id, piece_path.display());
            }

            for (idx, pin_mesh) in result.pins.iter().enumerate() {
                let pin_path = output_dir.join(format!("pin_{}.stl", idx + 1));
                write_stl_file(&pin_path, pin_mesh)?;
                println!("Saved pin {} to:              {}", idx + 1, pin_path.display());
            }
        }
        None => {
            println!("Slyser v0.1.0 - Use --help for usage information");
        }
    }

    Ok(())
}

