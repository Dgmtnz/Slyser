use crate::mesh::{Mesh, Triangle};
use glam::Vec3;
use std::fs::File;
use std::io::{BufReader, BufWriter, Error, ErrorKind, Read, Write};
use std::path::Path;

const BINARY_HEADER_SIZE: usize = 80;
const TRIANGLE_RECORD_SIZE: usize = 50;

/// Reads an STL mesh from a reader, automatically detecting whether it is binary or ASCII.
pub fn read_stl<R: Read>(reader: &mut R) -> Result<Mesh, Error> {
    let mut buffer = Vec::new();
    reader.read_to_end(&mut buffer)?;

    if buffer.is_empty() {
        return Err(Error::new(
            ErrorKind::UnexpectedEof,
            "Cannot read STL from empty stream",
        ));
    }

    // Determine format: check binary heuristic first
    if buffer.len() >= BINARY_HEADER_SIZE + 4 {
        let count_bytes: [u8; 4] = buffer[80..84]
            .try_into()
            .map_err(|e| Error::new(ErrorKind::InvalidData, e))?;
        let tri_count = u32::from_le_bytes(count_bytes) as usize;
        let expected_binary_len = BINARY_HEADER_SIZE + 4 + tri_count * TRIANGLE_RECORD_SIZE;

        // Exact binary length match is the most definitive indicator
        if buffer.len() == expected_binary_len {
            return parse_stl_binary(&buffer[BINARY_HEADER_SIZE + 4..], tri_count);
        }

        // If buffer does not start with "solid", or has trailing padding but valid binary length
        let starts_with_solid = starts_with_case_insensitive_solid(&buffer);
        if !starts_with_solid && buffer.len() >= expected_binary_len && tri_count > 0 {
            return parse_stl_binary(&buffer[BINARY_HEADER_SIZE + 4..], tri_count);
        }
    }

    // Try parsing as ASCII STL
    match parse_stl_ascii(&buffer) {
        Ok(mesh) => Ok(mesh),
        Err(ascii_err) => {
            // If ASCII parse fails and file length matched binary with some tolerance, try binary
            if buffer.len() >= BINARY_HEADER_SIZE + 4 {
                let count_bytes: [u8; 4] = buffer[80..84]
                    .try_into()
                    .map_err(|e| Error::new(ErrorKind::InvalidData, e))?;
                let tri_count = u32::from_le_bytes(count_bytes) as usize;
                if buffer.len() >= BINARY_HEADER_SIZE + 4 + tri_count * TRIANGLE_RECORD_SIZE {
                    return parse_stl_binary(&buffer[BINARY_HEADER_SIZE + 4..], tri_count);
                }
            }
            Err(ascii_err)
        }
    }
}

/// Helper to check if byte buffer starts with ASCII "solid" (ignoring leading whitespace)
fn starts_with_case_insensitive_solid(buffer: &[u8]) -> bool {
    let mut i = 0;
    while i < buffer.len() && buffer[i].is_ascii_whitespace() {
        i += 1;
    }
    if i + 5 <= buffer.len() {
        buffer[i..i + 5].eq_ignore_ascii_case(b"solid")
    } else {
        false
    }
}

/// Parses binary triangle records from memory.
fn parse_stl_binary(data: &[u8], tri_count: usize) -> Result<Mesh, Error> {
    if data.len() < tri_count * TRIANGLE_RECORD_SIZE {
        return Err(Error::new(
            ErrorKind::UnexpectedEof,
            format!(
                "Incomplete binary STL: expected {} bytes for {} triangles, got {}",
                tri_count * TRIANGLE_RECORD_SIZE,
                tri_count,
                data.len()
            ),
        ));
    }

    let mut triangles = Vec::with_capacity(tri_count);
    let mut offset = 0;

    for _ in 0..tri_count {
        let nx = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap());
        let ny = f32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap());
        let nz = f32::from_le_bytes(data[offset + 8..offset + 12].try_into().unwrap());

        let v0x = f32::from_le_bytes(data[offset + 12..offset + 16].try_into().unwrap());
        let v0y = f32::from_le_bytes(data[offset + 16..offset + 20].try_into().unwrap());
        let v0z = f32::from_le_bytes(data[offset + 20..offset + 24].try_into().unwrap());

        let v1x = f32::from_le_bytes(data[offset + 24..offset + 28].try_into().unwrap());
        let v1y = f32::from_le_bytes(data[offset + 28..offset + 32].try_into().unwrap());
        let v1z = f32::from_le_bytes(data[offset + 32..offset + 36].try_into().unwrap());

        let v2x = f32::from_le_bytes(data[offset + 36..offset + 40].try_into().unwrap());
        let v2y = f32::from_le_bytes(data[offset + 40..offset + 44].try_into().unwrap());
        let v2z = f32::from_le_bytes(data[offset + 44..offset + 48].try_into().unwrap());

        // offset + 48 .. offset + 50 is attribute byte count (ignored)
        offset += TRIANGLE_RECORD_SIZE;

        let normal = Vec3::new(nx, ny, nz);
        let v0 = Vec3::new(v0x, v0y, v0z);
        let v1 = Vec3::new(v1x, v1y, v1z);
        let v2 = Vec3::new(v2x, v2y, v2z);

        triangles.push(Triangle::with_normal([v0, v1, v2], normal));
    }

    Ok(Mesh::new(triangles))
}

/// Parses ASCII STL from byte buffer.
fn parse_stl_ascii(buffer: &[u8]) -> Result<Mesh, Error> {
    let text = std::str::from_utf8(buffer)
        .map_err(|e| Error::new(ErrorKind::InvalidData, format!("ASCII STL UTF-8 error: {e}")))?;

    let mut tokens = text.split_whitespace();
    let mut triangles = Vec::new();

    while let Some(tok) = tokens.next() {
        if tok.eq_ignore_ascii_case("facet") {
            let next_tok = tokens.next();
            let normal = if let Some(n_label) = next_tok {
                if n_label.eq_ignore_ascii_case("normal") {
                    let nx: f32 = parse_next_f32(&mut tokens)?;
                    let ny: f32 = parse_next_f32(&mut tokens)?;
                    let nz: f32 = parse_next_f32(&mut tokens)?;
                    Vec3::new(nx, ny, nz)
                } else {
                    Vec3::ZERO
                }
            } else {
                Vec3::ZERO
            };

            let mut vertices = [Vec3::ZERO; 3];
            let mut vertex_idx = 0;

            while vertex_idx < 3 {
                match tokens.next() {
                    Some(t) if t.eq_ignore_ascii_case("vertex") => {
                        let vx: f32 = parse_next_f32(&mut tokens)?;
                        let vy: f32 = parse_next_f32(&mut tokens)?;
                        let vz: f32 = parse_next_f32(&mut tokens)?;
                        vertices[vertex_idx] = Vec3::new(vx, vy, vz);
                        vertex_idx += 1;
                    }
                    Some(t) if t.eq_ignore_ascii_case("endfacet") => {
                        break;
                    }
                    Some(_) => continue,
                    None => {
                        return Err(Error::new(
                            ErrorKind::UnexpectedEof,
                            "Premature end of file while reading facet vertices",
                        ));
                    }
                }
            }

            if vertex_idx == 3 {
                triangles.push(Triangle::with_normal(vertices, normal));
            }
        }
    }

    Ok(Mesh::new(triangles))
}

fn parse_next_f32<'a, I: Iterator<Item = &'a str>>(iter: &mut I) -> Result<f32, Error> {
    let val_str = iter.next().ok_or_else(|| {
        Error::new(ErrorKind::UnexpectedEof, "Expected float token in ASCII STL")
    })?;
    val_str.parse::<f32>().map_err(|e| {
        Error::new(
            ErrorKind::InvalidData,
            format!("Invalid float '{val_str}': {e}"),
        )
    })
}

/// Writes a mesh in binary STL format to a writer.
pub fn write_stl_binary<W: Write>(writer: &mut W, mesh: &Mesh) -> Result<(), Error> {
    let mut buf_writer = BufWriter::new(writer);

    // 80-byte header
    let mut header = [0u8; BINARY_HEADER_SIZE];
    let header_text = b"Slyser binary STL export";
    header[..header_text.len()].copy_from_slice(header_text);
    buf_writer.write_all(&header)?;

    // 4-byte triangle count
    let tri_count = mesh.triangles.len() as u32;
    buf_writer.write_all(&tri_count.to_le_bytes())?;

    // 50 bytes per triangle
    for tri in &mesh.triangles {
        // Normal (12 bytes)
        buf_writer.write_all(&tri.normal.x.to_le_bytes())?;
        buf_writer.write_all(&tri.normal.y.to_le_bytes())?;
        buf_writer.write_all(&tri.normal.z.to_le_bytes())?;

        // Vertices (36 bytes)
        for v in &tri.vertices {
            buf_writer.write_all(&v.x.to_le_bytes())?;
            buf_writer.write_all(&v.y.to_le_bytes())?;
            buf_writer.write_all(&v.z.to_le_bytes())?;
        }

        // Attribute byte count (2 bytes)
        buf_writer.write_all(&0u16.to_le_bytes())?;
    }

    buf_writer.flush()
}

/// Writes a mesh in ASCII STL format to a writer.
pub fn write_stl_ascii<W: Write>(writer: &mut W, mesh: &Mesh, name: &str) -> Result<(), Error> {
    let mut buf_writer = BufWriter::new(writer);

    writeln!(buf_writer, "solid {name}")?;
    for tri in &mesh.triangles {
        writeln!(
            buf_writer,
            "  facet normal {} {} {}",
            tri.normal.x, tri.normal.y, tri.normal.z
        )?;
        writeln!(buf_writer, "    outer loop")?;
        for v in &tri.vertices {
            writeln!(buf_writer, "      vertex {} {} {}", v.x, v.y, v.z)?;
        }
        writeln!(buf_writer, "    endloop")?;
        writeln!(buf_writer, "  endfacet")?;
    }
    writeln!(buf_writer, "endsolid {name}")?;

    buf_writer.flush()
}

/// Reads an STL file from disk.
pub fn read_stl_file(path: &Path) -> Result<Mesh, Error> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    read_stl(&mut reader)
}

/// Writes a mesh to an STL file on disk in binary format.
pub fn write_stl_file(path: &Path, mesh: &Mesh) -> Result<(), Error> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    write_stl_binary(&mut writer, mesh)
}
