//! 3MF export

use crate::core::config::Unit;
use crate::geometry::mesh::Mesh;
use std::io::Write;
use std::path::Path;
use thiserror::Error;
use zip::{write::SimpleFileOptions, ZipArchive, ZipWriter};

#[derive(Error, Debug)]
pub enum ThreemfError {
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Zip error: {0}")]
    ZipError(#[from] zip::result::ZipError),

    #[error("Invalid mesh")]
    InvalidMesh,
}

/// Write mesh to 3MF file (compressed)
pub fn write_3mf(mesh: &Mesh, path: &Path, unit: Unit) -> Result<(), ThreemfError> {
    if mesh.triangles.is_empty() {
        return Err(ThreemfError::InvalidMesh);
    }

    let file = std::fs::File::create(path)?;
    let mut zip = ZipWriter::new(file);

    // Unit multiplier for 3MF (it uses meters)
    let unit_scale = match unit {
        Unit::Millimeters => 0.001, // mm to meters
        Unit::Centimeters => 0.01,
        Unit::Inches => 0.0254,
    };

    // Write [Content_Types].xml
    let content_types = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="xml" ContentType="application/xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#;

    zip.start_file("[Content_Types].xml", SimpleFileOptions::default())?;
    zip.write_all(content_types.as_bytes())?;

    // Write .rels
    let rels = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="r1" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata" Target="/3D/3DModel.model"/>
</Relationships>"#;

    zip.start_file("_rels/.rels", SimpleFileOptions::default())?;
    zip.write_all(rels.as_bytes())?;

    // Write 3DModel.model
    let model_xml = generate_3mf_model(mesh, unit_scale);
    zip.start_file("3D/3DModel.model", SimpleFileOptions::default())?;
    zip.write_all(model_xml.as_bytes())?;

    // Write metadata
    let metadata = generate_metadata();
    zip.start_file("Metadata/metadata.xml", SimpleFileOptions::default())?;
    zip.write_all(metadata.as_bytes())?;

    zip.finish()?;

    Ok(())
}

/// Generate 3MF model XML
fn generate_3mf_model(mesh: &Mesh, scale: f32) -> String {
    let mut xml = String::new();

    xml.push_str(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model xmlns="http://schemas.microsoft.com/3dmanufacturing/2013/01" unit="meter">
  <resources>
"#,
    );

    // Generate mesh resource
    xml.push_str("    <object id=\"1\" type=\"model\">\n");
    xml.push_str("      <mesh>\n");

    // Vertices
    xml.push_str("        <vertices>\n");
    for v in &mesh.vertices {
        xml.push_str(&format!(
            "          <vertex x=\"{:.6}\" y=\"{:.6}\" z=\"{:.6}\"/>\n",
            v.x * scale,
            v.y * scale,
            v.z * scale
        ));
    }
    xml.push_str("        </vertices>\n");

    // Triangles
    xml.push_str("        <triangles>\n");
    for tri in &mesh.triangles {
        xml.push_str(&format!(
            "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
            tri.indices[0], tri.indices[1], tri.indices[2]
        ));
    }
    xml.push_str("        </triangles>\n");

    xml.push_str("      </mesh>\n");
    xml.push_str("    </object>\n");

    xml.push_str("  </resources>\n");
    xml.push_str("  <build>\n");
    xml.push_str("    <item objectid=\"1\"/>\n");
    xml.push_str("  </build>\n");
    xml.push_str("</model>\n");

    xml
}

/// Generate metadata XML
fn generate_metadata() -> String {
    r#"<?xml version="1.0" encoding="UTF-8"?>
<metadata>
  <item name="Generator" value="AutoMold"/>
  <item name="Description" value="Generated mold mesh"/>
</metadata>"#
        .to_string()
}

/// Read 3MF file (simplified)
pub fn read_3mf(path: &Path) -> Result<Mesh, ThreemfError> {
    // Simplified - for now just return error
    // Full implementation would parse the ZIP and extract the mesh
    let file = std::fs::File::open(path)?;
    let _archive = ZipArchive::new(file)?;

    // Extract 3DModel.model and parse
    // For now, return error
    Err(ThreemfError::InvalidMesh)
}
