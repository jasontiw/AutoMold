# AutoMold

**Herramienta CLI para generación automática de moldes 3D desde modelos STL/OBJ**

[![MIT License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-1.75+-orange.svg)](https://rustup.rs)

---

## ¿Qué es AutoMold?

AutoMold es una herramienta de línea de comandos que **genera automáticamente moldes y contramoldes a partir de modelos 3D**, orientada a escultores y artistas que no tienen experiencia en software CAD.

El usuario solo necesita proporcionar un modelo `STL` u `OBJ`, y AutoMold generará automáticamente:

- Molde de una parte o dos partes
- Cavidad del modelo con tolerancias configurables
- Pines de alineación
- Bloque del molde
- Canales de vertido opcionales

### El objetivo: reducir el diseño de moldes a un solo comando

```bash
automold sculpture.stl
```

---

## Características

### Fase 1 (Actual)

- **Carga de formatos**: STL (binario y ASCII), OBJ
- **Generación de moldes**: Molde de dos partes con cavidad
- **Tolerancias configurables**: Offset de cavidad para diferentes materiales
- **Soporte de unidades**: mm, cm, in
- **Estimación de memoria**: Detecta RAM disponible y warns antes de procesar
- **Exportación**: STL

### Fase 2 (Planificado)

- Orientación automática basada en PCA
- Detección de undercuts
- Generación automática de pines
- Auto-decimación por presupuesto de memoria
- Reparación avanzada de mallas
- Paralelismo con `rayon`

### Fase 3 (Planificado)

- Moldes huecos (`--shell`)
- Canales de vertido automáticos
- Exportación 3MF
- Soporte para modelos grandes (hasta 2M triángulos)
- Fallback SDF con resolución adaptativa

---

## Instalación

### Desde código fuente

```bash
# Clonar el repositorio
git clone https://github.com/tu-usuario/automold.git
cd automold

# Compilar
cargo build --release

# Instalar (opcional)
cargo install --path .
```

### Requisitos

- **Rust** 1.75 o superior
- **Sistema operativo**: Windows, macOS, Linux

---

## Uso

### Comando básico

```bash
automold modelo.stl
```

### Opciones disponibles

| Flag | Valores | Descripción | Default |
|------|---------|-------------|---------|
| `--open-mold` | flag | Genera molde abierto (sin tapa) | false |
| `--split-axis` | X, Y, Z | Eje de división del molde | auto |
| `--wall` | número | Grosor de pared en mm | auto |
| `--tolerance` | número | Offset de cavidad (mm) | 0.2 |
| `--pins` | flag | Genera pines de alineación | false |
| `--pour` | flag | Genera canal de vertido | false |
| `--shell` | flag | Genera molde hueco | false |
| `--unit` | mm, cm, in | Unidad del modelo de entrada | mm |
| `--format` | stl, 3mf | Formato de exportación | stl |
| `--decimate` | 0.0-1.0 | Reduce polígonos antes de procesar | - |
| `--memory-limit` | número | Límite de RAM en MB | auto |
| `--threads` | 1-16 | Número de threads | auto |
| `--output`, `-o` | ruta | Directorio de salida | actual |
| `--verbose`, `-v` | flag | Salida detallada | false |

### Ejemplos

```bash
# Molde básico
automold escultura.stl

# Molde con tolerancia para resina epóxica
automold modelo.stl --tolerance 0.3

# Especificar unidades en pulgadas
automold pieza.obj --unit in

# Molde abierto (sin tapa)
automold forma.stl --open-mold

# Con pines de alineación
automold molde.stl --pins

# Limitar uso de memoria a 1GB
automold grande.stl --memory-limit 1024

# Forzar procesamiento aunque exceda memoria
automold grande.stl --force
```

### Salida de ejemplo

```
AutoMold v0.1.0
Input: sculpture.stl (245,312 triangles, 45.2 x 32.1 x 28.7 mm)

Decisions:
  Split axis:      Z      (auto — least undercuts: 3)
  Wall thickness: 12mm   (auto — based on bounding box)
  Tolerance:       0.2mm (auto — default)
  Pins:            enabled (auto)
  Memory budget:   1,820MB available
  Threads:         4 (auto)

Processing...
  [OK] Mesh loaded
  [OK] Mesh repaired (2 holes filled)
  [OK] Orientation analyzed
  [OK] Mold block generated
  [OK] Boolean operation
  [OK] Split applied
  [OK] Pins generated

Output:
  sculpture_mold_A.stl
  sculpture_mold_B.stl
  metadata.json

Done in 1.4s
```

---

## Arquitectura técnica

### Stack tecnológico

- **Lenguaje**: Rust
- **CLI**: `clap`
- **Geometría**: `nalgebra`, `glam`
- **BVH**: `bvh`
- **Procesamiento de mallas**: `stl_io`, `tobj`, `meshopt`
- **Serialización**: `serde_json`
- **Paralelismo**: `rayon`

### Pipeline geométrico

```
mesh model
     │
     ▼
mesh repair (holes, normales, non-manifold)
     │
     ▼
memory budget estimation
     │
     ▼
decimation pass (opcional)
     │
     ▼
orientation analysis (PCA + undercuts)
     │
     ▼
apply tolerance offset
     │
     ▼
mold block generation
     │
     ▼
boolean operation (mesh - model)
     │
     ▼
mold split
     │
     ▼
pin generation (opcional)
     │
     ▼
export STL/3MF + metadata.json
```

### Estructura del proyecto

```
automold/
├── Cargo.toml
├── src/
│   ├── main.rs
│   ├── cli/
│   │   ├── args.rs
│   │   └── commands.rs
│   ├── core/
│   │   ├── config.rs
│   │   ├── pipeline.rs
│   │   └── context.rs
│   ├── geometry/
│   │   ├── mesh.rs
│   │   ├── bbox.rs
│   │   ├── bvh.rs
│   │   ├── sdf.rs
│   │   └── offset.rs
│   ├── pipeline/
│   │   ├── loader.rs
│   │   ├── repair.rs
│   │   ├── decimate.rs
│   │   ├── orientation.rs
│   │   ├── mold_block.rs
│   │   ├── boolean.rs
│   │   ├── split.rs
│   │   ├── pins.rs
│   │   └── pour.rs
│   ├── export/
│   │   ├── stl.rs
│   │   ├── threemf.rs
│   │   └── metadata.rs
│   └── utils/
│       ├── math.rs
│       ├── logging.rs
│       ├── units.rs
│       └── memory.rs
├── tests/
├── examples/
├── docs/
└── test_data/
```

---

## Testing

### Datos de prueba incluidos

El proyecto incluye modelos de test en `test_data/`:

| Archivo | Triángulos | Descripción |
|---------|------------|-------------|
| `cube_10mm.stl` | 12 | Cubo simple (10mm) - caso ideal |
| `cylinder_30mm.stl` | 96 | Cilindro (15mm radio, 30mm altura) |
| `sphere_20mm.stl` | 512 | Esfera (20mm radio) - geometría curva |

### Ejecutar tests unitarios y de integración

```bash
# Ejecutar todos los tests
cargo test

# Ejecutar un test específico
cargo test test_cube_basic

# Ejecutar con output detallado
cargo test -- --nocapture
```

### Tests disponibles

| Test | Descripción |
|------|-------------|
| `test_cube_basic` | Verifica que cube_10mm genera archivos de salida válidos |
| `test_cylinder_no_crash` | Verifica procesamiento sin errores |
| `test_sphere_curved_geometry` | Verifica manejo de geometría curva |
| `test_metadata_contains_fields` | Verifica que metadata.json tiene los campos requeridos |
| `test_auto_decimate_triggers` | Verifica que auto-decimate se activa con memory limit bajo |
| `test_tolerance_config` | Verifica que la tolerancia configurada se respeta |

### Probar la app manualmente

```bash
# Desde el directorio raíz del proyecto

# Compilar en modo debug
cargo build

# Ejecutar con uno de los modelos de prueba
cargo run -- test_data/cube_10mm.stl

# Especificar directorio de salida
cargo run -- test_data/sphere_20mm.stl --output test_output

# Con tolerancia personalizada
cargo run -- test_data/cylinder_30mm.stl --tolerance 0.5

# Verbose mode para ver más detalles
cargo run -- test_data/cube_10mm.stl --verbose
```

### Estructura de directorios de test

```
test_data/                    # Modelos de entrada para pruebas
├── cube_10mm.stl
├── cylinder_30mm.stl
├── sphere_20mm.stl
├── cube_10mm_mold_A.stl     # Moldes de referencia (Phase 0)
├── cube_10mm_mold_B.stl
├── cylinder_30mm_mold_A.stl
├── cylinder_30mm_mold_B.stl
├── sphere_20mm_mold_A.stl
├── sphere_20mm_mold_B.stl
└── metadata.json

test_output/                  # Resultados generados por tests
├── cube_10mm_mold_A.stl
├── cube_10mm_mold_B.stl
├── metadata.json
└── ...

tests/
└── integration_test.rs       # Suite de tests de integración

examples/
└── generate_test_data.rs    # Generador de modelos de prueba
```

### Generar nuevos datos de prueba

```bash
# Generar modelos de prueba adicionales
cargo run --example generate_test_data
```

---

## Gestión de errores

### Códigos de salida

| Código | Causa |
|--------|-------|
| 0 | Éxito |
| 1 | Archivo no encontrado |
| 2 | Formato no soportado |
| 3 | Malla irrecuperable |
| 4 | Boolean falló |
| 5 | Argumento inválido |
| 6 | Advertencia de escala (continúa) |
| 7 | Memoria insuficiente |

---

## Tolerancias recomendadas por material

| Material | Offset recomendado |
|----------|-------------------|
| Silicona (RTV) | 0.0 - 0.1 mm |
| Resina epóxica | 0.1 - 0.3 mm |
| Resina de poliuretano | 0.2 - 0.5 mm |
| Yeso | 0.3 - 0.6 mm |

---

## Estado de desarrollo

### Fase 0 ✅ Completada

- Prototipo geométrico básico
- Carga de STL
- Generación de bloque de molde
- Validación de dependencias
- Medición de memoria

### Fase 1 🔄 En desarrollo

- Implementación de boolean CSG verdadero
- CLI completa
- Exportación STL
- Estimación de memoria

### Fase 2 📋 Planificado

- Orientación automática
- Detección de undercuts
- Auto-decimación

### Fase 3 📋 Planificado

- Moldes huecos
- Canales de vertido
- Exportación 3MF

---

## Licencia

MIT License - vea [LICENSE](LICENSE) para detalles.

---

## Contribuciones

Las contribuciones son bienvenidas. Por favor, abra un issue o PR en el repositorio.

---

## Enlaces útiles

- [Documento de requisitos (PRD)](PRD.md)
- [Resultados de Fase 0](docs/phase0_results.md)
- [Propuesta de Fase 1](docs/phase1_archive.md)
