# AutoMold — Product Requirements Document
**Versión 1.2 — Optimización de memoria y hardware modesto**
*Generación automática de moldes desde modelos 3D*

---

## Tabla de contenido

1. [Visión del producto](#1-visión-del-producto)
2. [Problema](#2-problema)
3. [Usuarios objetivo](#3-usuarios-objetivo)
4. [Objetivos del producto](#4-objetivos-del-producto)
5. [Filosofía de diseño](#5-filosofía-de-diseño)
6. [Formatos soportados](#6-formatos-soportados)
7. [Unidades y escala](#7-unidades-y-escala) 
8. [Interfaz CLI](#8-interfaz-cli)
9. [Gestión de errores y códigos de salida](#9-gestión-de-errores-y-códigos-de-salida) 
10. [Tolerancias del molde](#10-tolerancias-del-molde) 
11. [Reparación de mallas](#11-reparación-de-mallas-mesh-repair)
12. [Arquitectura técnica](#12-arquitectura-técnica)
13. [Dependencias externas](#13-dependencias-externas-crates-de-rust) 
14. [Pipeline geométrico](#14-pipeline-geométrico)
15. [Algoritmo de división del molde](#15-algoritmo-de-división-del-molde)
16. [Optimización de modelos grandes](#16-optimización-de-modelos-grandes)
17. [Estrategia de memoria para hardware modesto](#17-estrategia-de-memoria-para-hardware-modesto) 🆕 nuevo
18. [Estructura del repositorio](#18-estructura-del-repositorio)
19. [Fases de desarrollo](#19-fases-de-desarrollo)
20. [Testing](#20-testing)
21. [Métricas de rendimiento](#21-métricas-de-rendimiento)
22. [Criterios de aceptación por fase](#22-criterios-de-aceptación-por-fase)
23. [Licencia](#23-licencia)
24. [Futuro del proyecto](#24-futuro-del-proyecto)

---

## 1. Visión del producto

**AutoMold** es una herramienta de línea de comandos que genera automáticamente **moldes y contramoldes a partir de modelos 3D**, orientada a escultores y artistas que no tienen experiencia en software CAD.

El usuario solo necesita proporcionar un modelo `STL` o `OBJ`, y AutoMold generará automáticamente:

* Molde de una parte o dos partes
* Cavidad del modelo con tolerancias configurables
* Pines de alineación
* Bloque del molde
* Canales de vertido opcionales

El objetivo principal es **reducir el proceso de diseño de moldes a un solo comando**:

```
automold sculpture.stl
```

---

## 2. Problema

Crear moldes actualmente requiere:

* Conocimiento avanzado de CAD
* Múltiples operaciones booleanas
* Conocimiento de líneas de partición
* Mucho tiempo de modelado manual

Los artistas normalmente trabajan solo con:

* Mallas STL
* Escultura digital
* Escaneos 3D

AutoMold automatiza este proceso.

---

## 3. Usuarios objetivo

* Escultores digitales
* Artistas de miniaturas
* Creadores de props
* Makers
* Pequeños talleres de prototipado

---

## 4. Objetivos del producto

AutoMold debe:

1. Generar moldes automáticamente desde STL u OBJ
2. Eliminar la necesidad de CAD complejo
3. Funcionar desde CLI
4. Soportar modelos grandes (hasta 2M triángulos)
5. Generar moldes listos para impresión 3D con tolerancias correctas

---

## 5. Filosofía de diseño

AutoMold sigue cinco principios:

1. Simplicidad para artistas
2. Decisiones automáticas inteligentes
3. Robustez frente a mallas imperfectas
4. Velocidad de procesamiento
5. Arquitectura modular

---

## 6. Formatos soportados

**Entrada:**

* STL (binario y ASCII)
* OBJ

**Salida:**

* STL — formato principal
* 3MF — formato recomendado (preserva unidades, escala y metadatos)

**Archivos generados para molde de dos partes:**

```
model_mold_A.stl / model_mold_A.3mf
model_mold_B.stl / model_mold_B.3mf
metadata.json
```

**Para molde abierto:**

```
model_open_mold.stl / model_open_mold.3mf
metadata.json
```

---

## 7. Unidades y escala

### Unidad por defecto

**Milímetros (mm).** Esta es la unidad de facto en impresión 3D y coincide con la escala habitual de software como Blender, PrusaSlicer y Meshmixer.

### Flag de unidades

```
--unit mm|cm|in
```

Permite al usuario indicar la unidad original del modelo. AutoMold normalizará internamente a milímetros para todos los cálculos.

### Detección automática de escala

Si el bounding box del modelo es menor a 1 mm³ o mayor a 2000 mm³, AutoMold emitirá una advertencia:

```
WARNING: Model bounding box is 0.4mm x 0.3mm x 0.2mm. Did you mean to use --unit in?
```

La unidad utilizada quedará registrada en `metadata.json`:

```json
{
  "input_unit": "mm",
  "normalized_unit": "mm",
  "bounding_box_mm": [45.2, 32.1, 28.7]
}
```

---

## 8. Interfaz CLI

**Comando mínimo:**

```
automold model.stl
```

**Opciones disponibles:**

| Flag | Valores | Descripción |
|------|---------|-------------|
| `--open-mold` | flag | Genera molde abierto (sin tapa) |
| `--split-axis` | X \| Y \| Z | Eje de división del molde |
| `--wall <mm>` | número | Grosor de pared en mm (default: auto) |
| `--tolerance <mm>` | número | Offset de cavidad para material de colada (default: 0.2mm) |
| `--pins` | flag | Genera pines de alineación |
| `--pour` | flag | Genera canal de vertido |
| `--shell` | flag | Genera molde hueco (Fase 3) |
| `--unit` | mm \| cm \| in | Unidad del modelo de entrada (default: mm) |
| `--format` | stl \| 3mf | Formato de exportación (default: stl) |
| `--decimate <ratio>` | 0.0–1.0 | Reduce polígonos antes de procesar |
| `--memory-limit <MB>` | número | Límite de RAM en MB (default: auto-detect) 🆕 |
| `--threads <n>` | 1–16 | Número de threads de procesamiento (default: auto) 🆕 |

### Ejemplo de salida informativa

Si el usuario no especifica valores, el sistema toma decisiones automáticas y las informa:

```
AutoMold v0.1.0
Input: sculpture.stl (245,312 triangles, 45.2 x 32.1 x 28.7 mm)

Decisions:
  Split axis:      Z      (auto — least undercuts: 3)
  Wall thickness:  12mm   (auto — based on bounding box)
  Tolerance:       0.2mm  (auto — default)
  Pins:            enabled (auto)
  Pour channel:    disabled
  Unit:            mm (default)
  Memory budget:   1,820MB available — no decimation needed
  Threads:         4 (auto)

Processing...
  [OK] Mesh loaded
  [OK] Mesh repaired (2 holes filled, 0 non-manifold edges)
  [OK] Orientation analyzed
  [OK] Mold block generated
  [OK] Boolean operation (mesh)
  [OK] Split applied
  [OK] Pins generated

Output:
  sculpture_mold_A.stl
  sculpture_mold_B.stl
  metadata.json

Done in 1.4s
```

---

## 9. Gestión de errores y códigos de salida

Los errores se escriben a **stderr**. La salida informativa normal va a **stdout**, lo que permite capturarlos por separado:

```
automold model.stl 2> errors.log
```

### Códigos de salida

| Código | Causa | Mensaje de ejemplo |
|--------|-------|--------------------|
| `0` | Éxito | `Done in 1.4s` |
| `1` | Archivo no encontrado | `ERROR: File 'model.stl' not found` |
| `2` | Formato no soportado | `ERROR: Unsupported format '.fbx'. Use STL or OBJ` |
| `3` | Malla irrecuperable | `ERROR: Mesh repair failed — too many non-manifold edges (>10%)` |
| `4` | Boolean falló (incluso fallback SDF) | `ERROR: Boolean operation failed after SDF fallback. Try --decimate` |
| `5` | Argumento inválido | `ERROR: --wall must be > 0. Got: -3` |
| `6` | Advertencia de escala (continúa) | `WARNING: Bounding box < 1mm. Consider --unit in` |
| `7` | Memoria insuficiente y `--force` no indicado | `ERROR: Estimated memory 3.2GB exceeds limit 1.8GB. Use --decimate or --force` |

---

## 10. Tolerancias del molde

### Offset de cavidad

El sistema aplica un offset uniforme hacia afuera de la superficie del modelo antes de la operación booleana. Este offset compensa la contracción o expansión del material de colada al curar.

**Valores de referencia por material:**

| Material | Offset recomendado | Notas |
|----------|--------------------|-------|
| Silicona (RTV) | 0.0 – 0.1 mm | Alta flexibilidad, desmolde fácil |
| Resina epóxica | 0.1 – 0.3 mm | Contracción baja |
| Resina de poliuretano | 0.2 – 0.5 mm | Contracción moderada |
| Yeso | 0.3 – 0.6 mm | Se expande al curar |

El valor default de AutoMold es **0.2 mm**, adecuado para la mayoría de resinas. El usuario puede ajustarlo con:

```
automold model.stl --tolerance 0.3
```

### Holgura de línea de partición

En la línea de partición se aplicará una holgura adicional de **0.1 mm** para facilitar la apertura sin dañar los bordes. Este valor no es configurable en Fase 1 pero podrá serlo en fases futuras.

---

## 11. Reparación de mallas (mesh repair)

Los modelos 3D provenientes de escaneos o escultura digital frecuentemente contienen errores geométricos. AutoMold los maneja automáticamente antes de proceder al pipeline.

### Tipos de errores tratados

| Error | Severidad | Acción |
|-------|-----------|--------|
| Normales invertidas | Baja | Recalcular automáticamente |
| Agujeros (holes) | Media | Rellenar con triangulación mínima |
| Aristas no-manifold | Media | Intentar reparar; advertir si > 5% del total |
| Caras duplicadas | Baja | Eliminar automáticamente |
| Intersecciones propias | Alta | Intentar resolver; usar SDF fallback si falla |
| > 10% aristas irrecuperables | Fatal | Abortar con exit code `3` |

### Reporte de reparación

AutoMold informa qué reparaciones realizó:

```
  [OK] Mesh repaired (2 holes filled, 14 inverted normals fixed, 0 non-manifold)
```

---

## 12. Arquitectura técnica

**Lenguaje principal: Rust**

Razones:

* Alto rendimiento
* Seguridad de memoria sin garbage collector
* Excelente soporte para CLI (`clap` crate)
* Adecuado para procesamiento geométrico intensivo

---

## 13. Dependencias externas (crates de Rust)

La selección final debe validarse en la **Fase 0** antes de comprometerse con una dependencia en todo el proyecto.

| Módulo | Crate principal | Alternativa | Notas |
|--------|----------------|-------------|-------|
| CLI parsing | `clap 4.x` | `argh` | clap recomendado, más features |
| Carga STL | `stl_io` | implementación propia | Soporta binario y ASCII |
| Carga OBJ | `tobj` | `wavefront_obj` | tobj más mantenido |
| BVH acceleration | `bvh` crate | `parry3d` | parry3d más completo pero más pesado |
| Mesh boolean | `mesh_bool` (experimental) | implementación propia con BVH | Evaluar en Fase 0 |
| SDF fallback | `fast_surface_nets` | `isosurface` | Evaluar rendimiento en Fase 0 |
| Decimación QEM | `meshopt` | implementación propia | meshopt es portado de C++, probado |
| Álgebra lineal | `nalgebra` | `glam` | nalgebra más completo para geometría 3D |
| Serialización JSON | `serde_json` | — | Estándar en ecosistema Rust |
| Logging | `tracing` | `log` + `env_logger` | tracing para diagnóstico estructurado |

---

## 14. Pipeline geométrico

AutoMold utiliza una arquitectura híbrida optimizada para rendimiento.

```
mesh model
     │
     ▼
mesh repair
(holes, normales, non-manifold)
     │
     ▼
memory budget estimation 🆕
(estimar RAM requerida; activar auto-decimate si excede límite)
     │
     ▼
decimation pass 1 — ligera (~80%) para análisis 🆕
     │
     ▼
orientation analysis (PCA + undercuts)
     │
     ▼
decimation pass 2 — completa si --decimate fue indicado 🆕
     │
     ▼
apply tolerance offset
     │
     ▼
mold block generation
     │
     ▼
fast mesh boolean (BVH + spatial hashing) 🆕
     │
     ▼
fallback SDF boolean — resolución adaptativa por memoria 🆕
     │
     ▼
mold split
     │
     ▼
pin generation
     │
     ▼
pour channel generation
     │
     ▼
streaming STL / 3MF export 🆕 + metadata.json
```

### Operación booleana central

```
mold = block - model
```

**Nueva arquitectura CSG (v0.1.0+):**
El sistema implementa una estrategia de fallback en cascada:
1. **CSG (csgrs)**: BSP-tree based CSG para booleanos exactos
2. **Voxelization**: Fallback con grid de voxeles + marching cubes
3. **SimpleAABB**: Fallback final rápido (solo elimina triángulos completamente fuera del AABB)

Selección automática basada en complejidad de la malla y presupuesto de memoria (límite configurable, default 512MB).

**Booleanos rápidos sobre malla (BVH + spatial hashing):**
* 10–50× más rápido que SDF
* Menor uso de memoria
* Spatial hashing para aislar la región de intersección antes del boolean completo — reduce el problema de 500k triángulos a ~50k en la zona activa
* Adecuado para modelos grandes

**Fallback SDF con resolución adaptativa:**
Si el boolean de malla falla por errores geométricos, el sistema hace fallback automático a SDF boolean. La resolución del voxel grid se calcula dinámicamente según el presupuesto de memoria disponible para evitar OOM en laptops:

```
max_voxels = memory_budget_bytes / 4
grid_side  = cbrt(max_voxels)
voxel_size = max(bbox) / grid_side
```

Esto garantiza que el fallback SDF nunca supera la RAM disponible. En hardware modesto la resolución será menor, pero el proceso nunca muere por falta de memoria.

---

## 15. Algoritmo de división del molde

El sistema implementa tres niveles de complejidad progresivos.

### Nivel 1 — División simple (Fase 1)

Basado en PCA orientation y plano central del bounding box:

```
split_plane = bbox.center
```

Rápido y adecuado para V1.

### Nivel 2 — Detección de undercuts (Fase 2)

El sistema analiza las normales de las superficies:

```
dot(normal, pull_direction) > 0  →  visible
dot(normal, pull_direction) < 0  →  undercut
```

La dirección con menos undercuts es seleccionada automáticamente.

### Nivel 3 — Análisis avanzado de visibilidad (Fase 3)

Método basado en ray casting. Permite generar superficies de partición complejas, equivalente a lo usado en software industrial.

---

## 16. Optimización de modelos grandes

Antes de generar el molde se pueden aplicar limpieza de malla y reducción de polígonos mediante **Quadric Error Metrics (QEM)**. Esto reduce modelos extremadamente densos sin perder detalle visible.

AutoMold implementa una **estrategia de doble pasada de decimación** para no comprometer el análisis de orientación:

* **Pasada 1 (ligera, ~80% triángulos):** usada exclusivamente para el análisis de orientación y detección de undercuts. Preserva la geometría fina que define cuál eje es óptimo para partir el molde.
* **Pasada 2 (completa, ratio configurado por el usuario):** aplicada antes del boolean si el usuario indicó `--decimate`.

```
automold large_model.stl --decimate 0.5   # reduce a 50% para el boolean
```

Si no se indica `--decimate` pero el modelo excede el presupuesto de memoria, AutoMold activará decimación automática e informará al usuario (ver sección 17).

---

## 17. Estrategia de memoria para hardware modesto

> 🆕 Sección nueva — AutoMold debe funcionar correctamente en laptops con 8GB RAM, no solo en workstations.

El mayor riesgo en hardware modesto es quedarse sin memoria durante el boolean o el fallback SDF, lo que resulta en un proceso killed sin mensaje de error útil para el usuario.

### Estimación de memoria antes de procesar

Al cargar el modelo, AutoMold estima la RAM requerida para el pipeline completo:

| Estructura | Fórmula estimada |
|------------|-----------------|
| Malla original | `triángulos × 120 bytes` |
| BVH tree | `triángulos × 80 bytes` |
| Copia para boolean | `triángulos × 120 bytes` |
| Buffer de resultado | `triángulos × 150 bytes` (estimado conservador) |
| **Total estimado** | **`triángulos × ~470 bytes`** |

Para un modelo de 2M triángulos esto es ~940MB. Si la RAM disponible es insuficiente, AutoMold actúa antes de procesar, no después de crashear.

### Comportamiento automático por presupuesto

```
RAM disponible detectada (via sysinfo crate)
          │
          ▼
   ¿estimado <= disponible × 0.75?
        /        \
      Sí          No
      │            │
   Proceder     ¿--force?
   normalmente    /    \
               Sí       No
               │         │
           Proceder   Auto-decimate
           con warning  + informar
```

Ejemplo de warning con auto-decimate:

```
WARNING: Model requires ~1.4GB estimated. Available: 900MB.
Auto-decimating to 60% (--decimate 0.6). Use --force to skip.
  [OK] Decimation applied (2,000,000 → 1,200,000 triangles)
```

### Exportación por streaming

La exportación STL escribe triángulos al archivo en chunks de 10,000 en lugar de mantener el molde completo en memoria. Esto elimina el pico de memoria al final del pipeline, que puede ser el momento más crítico.

El módulo `export/stl.rs` debe implementar escritura incremental — nunca acumular el resultado completo en un `Vec<Triangle>` antes de escribir.

### Paralelismo conservador con `rayon`

`rayon` se usa para paralelizar las etapas embarrassingly parallel del pipeline (análisis de undercuts, construcción del BVH, evaluación de normales). Sin embargo, en hardware modesto el paralelismo agresivo compite con la memoria caché y puede ser contraproducente.

Regla por defecto:

```
threads = min(available_logical_cores, 4)
```

El usuario puede sobreescribir con `--threads`. En laptops con 4GB activos se recomienda `--threads 2`.

```
automold model.stl --threads 2
```

### Consumo de memoria estimado por configuración

| Modelo | Sin opciones | Con `--decimate 0.5` | Fallback SDF (resolución auto) |
|--------|-------------|----------------------|-------------------------------|
| 50k triángulos | ~24MB | ~12MB | ~50MB |
| 500k triángulos | ~235MB | ~118MB | ~200MB |
| 2M triángulos | ~940MB | ~470MB | ~400–800MB |

> Los valores del fallback SDF dependen del bounding box y la RAM disponible, no del conteo de triángulos.

---

## 18. Estructura del repositorio

```
automold/

Cargo.toml

src/

  main.rs

  cli/
    args.rs
    commands.rs

  core/
    config.rs
    pipeline.rs
    context.rs

  geometry/
    mesh.rs
    bbox.rs
    bvh.rs
    sdf.rs
    offset.rs        — tolerance offset

  pipeline/
    loader.rs
    repair.rs        — manejo detallado de errores
    decimate.rs
    orientation.rs
    mold_block.rs
    boolean.rs
    split.rs
    pins.rs
    pour.rs

  export/
    stl.rs
    threemf.rs       — exportación 3MF
    metadata.rs

  utils/
    math.rs
    logging.rs
    units.rs         — conversión de unidades
    memory.rs       🆕 nuevo — estimación y presupuesto de RAM

tests/

examples/
```

---

## 19. Fases de desarrollo

### Fase 0 — Prototipo geométrico

**Objetivo:** Probar generación de cavidad básica y validar dependencias.

Características:
* Cargar STL
* Generar bloque
* Boolean simple
* Export STL
* Validar crates de BVH y SDF en modelos reales
* Benchmark de consumo de memoria en modelos de prueba

### Fase 1 — Generación de moldes

Funciones:
* Moldes de dos partes
* Configuración de pared
* CLI básica con flags principales
* Tolerancia de cavidad configurable (`--tolerance`)
* Unidades configurables (`--unit`)
* Códigos de salida y mensajes de error
* Estimación de memoria y warning antes de procesar
* Exportación por streaming (sin acumular en memoria)

### Fase 2 — Automatización

Funciones:
* Orientación automática
* División automática con detección de undercuts
* Generación automática de pines
* Decisiones automáticas informadas en stdout
* Reparación de mallas avanzada
* Auto-decimate por presupuesto de memoria (`--memory-limit`)
* Paralelismo con `rayon` + flag `--threads`
* Doble pasada de decimación (análisis vs boolean)

### Fase 3 — Optimización

Funciones:
* Moldes huecos (`--shell`)
* Canales de vertido
* Exportación 3MF
* Optimización de material
* Soporte robusto para modelos grandes (hasta 2M triángulos en laptops de 8GB)
* SDF fallback con resolución adaptativa completa

---

## 20. Testing

### Unit tests

* Carga de mallas (STL binario, STL ASCII, OBJ)
* Cálculo de bounding box
* PCA orientation
* Operaciones booleanas
* Generación de moldes
* Conversión de unidades
* Aplicación de tolerancia offset
* Estimación de memoria (verificar fórmula contra medición real)
* Resolución adaptativa SDF (verificar que no excede presupuesto dado)

### Tests de integración

* Pipeline completo con modelos reales:
  * Esfera (sin undercuts, caso ideal)
  * Cubo (aristas rectas, caso simple)
  * Modelo con agujeros (prueba reparación)
  * Modelo con undercuts (prueba análisis de partición)
  * Modelo de alta densidad >500k triángulos (prueba rendimiento)
  * Modelo de 2M triángulos con `--memory-limit 800` (prueba auto-decimate)
* Verificar que los STL generados son válidos e imprimibles
* Verificar que la tolerancia aplicada coincide con el valor configurado
* Verificar que exportación streaming produce archivo idéntico a exportación en memoria

### Tests de regresión

* Suite de modelos STL de referencia con resultados esperados conocidos
* Ejecutar en cada PR para detectar regresiones en el pipeline geométrico

---

## 21. Métricas de rendimiento

Objetivos medidos en **dos configuraciones de referencia**:

* **Workstation:** Apple M2 8-core / AMD Ryzen 5 5600X, 16GB RAM
* **Laptop modesta:** Intel Core i5-10th gen 4-core, 8GB RAM (objetivo prioritario)

### Tiempo de procesamiento

| Modelo | Triángulos | Workstation | Laptop modesta |
|--------|-----------|-------------|----------------|
| Pequeño | ~50k | < 1s | < 2s |
| Mediano | ~500k | 1 – 3s | 3 – 6s |
| Grande | ~2M | 3 – 10s | 8 – 20s (con auto-decimate) |

Con `--decimate 0.5`, el tiempo de modelos grandes debe reducirse al rango del modelo mediano en ambas configuraciones.

### Consumo de memoria máximo

| Modelo | Triángulos | Sin opciones | Con `--decimate 0.5` | Fallback SDF (auto) |
|--------|-----------|-------------|----------------------|---------------------|
| Pequeño | ~50k | < 30MB | < 15MB | < 60MB |
| Mediano | ~500k | < 250MB | < 130MB | < 220MB |
| Grande | ~2M | < 1GB | < 500MB | < 800MB |

En laptop de 8GB, los modelos grandes deben procesarse dentro del límite de 800MB con auto-decimate activado automáticamente.

### Criterio de no-regresión de memoria

Ningún PR puede incrementar el pico de memoria en más de 10% para ninguno de los modelos de prueba de la suite de regresión.

---

## 22. Criterios de aceptación por fase

### Fase 0 — Completada cuando:

* Se puede cargar un STL arbitrario sin errores
* La operación `block - model` produce una cavidad geométricamente correcta en al menos 5 modelos de prueba distintos
* El boolean mesh funciona en modelos limpios; el fallback SDF funciona en modelos con errores geométricos
* Las crates seleccionadas para BVH y SDF están integradas y los benchmarks de rendimiento son aceptables
* El STL exportado se puede abrir en PrusaSlicer sin errores
* Se ha medido el consumo de memoria real de cada etapa del pipeline en los 5 modelos de prueba

### Fase 1 — Completada cuando:

* `automold model.stl` genera dos mitades de molde válidas para ≥ 90% de los modelos de prueba
* Los flags `--wall`, `--tolerance`, `--unit`, `--memory-limit`, `--threads` funcionan y se validan
* Los códigos de salida son correctos para todos los escenarios documentados incluyendo el código `7` (OOM)
* La exportación por streaming produce archivos STL idénticos a la exportación en memoria
* La suite de tests de integración pasa al 100%
* El pico de memoria en laptop de 8GB no supera 250MB para modelos medianos

### Fase 2 — Completada cuando:

* La orientación automática selecciona el eje óptimo en ≥ 80% de los casos
* Auto-decimate activa correctamente cuando el modelo excede el presupuesto de memoria
* El análisis de orientación usa la pasada ligera (80%) y no la decimación completa
* `--threads` reduce el tiempo en proporción razonable (≥ 1.5× speedup de 1 a 4 threads)
* Tests de regresión pasan sin degradación respecto a Fase 1

### Fase 3 — Completada cuando:

* `--shell` genera moldes huecos con grosor de pared uniforme
* La exportación 3MF preserva correctamente las unidades y metadatos
* Modelos de 2M triángulos procesan en laptop de 8GB dentro del target con auto-decimate
* SDF fallback con resolución adaptativa nunca excede el presupuesto de memoria configurado
* Los canales de vertido se posicionan automáticamente en la parte superior del molde

---

## 23. Licencia

MIT License.

Permite:
* Uso comercial
* Modificación
* Redistribución

---

## 24. Futuro del proyecto

Evoluciones posibles:

* Interfaz gráfica
* Plugin para Blender
* Optimización industrial de moldes
* Simulación de colada
* Generación multicavidad

AutoMold busca convertirse en una herramienta estándar para artistas que necesitan generar moldes de forma rápida y automática.

---

*AutoMold PRD v1.2 — Última revisión: 2026-03-15*