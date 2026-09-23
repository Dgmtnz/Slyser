# Registro de Iteraciones - Slyser

| Iteración | Objetivo | Estado | Notas / Resultados |
|---|---|---|---|
| **0** | Arquitectura base, estructuras de datos, I/O STL y pipeline de tests | Completada | 11 tests unitarios pasando, clippy limpio, soporte binario/ASCII, modelos sintéticos (cubo, T-bracket con voladizos a 90°). |
| **1** | Motor geométrico de overhangs y estimación de volumen de soporte | Completada | BVH acelerador, Möller-Trumbore, integración por cuadratura, discriminación cama/modelo, paralelismo Rayon en 16 hilos. 6 tests de integración pasando. |
| **2** | Reorientación automática óptima en la cama de impresión | Completada | Clustering planar, rotaciones a -Z, convex hull 2D para estabilidad física, evaluación multihilo Rayon. 7 tests pasando, 100% reducción en T-bracket. |
| **3** | Motor de corte plano con cierre hermético (watertight) | Completada | Partición por plano arbitrario, reconstrucción de tapas herméticas por ear clipping, conservación de volumen < 1%, verificación 2-manifold closed. 9 tests pasando. |
| **4** | Generador de cajeados hembra-hembra y clavijas cuadradas independientes | Completada | Cavidades paramétricas con holgura FDM, cierre hermético garantizado, generación de clavijas independientes a Z=0. 7 tests pasando. |

| **5** | Optimizador inteligente de particiones (Pareto) | Completada | Heurísticas multiescala (overhangs, cuellos de botella, PCA, offsets), exploración paralela con Rayon en 16 hilos, reducción 100% de soporte en T-bracket. 6 tests pasando. |
| **6** | CLI completa, estadísticas y validación end-to-end | Completada | Subcomando `partition` con flags completas, reporte de métricas en terminal y exportación STL de piezas a Z=0 y clavijas. |
| **7** | Garantía de piezas contiguas (sin fragmentos desconexos) y clavijas adaptativas | Completada | DSU de componentes conexas, restricción en cortes del optimizador y auto-escalado adaptativo de clavijas en secciones estrechas. 5 tests pasando. |

---

## Log de Iteraciones

### Iteración 0 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Inicializado proyecto Rust `slyser` con cargo.
  - Verificado entorno: rustc 1.94.1, 16 núcleos lógicos de CPU disponibles.
  - Implementadas estructuras de malla (`Vertex`, `Triangle`, `Mesh`, `Plane`, `AABB`) en `src/mesh.rs`.
  - Implementado parser/writer binario y ASCII con detección automática en `src/stl.rs`.
  - Creados generadores de mallas sintéticas en `src/primitives.rs` (cubos y T-brackets con voladizos analíticos a 90°).
  - Creados 11 tests unitarios en `tests/iteration_0.rs` verificando cálculo de volumen, áreas, normales, transformaciones afines e I/O STL de ida y vuelta.
  - Verificación exitosa: 11/11 tests aprobados, `cargo clippy` sin advertencias.

### Iteración 1 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Implementado árbol BVH lineal con particionamiento por mediana espacial y recorrido en pila fija sin asignaciones dinámicas en `src/bvh.rs`.
  - Implementado algoritmo de intersección rayo-triángulo Möller-Trumbore (`RayHit`, `intersect_triangle`, `intersect_ray`, `intersect_ray_with_options`).
  - Implementado motor de análisis de voladizos y estimación de soportes en `src/support.rs` (`SupportConfig`, `SupportMetrics`, `analyze_support`, `is_overhang`).
  - Detección de voladizos según convención estándar FDM con exclusión automática de capas base en cama ($z = z_{\text{bed}}$).
  - Integración numérica por cuadratura regular sobre facetas proyectadas con discriminación de columnas que aterrizan en cama vs modelo.
  - Paralelización multihilo con `rayon` sobre los 16 núcleos disponibles.
  - Añadido comando `analyze` en `src/main.rs`.
  - Añadidos 6 tests de integración en `tests/iteration_1.rs` (cubo plano 0 mm³ soporte, cubo rotado 45°, volumen analítico T-bracket verificado dentro de tolerancia < 2%, soporte sobre modelo, raycasting BVH y rendimiento multihilo).
  - Verificación exitosa: 20/20 tests aprobados, `cargo clippy --all-targets -- -D warnings` con 0 advertencias.

### Iteración 2 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Implementado motor de reorientación automática óptima en `src/orient.rs` (`OrientationCandidate`, `OptimalOrientation`, `extract_candidate_orientations`, `orient_mesh_to_bed`, `is_orientation_stable`, `find_optimal_orientation`, `compute_bed_contact_area`, `rotation_to_down`).
  - Agrupación por clustering planar de triángulos coplanares dentro de tolerancia angular (~8°) y traslación a $z_{\min}=0$.
  - Inclusión garantizada de rotaciones sobre ejes canónicos ($\pm X, \pm Y, \pm Z$) y deduplicación angular.
  - Algoritmo de convex hull 2D (Andrew's Monotone Chain) y test de contención punto-en-polígono convexo para verificación de estabilidad estática bajo gravedad en cama ($z \le 1e-3$).
  - Función de aptitud multi-objetivo evaluada en paralelo con Rayon: 1) minimizar volumen de soporte, 2) minimizar soporte sobre el modelo, 3) maximizar área de contacto con la cama, 4) priorizar estabilidad física.
  - Re-exportadas estructuras y funciones en `src/lib.rs`.
  - Añadido subcomando `orient` en CLI (`src/main.rs`) reportando volumen original vs reorientado, porcentaje de reducción, área de contacto, estabilidad y exportación STL.
  - Creados 7 tests de integración en `tests/iteration_2.rs` (cubo plano, cubo rotado 45° recuperado a plano, T-bracket invertido con 100% de reducción a 0.0 mm³, T-bracket asimétrico, cuña prismática inclinada, comprobación de estabilidad con cantilever fuera de base y extracción de candidatos).
  - Verificación exitosa: 27/27 tests aprobados, `cargo clippy --all-targets -- -D warnings` con 0 advertencias.

### Iteración 3 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Implementado motor de particionamiento plano y generación de tapas herméticas en `src/cut.rs` (`CutResult`, `slice_mesh_by_plane`, `compute_plane_basis`, `project_to_plane_2d`, `unproject_from_plane_2d`).
  - Algoritmo de división robusta de triángulos contra plano arbitrario con snapping numérico a la tolerancia $\epsilon = 1e-5$.
  - Extracción de bordes límite no emparejados en el plano de corte con cancelación de aristas internas.
  - Encadenamiento directo de segmentos en contornos poligonales cerrados 2D (`cut_polygons_2d`).
  - Triangulación de contornos 2D mediante algoritmo de Ear Clipping con preservación de aristas 3D coincidentes y normales orientadas hacia $-\hat{n}$ (malla positiva) y $+\hat{n}$ (malla negativa).
  - Verificación estricta de conservación de volumen ($\Delta V < 1\%$).
  - Añadidos métodos `is_closed` e `is_watertight` en `Mesh` (`src/mesh.rs`) para verificación topológica de variedades 2D cerradas sin bordes abiertos.
  - Perfeccionada la generación de `create_t_bracket` en `src/primitives.rs` para garantizar variedad 2-manifold estricta sin uniones en T (T-junctions).
  - Añadido subcomando `cut` a la CLI (`src/main.rs`) permitiendo cortar modelos con punto y normal arbitrarios y exportar los dos STLs herméticos (`<prefix>_pos.stl`, `<prefix>_neg.stl`).
  - Creados 9 tests de integración en `tests/iteration_3.rs` cubriendo cortes ortogonales de cubos 20x20x20, cortes diagonales a 45° a través de vértices y aristas coplanares, cortes oblicuos 3D simétricos, cortes a través del vástago de T-bracket, cortes en la base de los voladizos, cortes de simetría vertical, comprobación de bases ortonormales y manejo de errores para planos disjuntos o normales nulas.
  - Verificación exitosa: 36/36 tests aprobados, `cargo clippy --all-targets -- -D warnings` con 0 advertencias.

### Iteración 4 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Implementado motor de cajeados hembra-hembra y clavijas cuadradas independientes en `src/pins.rs` (`PinConfig`, `PinPlacement`, `find_pin_placements`, `generate_square_pin`, `apply_female_sockets_and_generate_pins`).
  - Algoritmo de colocación segura de clavijas verificando contención estricta en el polígono de corte más grande con margen de seguridad perimetral configurable (`edge_margin`), distribuyendo automáticamente 1 clavija en centroide para secciones medias, 2 clavijas en secciones alargadas, o 4 en secciones grandes.
  - Reconstrucción hermética de tapas con orificios mediante triangulación con puentes de costura y paredes laterales ortogonales hacia adentro de ambas mitades ($\pm \hat{n}$), garantizando topología 2-manifold cerrada (`is_closed() == true` y `is_watertight() == true`).
  - Verificación analítica de reducción exacta de volumen en mitades positiva y negativa: $\Delta V = (S + 2c)^2 \times D$.
  - Generador de mallas de clavijas cuadradas independientes orientadas planas sobre $Z=0$ con tolerancia axial de 0.4 mm para contacto rasante.
  - Actualizado subcomando `cut` en CLI (`src/main.rs`) añadiendo flags `--pins`, `--pin-size`, `--pin-depth`, `--pin-clearance`, exportando `<prefix>_pos.stl`, `<prefix>_neg.stl` y `<prefix>_pin_*.stl`.
### Iteración 5 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Implementado motor de optimización multi-pieza de Pareto en `src/optimizer.rs` (`OptimizerConfig`, `CandidatePlaneGenerator`, `PartitionPiece`, `OptimizationResult`, `optimize_partitions`).
  - Diseñadas 4 heurísticas de generación de planos de corte candidatos:
    1. Planos de contorno de voladizos (niveles Z y coordenadas de unión con paredes verticales).
    2. Barrido de cuellos de botella geométricos mediante cálculo por Teorema de Green de áreas de sección transversal a lo largo de X, Y, Z.
    3. Descomposición espectral PCA (mediante rotaciones de Jacobi para matrices simétricas 3x3) sobre clusters de facetas en voladizo.
    4. Planos distribuidos a lo largo de los ejes principales de la caja envolvente (AABB) y diagonales canónicas.
  - Evaluación paralela con `rayon` sobre los 16 núcleos de CPU con filtrado de fragmentos mínimos (`min_piece_volume_ratio`).
  - Función de aptitud de Pareto combinando volumen de soporte y penalización de área de corte con reorientación automática de cada mitad a la cama ($Z=0$).
  - Particionado recursivo multi-pieza hasta alcanzar `max_pieces` o soporte cero.
  - Integración completa con generación de cajeados hembra y clavijas de alineación cuando `pin_config` está activo.
  - Re-exportadas estructuras en `src/lib.rs`.
  - Creados 6 tests de integración en `tests/iteration_5.rs` (optimización y 100% de reducción en T-bracket, ensamble con clavijas, compromiso de Pareto sin cortes en cubos planos, heurísticas de generación de planos, rechazo de micro-fragmentos y límite de piezas).
  - Verificación exitosa: 49/49 tests aprobados, `cargo clippy --all-targets -- -D warnings` con 0 advertencias.

### Iteración 6 (Completada)
- **Fecha:** 2026-09-23
- **Acciones:**
  - Implementado subcomando `partition` en CLI (`src/main.rs`) con argumentos `--input`, `--output-dir`, `--max-pieces`, `--angle`, `--pins`, `--pin-size`, `--pin-depth`, `--pin-clearance`.
  - Tabla de telemetría formateada en terminal con reporte de volumen original vs final, porcentaje de reducción, piezas generadas, número de clavijas y tiempos de ejecución.
  - Desglose por pieza (dimensiones, volumen de soporte restante, área de contacto en cama).
  - Exportación automática a `<out_dir>/piece_*.stl` (planas en $Z=0$) y `<out_dir>/pin_*.stl`.
  - Verificación de ejecución completa end-to-end sobre modelos STL.

### Iteración 7 (Completada)
- **Fecha:** 2026-09-24
- **Acciones:**
  - Implementado algoritmo de clustering de componentes conexas en `Mesh` (`src/mesh.rs`) con cuantización de coordenadas a $10^{-4}$ mm y estructura Disjoint Set Union (DSU) (`connected_components`, `connected_components_count`, `is_contiguous`, `largest_connected_component`).
  - **Preservación total de geometría:** Eliminada cualquier lógica de descarte de fragmentos o pérdida de volumen. Cero geometría eliminada (conservación del 100.0% del volumen del modelo original).
  - **Planos inclinados para corte anatómico limpio:** Incorporada generación de planos candidatos inclinados (pitch/roll de ±15°, ±25°, ±35°) en cuellos de botella y centroides, permitiendo separar piezas orgánicas (ej. cuello en bustos) en un único lazo cerrado sin rozar ni separar puntas flotantes.
  - **Sondeo de espesor y profundidad segura de clavijas:** Implementado sondeo por raycasting BVH (`probe_mesh_thickness`, `compute_safe_pin_depth`) desde el centro y los 4 vértices del cajeado hacia el interior de ambas piezas. Se descuenta un margen de seguridad de al menos 1.8 mm de pared sólida, garantizando que el cajeado nunca perfore ni rompa la superficie exterior.
  - Dimensionamiento adaptativo de clavijas cuadradas con longitud ajustada a `2.0 * safe_depth - 0.4 mm`.
  - Creados 6 tests de integración en `tests/iteration_7.rs`.
  - Verificación exitosa: 55/55 tests aprobados en todas las suites (iteraciones 0 a 7), `cargo clippy --all-targets -- -D warnings` con 0 advertencias.
  - Validación en `VenomEscalado.stl`: ambas piezas certificadas por PrusaSlicer como `manifold = yes` y `number_of_parts = 1`, con cajeado de 1.9 mm de profundidad y clavija [`pin_1.stl`](file:///home/dgmtnz/Documents/Slyser/pin_1.stl) de 2.5 x 2.5 x 3.4 mm.



