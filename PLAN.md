# Plan Maestro de Desarrollo - Slyser

Herramienta de alto rendimiento en Rust para partición inteligente de mallas STL orientada a eliminación de soportes en impresión 3D (FDM).

## Especificaciones Clave
- **Lenguaje:** Rust (edición 2021/2024), 100% nativo sin dependencias externas de runtime.
- **Concurrencia:** Multihilo intensivo con `rayon` (aprovechando los 16 núcleos lógicos disponibles).
- **Cálculo de Soportes:** Detección de overhangs (ángulo crítico $Y$) y proyección vertical de sombras/raycast emulando algoritmos de slicers FDM.
- **Reorientación:** Búsqueda automática de la mejor orientación plana en cama por cada pieza resultante.
- **Sistema de Ensamble:** Cortes planos limpios con cavidades hembra-hembra cuadradas y generación de clavijas cuadradas independientes en STL separado con tolerancia FDM.
- **Optimización:** Búsqueda multi-objetivo (Pareto) para minimizar volumen de soporte respetando un límite máximo de $X$ piezas.

---

## Fases e Iteraciones

### Iteración 0: Setup, Arquitectura de Datos y Pipeline de Pruebas
- [x] Configuración de dependencias en `Cargo.toml` (`rayon`, `glam`, `clap`).
- [x] Estructuras de datos para mallas triangulares 3D (`Mesh`, `Triangle`, `Vertex`, `Plane`, `AABB`).
- [x] Parser/Writer binario y ASCII STL de alta velocidad con auto-detección.
- [x] Suite de pruebas unitarias y generación de mallas sintéticas de prueba (cubo, T-bracket con voladizos controlados a 90°).
- [x] *Criterio de aceptación:* `cargo test` pasa (11 tests unitarios), clippy limpio sin advertencias y comandos `info`/`generate` funcionales.

### Iteración 1: Motor Geométrico de Overhangs y Estimación de Soporte
- [x] Detección vectorial de facetas en voladizo según ángulo umbral configurable $Y$ ($\vec{n} \cdot -\vec{z} > \cos(\alpha)$).
- [x] Proyección de sombras y cálculo de volumen estimado de soporte ($\text{mm}^3$) hacia la cama ($Z=0$) o hacia superficies inferiores del modelo.
- [x] Paralelización Rayon para evaluación de mallas complejas en sub-milisegundos.
- [x] *Criterio de aceptación:* Cálculo exacto en piezas de prueba conocidas con voladizos analíticos controlados.

### Iteración 2: Reorientación Automática Óptima en Cama de Impresión
- [x] Extracción de planos de apoyo estables (facetas del convex hull y facetas planas dominantes).
- [x] Transformaciones de rotación $SE(3)$ para alinear cada base con la cama $Z=0$.
- [x] Evaluación paralela multihilo del soporte generado en cada orientación candidata.
- [x] *Criterio de aceptación:* Detección y selección automática de la orientación que minimiza soporte para cualquier pieza.

### Iteración 3: Motor de Corte Plano y Cierre Hermético (Watertight)
- [x] Algoritmo de división de triángulos por un plano arbitrario $P(P_0, \vec{n})$.
- [x] Reconstrucción y triangulación de la sección de corte (tapas planas herméticas para ambas mitades).
- [x] Verificación de manifoldness / piezas completamente cerradas aptas para laminar.
- [x] *Criterio de aceptación:* Corte limpio de modelos cerrados produciendo dos STLs cerrados sin agujeros ni aristas abiertas.


### Iteración 4: Generación de Cavidades Hembra-Hembra y Clavijas Cuadradas
- [x] Cálculo de área disponible en la faceta de corte para posicionamiento seguro de clavijas.
- [x] Generación de cajeados cuadrados (hembra) en ambas mitades a lo largo del vector normal del corte.
- [x] Inclusión de holgura/tolerancia FDM configurable (por defecto 0.2 mm).
- [x] Generación y exportación de las clavijas cuadradas independientes (`pin_*.stl`).
- [x] *Criterio de aceptación:* Visualización e inspección dimensional de las cajas y la clavija con holgura exacta.


### Iteración 5: Algoritmo Inteligente de Partición Óptima (Pareto)
- [x] Generación heurística de planos de corte candidatos (planos en cuello de botella, límites de clusters de overhangs, PCA de voladizos y offsets en ejes).
- [x] Exploración paralela en 16 hilos del espacio de corte con Rayon.
- [x] Función de aptitud Pareto: $\min (\text{VolumenSoporte} + \lambda_1 \cdot \text{ÁreaCorte})$ con descarte de fragmentos ínfimos (< 2%).
- [x] Soporte para límite máximo de $X$ piezas y parada temprana si el soporte llega a cero o no se mejora la aptitud.
- [x] *Criterio de aceptación:* Reducción demostrada del 100% de soporte en piezas de prueba complejas (T-bracket) respecto al STL original.

### Iteración 6: Interfaz CLI, Telemetría y Validación Final
- [x] Argumentos CLI con `clap` (`--input`, `--output-dir`, `--max-pieces`, `--angle`, `--pins`, `--pin-size`, `--pin-depth`, `--pin-clearance`).
- [x] Formateo de métricas en terminal (reducción de soporte en %, desglose dimensional y de soporte por pieza, tiempo de cómputo).
- [x] Exportación de piezas listas para la cama de impresión a Z=0 con sus clavijas de ensamble.
- [x] *Criterio de aceptación:* Ejecución completa fluida en CLI sobre modelos de prueba reales con tiempos de ejecución de milisegundos.
