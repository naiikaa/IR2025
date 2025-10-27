# Digital Twin of Kassel — OpenStreetMap to CARLA Workflow

## Overview
This readme documents the first tries to genrate a **digital twin environment of Kassel (Hessen, Germany)** for autonomous-driving simulation and research.  
The workflow integrates **OpenStreetMap (OSM)** data, **OpenDRIVE** conversion, and **CARLA 0.9.15** to produce a reproducible, open-source representation of a real urban area suitable for experimentation with autonomous agents, perception systems, and reinforcement-learning frameworks.

## Workflow Summary
1. **Data Acquisition**  
   A bounded region (~19 km²) covering central Kassel was extracted from OpenStreetMap using standard GIS utilities.  
   The area includes primary and secondary roads, intersections, and representative urban topology.

2. **Format Conversion**  
   The raw OSM dataset was transformed into **OpenDRIVE (.xodr)**, ensuring semantic consistency of lanes, junctions, and elevation data.  
   Lightweight geometry simplification was applied to maintain simulator stability while preserving essential structure.

3. **Integration with CARLA**  
   The generated `.xodr` file was imported into CARLA 0.9.15 using the built-in OpenDRIVE loader.  
   The simulator successfully generated a standalone map instance in which agents and traffic managers can be spawned and controlled automatically.

4. **Validation and Behavior Simulation**  
   Autonomous agents were instantiated using CARLA’s **Traffic Manager** to validate map navigability and intersection logic.  
   Although it is still far from the ultimate goal of having a digital twin of the city of Kassel.

5. **Toward a High-Fidelity Digital Twin**  
   Although the converted map is operational, constructing a **research-grade digital twin** requires additional refinement.  
   Users are encouraged to post-process the OpenDRIVE geometry in specialized editors such as:
   - *RoadRunner* (for lane topology, traffic signs, and signals),
   - *Blender* or *Unreal Editor* (for environmental modeling and textures),
   - *esmini* tools (for validation and coordinate-reference editing).

   These tools could enable accurate surface alignment, visual realism, and integration of high-resolution sensor assets.

## Recommendations
- Employ smaller sub-regions for testing before full-scale rendering to ensure simulator performance.  
- Maintain consistent georeferencing between OSM, OpenDRIVE, and CARLA to prevent coordinate drift.  
- When publishing results or datasets, cite both the OSM data source and the CARLA simulator to ensure reproducibility.

## Notes
This workflow was executed on Ubuntu 22.04 using CARLA 0.9.15 with GPU acceleration.  
All intermediate conversions were performed using open-source toolchains compatible with academic research reproducibility standards.

---

*Prepared as part of the IRLab2025, IES, Uni Kassel, documentation for autonomous-driving by Sören, Nikita, Iman*
