# Kassel Map Workflow (Optional Extension)

> This document summarizes the reproducible, optional pipeline to generate a CARLA-ready map for **Kassel (Hessen, Germany)** from OpenStreetMap (OSM) data. It complements the main CARLA + ROS2 workflow and remains separate from the core setup.

## Purpose
Create a localized, reproducible map of Kassel to support region-specific simulation experiments and future real–sim transfer studies within the IES group.

## File Formats Encountered
- **OSM**: `.osm`, `.pbf` (source geodata)
- **OpenDRIVE**: `.xodr` (road topology used by CARLA)
- **3D Assets**: `.fbx`, textures (environmental meshes/materials in Unreal Engine)

## Tools for Digital-Twin Enrichment
- **RoadRunner** (recommended by CARLA docs) for lane/traffic detail and export to CARLA-compatible assets.
- **Unreal Editor / CARLA Map Editor / Blender** for additional 3D authoring.

## General Workflow (High-Level)
1. **Acquire Data**  
   Select the Kassel area in OSM or obtain a ready extract (e.g., BBBike). Save as `.osm` or `.pbf`.
2. **Convert to OpenDRIVE**  
   Follow the official CARLA “Building Maps / Importing OpenStreetMap” guidance to convert OSM into `.xodr`.
3. **Import in CARLA**  
   Load the generated OpenDRIVE map in CARLA and verify that the world instantiates successfully.
4. **(Optional) Enrich the Map**  
   Use authoring tools (e.g., RoadRunner) to add 3D assets and details required for stable physics and realistic perception.


## References
- Official CARLA Docs — *Building Maps → Importing OpenStreetMap*  
  <https://carla.readthedocs.io/en/latest/tuto_G_openstreetmap/>
