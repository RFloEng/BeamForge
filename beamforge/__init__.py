"""BeamForge: start from any BeamNG vehicle, modify it, and bring SVJ data and meshes onto it.

Pure Python, standard library only, so the same modules run natively and in the browser (Pyodide).
  jbeam    lenient jbeam reader and safe expression evaluator
  beamng   vehicles as the game builds them: parts tree, tuning, configurations, geometry
  svj      SVJ bundles, glTF meshes, hardpoints in BeamNG coordinates, base-vs-SVJ comparison
  gltf     minimal glTF 2.0 reader / writer
"""

__version__ = "0.1.0"
