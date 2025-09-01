# Coordinate-Frames-and-Transformation-Matrix
This repository provides Python utilities for preliminary aircraft analysis, including aerodynamic angle calculations (α, β, γ), coordinate transformations in BODY/WORLD frames, construction of reference vectors for velocity and axis visualization, and interactive 3D aircraft visualization using `.stl` models with aerodynamic vectors.


---

# Aircraft Design Utilities ✈️

This repository provides a Python module for **preliminary aircraft analysis**.
It was created as an academic tool to support courses and projects in **aircraft design and flight dynamics**.

Features

* **Aerodynamic Angles**
  Calculate angle of attack (α), sideslip angle (β), and climb angle (γ) from velocity components.

* **Coordinate Transformations**
  Rotation matrices for converting vectors between BODY and WORLD reference frames.

* **Reference Vectors**
  Build unit vectors aligned with velocity and rotated axes for visualization and analysis.

* **Aircraft State Dictionary**
  Generate a structured output including aerodynamic angles, body velocities, angular rates, and attitude.

* **3D Visualization**
  Load `.stl` models and display interactive aircraft plots with aerodynamic vectors (using `matplotlib` and `numpy-stl`).

Requirements

* `numpy`
* `matplotlib`
* `numpy-stl`

Install dependencies with:

```bash
pip install -r requirements.txt
```

Example Usage

```python
from aircraft_util import alpha_beta_from_body, build_vectors, aircraft_state, plot_aircraft_with_vectors

# Example: velocity components in BODY axes
alpha, beta, Vhat_b, V_air_b = alpha_beta_from_body(100, 0, 10)

# Build reference vectors
vecs = build_vectors(Vhat_b, alpha, beta)

# Generate aircraft state dictionary
state = aircraft_state(alpha, beta, 100, 0, 10, 0, 0, 0, 0, 0.1, 0)
print(state["angles"])

# Plot with STL model
plot_aircraft_with_vectors("stl_models/aircraft.stl", alpha, beta, vecs)
```

Context
This code is intended for **educational use** in courses such as *Aircraft Design I* and supports learning about:

* Flight dynamics basics
* Reference frame transformations
* Aerodynamic performance visualization

---


