# Bachelor Thesis  
## Comparative Study of Two-Dimensional Metaheuristic Controllers (PI, PID, Median-Filtered PID)

### Overview
This project presents the design, modeling, and optimization of a two-dimensional (2×2) multivariable control system for a coupled water tank process.

The main objective was to compare different metaheuristically tuned controllers:
- PI Controller
- PID Controller
- Median-Filtered PID Controller

Controller parameters were optimized using Particle Swarm Optimization (PSO) in order to minimize the overall tracking error.

---

## System Description

The controlled plant is a two-input, two-output (MIMO) nonlinear dynamic system.

Steps performed in the project:

1. Data acquisition from the physical process  
2. Parametric system identification (ARX / ARMA models)  
3. Controller design (PI / PID / PID + filtering)  
4. Parameter tuning using PSO  
5. Closed-loop performance comparison  

---
