# ProjectileMotionSim

A physics-based projectile trajectory simulator and inverse solver: RK4 integration of gravity, drag, and Magnus (spin) force, plus a Levenberg–Marquardt solver that works backward from a target landing point to the launch conditions needed to reach it.

It was built and proven out for FRC Team 900's 2026 shooter (launching "Fuel" into "The Hub"), but almost none of the actual logic is FRC-specific. The engine (`ProjectilePath.py`) has zero game-specific code, and the rest of the repo is a complete, generic **reference pipeline** for turning a slow per-shot solve into a fast on-device lookup — you can reuse essentially all of it for a different robot, launcher, or projectile by swapping a small set of constants, not by rewriting the pipeline.

## The physics engine — `ProjectilePath.py`

Fully generic. No FRC/game references anywhere in this file.

**`RungeKutta4`** — a plain 4th-order Runge-Kutta integrator for any first-order ODE system `da/dt = f(t, a)`. Takes the derivative function, initial time/state/step size; `.sim(timelen, stop_on_y=...)` steps until a time limit, or (optionally) until a chosen state component crosses a threshold while decreasing — used here to stop once the projectile reaches a target height, but that hook works for any single-axis stopping condition.

**`Projectile`** — forward simulation of one projectile. Models:
- Gravity (`g`, along -y)
- Aerodynamic drag (quadratic in speed, via a `drag_coeff(Reynolds)` callable)
- Magnus lift from spin about the z-axis only, i.e. topspin/backspin (via a `lift_coeff(shear_parameter)` callable)
- Spin decay from rotational drag torque (via a `spin_coeff(Reynolds)` callable)

`radius`, `mass`, `I` (moment of inertia, auto-computed as a solid sphere if not given), `rho`/`mu` (fluid density/viscosity), and all three coefficient functions are constructor arguments — the defaults are simple constants, not a real aerodynamic model, so plug in your own `Cd`/`Cl`/spin-decay curves if you have them.

`trajectory(vx0, vy0, vz0, omega0, n_hat, sx0, sy0, sz0, t0, dt, sim_end_time, stop_on_y)` integrates velocity first, then re-integrates position from that velocity time series, and returns the full time series plus final speed/spin. `plot_solutions(...)` is a plain matplotlib helper (velocity/position vs. time + a 3D path plot) — nothing game-specific.

**`ProjectileSolver`** — the inverse solver. Given a target `(xt, yt, zt)`, it uses Levenberg–Marquardt (damped Gauss-Newton) to iteratively adjust launch speed, `theta`, `phi`, and spin until the simulated landing point matches the target within `tol`. Notable generic features:
- `vx_frame`/`vy_frame`/`vz_frame`/`omega_frame` — an additive "platform velocity/spin" baked in from the start, i.e. this is designed for shoot-on-the-move from the ground up, not bolted on later.
- `fix_speed`/`fix_omega` — solve for angles only, holding speed and/or spin constant, if your launcher can't vary them independently.
- `theta_bounds`/`phi_bounds`/`omega_bounds`/`v_bounds` — clip the solution to whatever your hardware can physically do.
- `clearance_func` — an arbitrary callable added as an extra residual term, for "must clear this obstacle" constraints (see `FuelClearance.py` below for the example implementation of this hook).
- `.levenberg_marquardt()` returns `True`/`False` for whether it actually converged — always check this before trusting `solver.vx`/`.phi`/`.theta`.

## The reference pipeline (FRC900 example numbers)

**`FuelClearance.py`** — one concrete implementation of the `clearance_func` hook: walks the trajectory backward from the target, finds where it crosses the target's horizontal boundary, and applies a smooth soft-penalty (`log(1+exp(...))`, so it's differentiable) if it's too low there at that point. The pattern (find the constraint-crossing point along the trajectory, penalize smoothly) generalizes; the specific numbers (hub size/position, ball diameter) don't.

**`FuelPath.py`** — validation: samples known-good shots from a precomputed table, runs them back through `ProjectileSolver`, and reports a table of solved-vs-expected angles and hit/miss distance. Good template for validating your own solver setup against known data.

**`TrajectorySurfaces/`** — makes the system fast enough for a robot loop:
1. `TrajectorySurface.py` — for a grid of (distance, forward velocity, lateral velocity), runs the LM solver at every grid point and saves the resulting `theta`/`phi` surfaces to `.npz`.
2. `GenerateValidShotProfile.py` — across multiple flywheel speeds (RPS), keeps only the grid points whose solved angles are within the launcher's mechanical limits, producing a table of every physically achievable (distance, velocity, RPS) combination.
3. `FitTrajectorySurface.py` — drops NaNs/outliers from the raw surfaces and fits a 4th-degree polynomial model (`theta`/`phi` as a function of distance and velocity) so the robot can evaluate it in microseconds instead of running LM live.
4. `PlotTrajectorySurfaces.py` — plots the raw surfaces against the fitted polynomial for to validate the polynomial.
5. `LoadSurface.py` — a small utility to inner-join separately-generated phi/theta CSVs by key.

**`LaunchJavaFiles/PolynomialModel.java`** — loads the JSON polynomial coefficients exported by `FitTrajectorySurface.py` and evaluates them on the robot at runtime. Completely generic 2-variable polynomial evaluator; nothing FRC-specific in the class itself.

## What to actually change for a different robot/object

| What | Where | Change it to |
|---|---|---|
| `hub_height`, `xc`/`zc` | `FuelClearance.py`, `FuelPath.py`, `TrajectorySurface.py` | Your target's height/position |
| `hub_half_diag`, `fuel_diameter`, `extra_tolerance` | `FuelClearance.py` | Your target opening size / projectile size |
| `Projectile(0.0762, 0.226796)` | `FuelPath.py`, `TrajectorySurface.py` | Your projectile's radius (m) / mass (kg) |
| `get_frc900_spin_and_speed_from_shooter_rps()` | `FuelPath.py`, `TrajectorySurface.py` | Your actuator's own speed/spin-vs-input curve (this one bakes in FRC900's specific gear ratio and wheel size) |
| `phi_bounds`/`theta_bounds` | `FuelPath.py`, `TrajectorySurface.py`, `GenerateValidShotProfile.py`, `FitTrajectorySurface.py` | Your launcher's actual mechanical range of motion |
| `r_vals`/`vf_vals`/`vl_vals` | `TrajectorySurface.py` | The distance/velocity range you actually expect to operate in |

## Setup

```bash
git clone https://github.com/anshmenghani/ProjectileMotionSim.git
cd ProjectileMotionSim
python3 -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Needs `numpy`, `matplotlib`, `pandas`, `scikit-learn`, `tabulate` — pinned in `requirements.txt`.

## Quick start with just the engine

```python
import ProjectilePath as pp

ball = pp.Projectile(radius=0.05, mass=0.15)  # your object's radius (m), mass (kg)

# Forward sim: given launch velocity/spin, where does it land?
t, vx, vy, vz, v_final, tp, x, y, z, omega_final = ball.trajectory(
    vx0=10, vy0=8, vz0=0, omega0=50, stop_on_y=0
)

# Inverse solve: given a target (xt, yt, zt), find the launch velocity/spin
solver = pp.ProjectileSolver(
    ball, xt=5, yt=0, zt=0,
    vx0=8, vy0=5, vz0=0, omega0=30,   # initial guess
    phi_bounds=(-1.5, 1.5), theta_bounds=(-1.5, 1.5),
)
converged = solver.levenberg_marquardt()
print(converged, solver.vx, solver.vy, solver.vz, solver.omega)
```

## Running the full pipeline

Order: `TrajectorySurface.py` → `GenerateValidShotProfile.py` → `FitTrajectorySurface.py` → `PlotTrajectorySurfaces.py`, with `FuelPath.py` as a spot-check.

Run these from the **parent directory** of this repo, not from inside it — the path strings above are relative to the working directory, not the script location, and assume the repo folder is literally named `ProjectileMotionSim`:

```bash
cd ..
python ProjectileMotionSim/TrajectorySurfaces/TrajectorySurface.py
```

## Worth knowing

### Read `ProjectileMotionSim.pdf` for an in-depth scholarly explanation of the project.

**`theta`/`phi` swap meaning between the engine and the pipeline scripts.** `ProjectileMotionSim.pdf` calls this out directly: the paper's text defines θ as elevation and φ as azimuth, but says the figures (generated from this code) use the opposite, and flags it in bold both times. Concretely: `ProjectilePath.py`'s comments describe `theta` as a "polar angle" and `phi` as "azimuthal" (standard z-up phrasing), but since gravity is along **-y** here, `phi = arctan2(vy, vx)` is what actually behaves as elevation, and `theta = arcsin(vz/v_mag)` behaves as azimuth/lateral aim — which is what all the `TrajectorySurfaces/` bounds (`phi` narrow at 45°–75°, `theta` wide at ~0°–340°) assume. 

## License

MIT — see `LICENSE`.
