# Copilot instructions

## Build and run

- This is a C++20 project built with CMake. From the repository root, configure and build with:
  ```sh
  cmake -S . -B build
  cmake --build build --target drone-simulation
  ```
- Run the executable with `build` as its working directory. The program writes `output.csv` to its current working directory and runs `python ../scripts/plot.py`, which reads that CSV using pandas, matplotlib, and seaborn. With a single-configuration generator, run `./drone-simulation` from `build`; with a multi-configuration generator, run `./Debug/drone-simulation` or `./Release/drone-simulation` from `build` (on Windows, use the `.exe` suffix). Keeping the working directory at `build` ensures the plotting script path resolves to the repository's `scripts` directory.
- There are no test targets or test files, and no lint target or linter configuration is defined.

## Architecture

- `src/main.cpp` owns the simulation setup and time-stepped loop. It advances the command schedule, flight controller, ESCs, motors, and drone plant, then writes the simulation signals to CSV and invokes the plotting script.
- `StepController` in `include/Controller.hpp` and `src/Controller.cpp` turns timestamped `Step` entries into roll, pitch, and yaw rate targets.
- `FlightController` uses the IMU measurements and one `PID` per axis to calculate control offsets, then mixes those offsets with the base motor command into four motor/ESC throttle commands.
- Each `ESC` converts throttle to motor voltage at its update rate. Each `Motor` models rotational response and derives thrust and torque from its speed.
- `Drone` is the plant: it combines the four thrust/torque contributions, uses the inertia tensor and quaternion attitude to integrate angular and translational state, and exposes the simulated state to the controller and output loop.
- `Vector3`, `Matrix3`, and `Quaternion` are small math types implemented in headers under `include/`. `scripts/plot.py` consumes the CSV schema emitted by `main.cpp` to plot targets, state, motor RPM, and PID outputs.

## Codebase conventions

- Public class declarations live in `include/*.hpp`; their implementations live in matching `src/*.cpp` files. The math types are header-only.
- `src/*.cpp` files are discovered by the CMake source glob. Keep new implementation files under `src/` and declarations under `include/`.
- The control and actuator classes maintain their own elapsed-time counters and update only at their configured rates, while the plant advances at the simulation timestep. Preserve that separation when changing update behavior.
- The CSV column names and order are shared between `src/main.cpp` and `scripts/plot.py`; update both sides together when changing logged signals.
