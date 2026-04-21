# constrained_smoother_core

Standalone C++ extraction of the `nav2_constrained_smoother` optimization core with:
- no ROS dependencies,
- no Navigation2 package dependencies,
- Ceres pulled via `FetchContent`,
- tests and a minimal CLI example,
- a local WebSocket dashboard for live path/obstacle tuning.

## Build (C++)

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Run C++ example

```bash
./build/constrained_smoother_example examples/weights.toml
```

## Run local dashboard

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

Then open `dashboard.html` in a browser.

The dashboard connects directly to `ws://127.0.0.1:8765` and sends JSON updates to the local Python optimizer.
For a non-default port, open `dashboard.html?ws=ws://127.0.0.1:8766`.
Obstacle avoidance in the dashboard uses a differentiable rectangle-clearance penalty sampled at path points and along path segments.

Useful CLI knobs:

```bash
python app.py --cost-weight 0.001 --obstacle-x 2.6 --obstacle-width 1.0
```

## Dependencies

### C++ package
- Ceres Solver (fetched in CMake via `FetchContent`)
- Eigen (transitive from Ceres)
- GoogleTest (fetched in CMake when tests are enabled)

### Python dashboard server
- websockets
- numpy
- scipy
