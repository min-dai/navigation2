# constrained_smoother_core

Standalone C++ extraction of the `nav2_constrained_smoother` optimization core with:
- no ROS dependencies,
- no Navigation2 package dependencies,
- Ceres pulled via `FetchContent`,
- tests and a minimal CLI example,
- a live web tuner (`app.py`) for interactive parameter and obstacle tuning.

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

## Run web live tuning session

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

Then open `http://localhost:8501` in a browser.

## Dependencies

### C++ package
- Ceres Solver (fetched in CMake via `FetchContent`)
- Eigen (transitive from Ceres)
- GoogleTest (fetched in CMake when tests are enabled)

### Python live tuner
- streamlit
- numpy
- scipy
- matplotlib
