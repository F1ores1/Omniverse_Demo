# Factory Work Cell Simulation

A factory work cell simulation platform: **one configurable cell, interchangeable equipment, decision-grade KPIs**
(throughput, cycle time, utilization, bottleneck). It runs two ways:

| | **DES mode** (headless) | **3D mode** (Omniverse) |
|---|---|---|
| What | Discrete-event simulation in plain Python (`simcore/`) | PhysX + kinematics inside USD Composer (`omniverse/`) |
| Gives | KPIs with 95% confidence intervals, weeks of factory time in about a second | Loads ride conveyors, UR10 robots move on cuMotion trajectories |
| Needs | Any machine with Python 3.10+ | Windows, NVIDIA RTX GPU, Isaac Sim 6.2, a USD Composer build |

You can run **Part A on its own** (5 minutes to set up). Part B (the 3D viewer) takes longer to install; allow an
hour or more the first time.

---

## Part A — DES demo (any machine, ~5 min setup)

### Install

```bash
git clone https://github.com/F1ores1/Omniverse_Demo.git
```

```bash
cd Omniverse_Demo
```

```bash
python -m venv .venv
```

Activate it (Windows PowerShell: `.venv\Scripts\Activate.ps1`; macOS/Linux: `source .venv/bin/activate`), then:

```bash
pip install -r requirements.txt
```

> Every command here runs from the repo root (the folder that holds `simcore/` and `scenarios/`).

### Run it

```bash
python -m simcore.run scenarios/baseline.yaml scenarios/second_cnc.yaml scenarios/forge_press.yaml
```

Expected (about 1 second): the baseline makes **~13.7 good parts/h with the CNC as bottleneck**. Adding a second CNC
gives **~22.9 parts/h (+68%)** and the bottleneck **moves to inspection**. Each run also writes a KPI JSON with its
config hash, seed and engine version to `output/reports/`.

### Prove it is correct

```bash
python -m pytest tests/unit tests/validation -q
```

`tests/validation` checks simulated output against closed-form queueing results (single-server queue, line balance)
within tolerance. `tests/unit/test_usd.py` is skipped without `pxr`; that is expected.

### Change the cell with config only

Scenarios are short YAML files that extend the baseline. Example, `scenarios/second_cnc.yaml`:

```yaml
name: second-cnc
description: Two CNCs in parallel. Nothing else changes.
extends: baseline.yaml
overrides:
  cell.stations.cnc.count: 2
```

Copy one, change a number (process time, MTBF, buffer size, count), re-run. No code change needed.

| Scenario file | Capital question it answers |
|---|---|
| `baseline.yaml` | conveyor → robot → CNC → inspection → conveyor |
| `second_cnc.yaml` | Buy a second CNC? |
| `second_cnc_fast_inspect.yaml` | ...and speed up inspection, since it becomes the new bottleneck? |
| `faster_robot.yaml` | Is a faster robot worth it? (No: it is not the constraint.) |
| `forge_press.yaml` | Replace a step with a forge press |

---

## Part B — 3D mode in Omniverse (Windows + RTX GPU)

### Requirements

- Windows 10/11, NVIDIA RTX GPU (8 GB+ VRAM recommended), a recent NVIDIA Studio/Game Ready driver
- ~60 GB free disk, internet access (robot and conveyor assets stream from NVIDIA's public S3 asset server)
- Git, and [uv](https://docs.astral.sh/uv/) or Python 3.12

### B1. Install Isaac Sim 6.2 (pip) into `C:\isaacsim\env`

```bash
uv venv --python 3.12 C:\isaacsim\env
```

```bash
C:\isaacsim\env\Scripts\activate
```

```bash
uv pip install "isaacsim[all,extscache]==6.2.0" --extra-index-url https://pypi.nvidia.com
```

Run `isaacsim` once in that environment and accept the NVIDIA EULA. The first launch caches shaders and takes a while.
Check NVIDIA's "Isaac Sim pip installation" docs if the package names have changed.

### B2. Build USD Composer from NVIDIA's kit-app-template into `C:\Omniverse\kit-app-template`

```bash
git clone https://github.com/NVIDIA-Omniverse/kit-app-template.git C:\Omniverse\kit-app-template
```

```bash
cd C:\Omniverse\kit-app-template
```

```bash
.\repo.bat template new
```

Choose **Application → USD Composer**, and keep the default name **`my_company.my_usd_composer`** (the launcher looks
for that file). Accept the defaults for the setup extensions. Then:

```bash
.\repo.bat build
```

```bash
.\repo.bat launch
```

Launch it once to confirm it opens, then close it. The demo was built against **Kit 110.4**.

### B3. Launch the demo

Use **Windows PowerShell** (the built-in one, not the Microsoft Store `pwsh`), from the repo folder:

```bash
powershell -ExecutionPolicy Bypass -File scripts\run_composer.ps1
```

If you installed into other folders, pass them in:

```bash
powershell -ExecutionPolicy Bypass -File scripts\run_composer.ps1 -Composer D:\kit-app-template\_build\windows-x86_64\release -Isaac D:\isaacsim\env\Lib\site-packages\isaacsim
```

Also edit `isaac_site_packages` in `omniverse/factory.workcell/config/extension.toml` to match your Isaac Sim path.

Optional install check (opens Composer and prints `[SMOKE] PASS/FAIL` lines in the terminal):

```bash
powershell -ExecutionPolicy Bypass -File scripts\run_composer.ps1 -Smoke
```

### B4. Use it

The **Modular Work Cell** panel opens docked in Composer. Open a layout with **File > Open** from `assets/cells/`:

| Layout | What it shows |
|---|---|
| `Line1.usda` | Basic straight line (the demo storyline cell) |
| `Line2_parallel.usda` | Parallel presses and CNCs with a buffer |
| `Line3_horseshoe.usda` | Line with bends |
| `Line4_assembly.usda` | Assembly line |

Panel sections:

1. **Catalog**: drag equipment onto the floor (or start from **File > New**)
2. **Selected equipment**: edit parameters, rotate, toggle 3D / 2D box
3. **Product flow**: order stations, add **Parallel** units, **Split**
4. **Run 3D (PhysX)**: Play / Pause / Stop, fast forward x5 to x20
5. **Run DES**: runs the same layout headless, then steps the floor event by event (**Run 4 weeks** for long runs)
6. **Bottleneck insights**: for whichever mode ran last

Robot moves are solved with cuMotion before every run. The first solve takes a few seconds per robot.

---

## 5-minute walkthrough

1. **Config is the source of truth.** Show `scenarios/baseline.yaml` (one short file).
2. **Headless KPIs.** Run Part A's command: throughput with CI, cycle time, utilization, **bottleneck = CNC**, judged by
   active time + blocked/starved, not utilization alone.
3. **Capital decision by config only.** `second_cnc.yaml` gives +68% and the bottleneck moves to inspection;
   `faster_robot.yaml` gives +0% (the robot is not the constraint). That is the "don't buy the wrong machine"
   moment. `second_cnc_fast_inspect.yaml` doubles output (+100%).
4. **Same layout in 3D.** In Composer open `assets/cells/Line1.usda`, press **Play** in section 4: loads ride
   conveyors, robots move, floor rectangles show each station's state. Then **Run DES** on the same layout.
5. **Trust.** Run `pytest tests/validation`: the simulation matches the analytical queueing result.

---

## What is in the repo

| Path | Contents |
|---|---|
| `simcore/` | Headless engine: DES loop, equipment models + registry, pydantic config schema, metrics, kinematics |
| `omniverse/factory.workcell/` | Kit extension: panel, 3D PhysX mode, robots, DES floor view |
| `scenarios/` | Hand-written scenario YAML |
| `assets/cells/` | USD layouts (geometry + `factory:*` attributes; behavior stays in Python) |
| `tests/` | `unit/`, `validation/` (headless) and `kit/` (run inside Composer with `-Exec`) |
| `scripts/` | Composer launcher, and the offline study that picks each robot's pedestal position |

`output/` is empty in git: runs and tests write their KPI reports, layouts and screenshots there.
The UR10 robot asset is streamed at runtime from NVIDIA's public asset server, so 3D mode needs internet.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError: simcore` | Run from the folder that contains `simcore/` |
| Composer: `kit.exe` not found | Check the `-Composer` path; it must contain `kit\kit.exe` and `apps\my_company.my_usd_composer.kit` |
| Panel missing / `pydantic` import error in Composer | Fix `isaac_site_packages` in `omniverse/factory.workcell/config/extension.toml` |
| Robots or conveyors appear as empty prims | No internet, or the S3 asset server is blocked by a firewall |
| Edited `simcore` code but Composer runs the old version | Kit reloads the extension on save; if it doesn't, restart Composer |
| Launcher fails to parse | You used the Store `pwsh`; use Windows PowerShell (`powershell.exe`) |

## License and provenance

Code in this repo is original. 3D assets are NVIDIA Isaac Sim / SimReady public assets, streamed or downloaded under
their own licenses. CNC and forge press are simple proxy geometry with the correct footprint. No proprietary data is
included.
