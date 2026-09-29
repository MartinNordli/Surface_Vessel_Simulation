# Running the simulator

Everything goes through the `scripts/njord` wrapper, which selects the right
Compose overlays (including WSL) and stamps run provenance. Run
`./scripts/njord --help` for the full usage.

## Requirements

- x86-64 Ubuntu 24.04 host, or WSL2
- Docker Engine with Compose v2.24 or newer, Git and Python 3
- For GPU rendering: NVIDIA Container Toolkit and a working NVIDIA host driver

`sudo bash scripts/setup-host.sh` installs Docker and the toolkit, **never a GPU
kernel driver**.

Docker commands need access to the Docker socket. Use `sudo` or your Docker group
configuration. If group membership was just added, start a fresh login or use
`sg docker -c './scripts/njord demo'` in the existing shell.

## Getting the image

CI publishes a tested image for every commit pushed to GitHub, so a new machine
downloads it instead of compiling Gazebo and VRX:

```bash
./scripts/njord pull              # image CI built and tested for this exact commit
./scripts/njord pull latest       # newest main image
./scripts/njord pull v1.0.0       # a release tag
```

`pull` downloads `ghcr.io/martinnordli/surface_vessel_simulation:sha-<commit>`
and tags it `njord-sim:local`; every other command uses that tag. Pull again after
`git pull` or `git checkout`.

Commands pin one immutable image ID and reject a missing or mismatched executable
digest. Editable YAML under `njord_sim/config` may change without rebuilding.
`./scripts/njord check-image`, `test` and CI additionally require the full source
digest, including YAML. There is no stale-image validation bypass.

An image exists only after the CI run for that commit has passed. For unpushed
commits or local changes, build instead with `./scripts/njord build`. An uncached
build compiles the Gazebo vendor packages and VRX and takes a long time.
Scenarios are copied into the image, so rebuild after adding or changing one;
`./scripts/njord build simulator` updates the shared image used by all three race
services.

## Commands

| Command | What it does |
|---|---|
| `doctor` | Docker, Compose, NVIDIA and image checks |
| `test` | Full test suite (CPU + ROS transport/model tests) in the container |
| `selftest` | Starts simulator and autonomy, checks live cameras, lidar and navigation, then stops |
| `demo [course]` | Headless race with simulator, autonomy and evaluator |
| `gui [course]` | Same race with the Gazebo window and RViz |
| `lab [course]` | Free driving (`RUN_MODE=free`): simulator, estimation and selected reference nodes, no evaluator; the boat drives without a race |
| `benchmark [course]` | Seed × environment × profile matrix, e.g. `--jobs 2` or `--dry-run` |
| `film [course]` | Headless race filmed by a chase camera, then a sped-up GIF (see [Filming](#filming)) |
| `simulator` | Simulator service only |
| `smoke` | `validation/check_runtime.py` against an already running stack |

### Races

`demo` creates a fresh timestamped `outputs/run-*` directory, waits for valid
perception, planning, navigation and contact monitoring, then starts race timing.
It stops all services on completion or failure, and exits nonzero for a failed
race. Generated SDF, URDF, bridge configuration, the resolved scenario and
metrics are kept in the output directory.

```bash
./scripts/njord demo                                   # reference course (default)
./scripts/njord demo slalom
SEED=1 ENVIRONMENT=moderate PROFILE=fast ./scripts/njord demo slalom
./scripts/njord benchmark slalom --dry-run
```

Courses are the files in `scenarios/`:

- **`reference`**: three gates and two additional obstacles.
- **`slalom`**: five alternating gates and five obstacles. It is a shared test
  environment for the control/autonomy and perception teams to compare algorithms
  and sensor configurations. The reference autonomy is a baseline and is not
  required to complete every run.
- **`dynamics`**: open water for dynamics measurements, not a race.

All support `SEED`, `ENVIRONMENT` (`calm`, `moderate`) and `PROFILE` (a
`speed_profiles_mps` name in `algorithms.yaml`: `conservative` or `fast` as
shipped). An explicit course name overrides `SCENARIO` for that
invocation; without one, `SCENARIO` (a path inside the container) still works.
Additional arguments follow the course name, for example
`./scripts/njord demo slalom recorder`.

To add a course, copy a file in `scenarios/`, edit it (format in
[configuration.md](configuration.md#scenario-files-scenarioscourseyaml)) and run
`./scripts/njord build`; it is then available as `./scripts/njord demo <file name>`.
To use a different vessel or tuning, see [configuration.md](configuration.md).

## Rendering

The default simulator uses OGRE2 with headless EGL rendering, and NVIDIA graphics
capabilities are passed to the container.

- **WSL2**: `scripts/njord` automatically adds `compose.wsl.yaml`, mounting
  WSLg/DXG and selecting Mesa D3D12 on the NVIDIA adapter. WSL uses the Windows
  driver; do not install Linux NVIDIA kernel modules inside WSL.
- **No GPU**: `NJORD_CPU=1` selects Mesa software rendering (`compose.cpu.yaml`)
  and skips the NVIDIA checks. It renders the real sensors, slowly, and is what CI
  uses. It does not replace a GPU for rendering-performance or benchmark claims.

```bash
NJORD_CPU=1 ./scripts/njord doctor
NJORD_CPU=1 ./scripts/njord selftest
```

`nvidia-smi` passing alone does not establish rendering: inspect Gazebo's
`~/.gz/rendering/ogre2.log` and run the live smoke test.

The GUI overlay uses the existing X display and Xauthority; it never runs
`xhost +`. A separate RViz service is available with
`docker compose --profile gui up rviz`, using the same Compose files, ROS domain
and Gazebo partition as the simulator.

## Manual Compose sessions

To keep an interactive simulator running and inspect it from another terminal:

```bash
export COMPOSE_FILE=compose.yaml:compose.wsl.yaml  # WSL; native Linux: compose.yaml
export RUN_ID=$(python3 -c 'import uuid; print(uuid.uuid4().hex)')
export OUTPUT_HOST=./outputs/manual-$RUN_ID
docker compose up simulator autonomy evaluator
# Another terminal, same Compose configuration:
./scripts/njord smoke
# Finish:
docker compose down
```

## Recording

Add `compose.wsl.yaml` to the list on WSL:

```bash
COMPOSE_FILE=compose.yaml:compose.record.yaml ./scripts/njord demo recorder
```

This writes compressed MCAP bags under the run directory, including `/clock`, TF,
sensors, navigation, commands and evaluation topics. `recording.json` records
topic selection and image/source provenance. Existing bags are never overwritten.
See [team integration](team-integration.md#7-record-and-compare-trials) for replay.

## Filming

The animation at the top of the README is made with:

```bash
./scripts/njord film slalom --output docs/media/slalom.gif
```

This runs the normal headless race plus a `filmer` service (`compose.film.yaml`,
`scripts/run_filmer.py`). The filmer spawns a camera into the world and moves it
behind and above the vessel, following its ground-truth pose. The camera is
presentation only: it is not on the vessel and is not bridged to ROS, so autonomy
never sees it. It adds a rendered 960×540 camera at 10 Hz, so the race can run
slower than real time; simulation time is unaffected. Frames and `frames.jsonl`
land in `<run>/film/`.

Only when the evaluator reports a completed course does `scripts/make_film_gif.py`
run on the host (Python 3 with numpy and Pillow). It skips the wait before the
vessel moves, plays at 20× simulation time and 12 frames per second, and uses one
shared palette. `--speed`, `--fps`, `--width` and `--colors` change this. A
race can fail on a heavily loaded host; the reference slalom does not complete
every run, so rerun it rather than committing a GIF of a failed race.

## Continuous integration and delivery

`.github/workflows/ci.yml` runs on every push, tag and fork pull request:

1. `unit` runs the host unit suite without Docker.
2. `image` builds the image with a layer cache in GHCR, confirms that its source
   digest matches the commit, runs `./scripts/njord test` and a CPU-rendered
   `./scripts/njord selftest`, then publishes the image. Tags are `sha-<commit>`
   for every push, `latest` for `main` and the tag name for `v*` tags. Pull
   requests from forks are tested but not published.

Images are only published after the tests pass. CI proves that the image builds,
that the container suite passes and that live sensors and navigation start with
software rendering. It does not test GPU rendering, race completion or benchmarks.
