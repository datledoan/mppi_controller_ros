# mppi_controller_ros benchmark

Measures per-call time of `mppi::Optimizer::evalControl()` (bypasses
`MPPIControllerROS`/move_base_flex), swept across `batch_size` and
`time_steps`. Comparable against `mppi_controller_cuda/benchmark`'s CUDA
version (same scenario). `batch_size` is a runtime setting here, so one run
sweeps the whole grid.

Needs a live `roscore` (`ParametersHandler`/`Costmap2DROS` need it):

```bash
roscore &
```

## Build

```bash
catkin build mppi_controller_ros
```

## Run

```bash
source devel/setup.bash
rosrun mppi_controller_ros cpu_sweep_benchmark [warmup] [reps]   # defaults 5 30
rosrun mppi_controller_ros cpu_sweep_benchmark 5 30 > cpu.csv    # redirect is safe, no log noise mixed in
```

`warmup`: calls made but not timed, to let things settle
(caches, MPPI's own warm-started control mean) before measuring. `reps`:
calls actually timed and used for the CSV's mean/median/p95/max.

Default sweep: `batch_size` in {256, 512, 1024, 2048,
4096, 8192, 16384}, `time_steps` in {32, 56, 100} — edit the vectors at the
top of `cpu_sweep_benchmark.cpp`'s `main()` to change the grid. Large
batch_sizes are slow (single CPU core) — e.g. 16384 takes ~1-2s *per call*,
so a full default sweep takes several minutes.

## Output

CSV: `batch_size,time_steps,wall_mean_ms,wall_median_ms,wall_p95_ms,wall_max_ms,cpu_mean_ms`

- `wall_*_ms`: wall-clock time per `evalControl()` call.
- `cpu_mean_ms`: process CPU-time (`getrusage`) for that call — tracks
  `wall_mean_ms` closely since the optimizer is single-threaded (1-core
  measurement, not the CPU's full multi-core capacity).

You may see `[WARN]`/`[ERROR]` lines on the terminal (not in a redirected
file) about "no inflation layer found" — harmless here, since
`cost_consider_footprint` is `false` (point-robot mode) in both this
package's default config and the real deployment, so the code path that
warning is about never runs.
