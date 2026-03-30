# Vision Pipeline Benchmark Results

Produced by `scripts/benchmark_vision.py` on the target hardware.

## Jetson Orin Nano 16 GB Results

**Date:** 2026-03-05  
**Hardware:** Jetson Orin (nvgpu), CUDA 12.6, Driver 540.4.0  
**Model:** best_fallback.engine (TensorRT)  
**Input size:** 640x640  
**Device:** cuda:0

### Inference Time (ms)

| Metric | Value |
|--------|-------|
| Min | 29.50 |
| Avg | 33.31 |
| Max | 39.75 |
| P50 | 33.24 |
| P99 | 39.62 |

### GPU Memory

- **Allocated:** 9.7 MB (approximate)

### Observations

- First inference slower due to TensorRT engine load (~2 s).
- Steady-state ~33 ms avg yields ~30 Hz; sufficient for 10 Hz inspection.
- Target &lt; 10 ms would require smaller input (e.g. 416) or lighter model.

## Notes

- Run `scripts/export_tensorrt.sh` on the Jetson to generate the engine.
- For faster inference, try `--input_size 416` or `wheel_imgsz:=416` in launch.
