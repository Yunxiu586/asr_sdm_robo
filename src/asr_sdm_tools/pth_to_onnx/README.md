# pth_to_onnx

FiveA+ `.pth` to `.onnx` conversion tool.

```bash
python3 src/asr_sdm_tools/pth_to_onnx/export_five_aplus_onnx.py \
  --weights path/to/FIVE_APLUS_epoch97.pth \
  --output path/to/five_aplus_epoch97.onnx
```

The ROS 2 package uses the generated ONNX model at runtime; this exporter is
kept outside the ROS package.
