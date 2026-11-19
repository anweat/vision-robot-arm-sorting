#!/bin/sh
# ONNX -> .om for the Atlas 200I DK A2 (Ascend310B1), on the Mac, in a container. No board needed.
#
#   sh scripts/convert_om.sh [data/models/blocks4/v3]
#
# The .om lands next to the ONNX as blocks4_yolov8n.om, which app/src/vision/blocks_detector.py picks up on
# the board (it needs ais_bench, so off the board it keeps using the ONNX).
#
# The toolkit is the board's own CANN 6.2.RC1 aarch64 copy under ~/Ascend; the image only supplies the
# interpreter and libraries it needs. Three things it got wrong before (2026-09-26):
#   - Python 3.10: the toolkit's own TBE code does `from collections import Iterable`, removed in 3.10.
#     Hence Python 3.9, as on the board.
#   - numpy 2.x: TBE uses `np.float_`, removed in 2.0. Hence numpy<2.
#   - libascend_hal.so: absent without a driver; the toolkit ships a stub in aarch64-linux/devlib.
# Without them atc failed with EZ3002 "Optype [Conv2D] ... unsupported", naming none of the real causes.
set -e
MODEL_DIR=${1:-data/models/blocks4/v3}
TOOLKIT=$HOME/Ascend/cann-6.2.RC1-aarch64/ascend-toolkit
IMAGE=cann-atc:6.2rc1-py39          # built from ~/Ascend/docker/Dockerfile

[ -f "$MODEL_DIR/best.onnx" ] || { echo "no $MODEL_DIR/best.onnx" >&2; exit 1; }
[ -d "$TOOLKIT" ] || { echo "no toolkit at $TOOLKIT" >&2; exit 1; }
DIR=$(cd "$MODEL_DIR" && pwd)

docker run --rm -m 10g \
  -v "$TOOLKIT:/usr/local/Ascend/ascend-toolkit:ro" -v "$DIR:/model" "$IMAGE" sh -c '
  . /usr/local/Ascend/ascend-toolkit/set_env.sh
  export LD_LIBRARY_PATH=/usr/local/Ascend/ascend-toolkit/latest/aarch64-linux/devlib:$LD_LIBRARY_PATH
  export HOME=/tmp TE_PARALLEL_COMPILER=4
  cd /tmp && atc --model=/model/best.onnx --framework=5 --output=/tmp/blocks4_yolov8n \
      --input_format=NCHW --input_shape="images:1,3,640,640" --soc_version=Ascend310B1 --log=error
  cp /tmp/blocks4_yolov8n.om /model/ && chmod 644 /model/blocks4_yolov8n.om'

ls -l "$MODEL_DIR/blocks4_yolov8n.om"
