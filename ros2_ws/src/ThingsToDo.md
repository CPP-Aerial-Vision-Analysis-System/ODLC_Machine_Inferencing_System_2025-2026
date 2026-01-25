we have protobuf 4.x installed but ONNX + Ultralytics + TensorRT requires protobuf ≤ 3.20.x

How to fix BOTH errors correctly
Step 1 — Disable Ultralytics auto-dependency installs (important)

You do not want this running on Jetson.

Set this environment variable (add to ~/.bashrc too):

export ULTRALYTICS_NO_AUTOINSTALL=1


Reload:

source ~/.bashrc


This stops:

pip installs

dependency churn

silent environment corruption

Step 2 — Fix protobuf (mandatory)

Run exactly this:

pip uninstall protobuf -y
pip install protobuf==3.20.3


Verify:

python3 - << 'EOF'
import google.protobuf
print(google.protobuf.__version__)
EOF


It must print something like:

3.20.3


If it prints 4.x → stop, it’s not fixed.

Step 3 — Install ONNX the Jetson-safe way
Option A (recommended on Jetson)
sudo apt update
sudo apt install python3-onnx