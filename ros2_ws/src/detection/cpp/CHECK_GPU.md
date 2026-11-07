# check_gpu.cpp: native GPU diagnostics

This executable is the second incremental Python-to-C++ port. It inspects the
machine or container it runs in. It does not modify drivers, power settings,
clocks, Git branches, or model weights. It does not depend on the ROS node.

## Build and run

From the repository root, in a Linux environment with CMake 3.17+, a C++17
compiler, and OpenCV 4 development libraries:

```bash
cmake -S ros2_ws/src/detection/cpp -B /tmp/odlc-cpp-build
cmake --build /tmp/odlc-cpp-build -j2
/tmp/odlc-cpp-build/check_gpu
ctest --test-dir /tmp/odlc-cpp-build --output-on-failure
```

CMake automatically looks for the CUDA Toolkit. If it finds it, the executable
links the CUDA Runtime and, when available, cuBLAS. No `.cu` source or custom CUDA
kernel is needed. If the toolkit is missing, the tool still builds and clearly
reports that native CUDA access was **not tested**. This is not proof that the
machine has no GPU. The existing annotation target does not acquire CUDA as a
dependency.

To specify an installed toolkit, configure with
`-DCUDAToolkit_ROOT=/usr/local/cuda`. To explicitly build without CUDA, use
`-DDETECTION_ENABLE_CUDA_DIAGNOSTICS=OFF`. The toolkit and driver must be compatible;
a container must also have access to the NVIDIA device. A container on an Apple
Mac cannot establish Jetson GPU availability.

Useful options:

```bash
/tmp/odlc-cpp-build/check_gpu --help
/tmp/odlc-cpp-build/check_gpu --skip-benchmark
/tmp/odlc-cpp-build/check_gpu --skip-system-tools
/tmp/odlc-cpp-build/check_gpu --require-gpu
/tmp/odlc-cpp-build/check_gpu --tool-timeout-ms 1000
```

The default returns 0 when the report completes, including on a CPU-only build.
`--require-gpu` returns 1 when GPU 0 cannot be accessed, or when a requested GPU
benchmark fails/cannot run. Combine it with `--skip-benchmark` to require device
access without running the matrix benchmark. Invalid arguments return 2, and
unexpected diagnostic failures return 1. Missing system tools are reported but
do not make the informational report fail.

## Sections of the C++ file

### 1. Options and formatting

The Python script always runs all checks. The native executable adds options for
skipping optional work, bounding external-tool runtime, and requiring a GPU for
automated checks. `Options` stores settings, `parse_options` validates arguments,
and `section` formats report headings.

### 2. System information and build environment

`uname` replaces Python's `platform` calls. Reading `/proc/device-tree/model`
still identifies a Jetson when Linux exposes it. Python's interpreter and
PyTorch version details are replaced by the C++ standard, compiler, OpenCV
version, and whether CUDA/cuBLAS were compiled into this executable.

### 3. CUDA availability, properties, and memory

The native CUDA Runtime replaces `torch.cuda`:

| Python | C++ |
|---|---|
| `torch.cuda.device_count()` | `cudaGetDeviceCount()` |
| Device name/properties | `cudaGetDeviceProperties()` |
| Select CUDA device | `cudaSetDevice()` |
| PyTorch-based memory estimate | `cudaMemGetInfo()` |
| CUDA build information | `cudaRuntimeGetVersion()` and `cudaDriverGetVersion()` |

For each device it reports name, compute capability, multiprocessors, global
memory, and currently available memory. The tool selects GPU 0 for the benchmark
and summary. An error on a different GPU is reported separately. CUDA Runtime
errors are printed rather than being interpreted as a missing Python package.

PyTorch's allocated/reserved cache measurements are not reproduced: this process
does not use PyTorch's allocator. `cudaMemGetInfo` measures the memory view of the
selected device at that moment. On Jetson, memory is shared with the CPU.

### 4. NVIDIA system tools

`posix_spawnp` replaces `subprocess.run` for `nvidia-smi` and
`jetson_clocks --show`. Arguments are passed directly without a shell. The code
captures stdout/stderr, reports exit status, and handles missing commands.

A steady-clock deadline limits each tool (five seconds by default). Output is
capped at 16 KiB per tool. The child runs in its own process group, so timeout
cleanup also terminates descendants holding its output pipe open. Read work per
iteration is bounded so a continuously printing command cannot bypass the timer.
These calls query settings; they do not run `sudo` or change clocks.

### 5. Python framework scope

This reports explicitly that PyTorch, cuDNN, TensorFlow, and Ultralytics Python
packages were not inspected. Importing Python packages would diagnose a different
runtime and make the native tool depend on Python again. The original script
remains available for those checks. A Python installation can be healthy while
native CUDA fails, and vice versa.

### 6. Model-inference scope

The Python script constructs a YOLO object and prints its current device, but
does not explicitly run GPU inference. This C++ tool makes no model claim: it
does not download weights, load `.pt` files, or validate a TensorRT/ONNX model.
Model loading belongs in the later `model_manager` migration.

### 7. Environment variables

`std::getenv` replaces `os.environ` for `CUDA_HOME`, `CUDA_PATH`,
`LD_LIBRARY_PATH`, and `PATH`. Long path values are summarized, as in Python.
A path containing the word CUDA is reported as text presence, not proof of a
working driver or installation.

### 8. Matrix benchmark

OpenCV `cv::gemm` replaces CPU `torch.matmul`. Native `cublasSgemm` replaces GPU
`torch.matmul` when CUDA and cuBLAS were found at build time. Both multiply the
same 1000-by-1000 FP32 matrices, run an untimed warmup, and average three measured
operations. CUDA synchronization ensures GPU work is complete before stopping
the timer. Allocation, transfers, and setup are outside the timed intervals.

OpenCV stores matrices in row-major order; cuBLAS uses column-major order. The
GPU call reverses the two inputs to compute the transpose interpretation of the
same result. The result is copied back and checked for finite values and against
the CPU reference using maximum absolute error divided by maximum CPU magnitude,
with a tolerance of `1e-3`. A successful timing alone is not accepted as a passing
GPU benchmark. Resource cleanup runs on success or exception.

These are native-library timings, not numerical reproductions of the old PyTorch
timings. The report does not assume a slow microbenchmark means a broken GPU and
does not predict YOLO speed from the ratio.

### 9. Summary and exit status

The summary distinguishes compile-time support, actual CUDA device access, and
benchmark verification. Missing `nvidia-smi` does not override a successful native
CUDA query; successful `nvidia-smi` does not override a failed CUDA query. No part
of this tool claims that a model was tested.

## Tests and limitations

`test/check_gpu_test.cmake` exercises the actual executable: help and argument
validation, report sections, no-CUDA behavior, the CPU benchmark, GPU-required
exit status, missing NVIDIA tools, successful/failing tool exit codes, hanging
tools, and continuously printing tools. Fake NVIDIA tools test subprocess
handling; they do not simulate real GPU inference.

Run the tool on the Jetson/NVIDIA deployment machine to verify the real CUDA
path. CPU-only Docker tests cannot establish GPU hardware compatibility or
benchmark correctness on a real NVIDIA GPU. This step leaves the Python script
and existing ROS runtime untouched.

Local verification used GCC 11.4 and OpenCV 4.5.4 in Docker. Both the default
configuration (toolkit absent) and explicitly disabled CUDA configuration build.
The annotation test and new diagnostic test suite both pass. The CUDA/cuBLAS
conditional code was not compiled or run in this environment; validate it on the
deployment machine with the NVIDIA CUDA Toolkit before relying on GPU results.

## API references

- [CUDA Runtime device management](https://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__DEVICE.html)
- [cuBLAS matrix operations](https://docs.nvidia.com/cuda/cublas/)
- [CMake FindCUDAToolkit](https://cmake.org/cmake/help/latest/module/FindCUDAToolkit.html)
