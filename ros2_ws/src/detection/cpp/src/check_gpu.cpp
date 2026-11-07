// Native counterpart of detection/check_gpu.py. No Python or ROS runtime needed.
#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <spawn.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

#include <opencv2/core.hpp>

#ifdef DETECTION_CHECK_GPU_CUDA
#include <cuda_runtime_api.h>
#endif
#ifdef DETECTION_CHECK_GPU_CUBLAS
#include <cublas_v2.h>
#endif

extern char** environ;

namespace {
using Clock = std::chrono::steady_clock;
#ifdef DETECTION_CHECK_GPU_CUDA
constexpr double kGiB = 1024.0 * 1024.0 * 1024.0;
#endif

// Section 1: command-line controls and report formatting.
struct Options {
  bool benchmark = true;
  bool system_tools = true;
  bool require_gpu = false;
  int tool_timeout_ms = 5000;
};

void section(const std::string& title) {
  std::cout << '\n' << title << '\n' << std::string(72, '-') << '\n';
}

void help() {
  std::cout << "Usage: check_gpu [options]\n"
      "  --skip-benchmark       Skip matrix multiplication timings\n"
      "  --skip-system-tools    Do not run nvidia-smi or jetson_clocks\n"
      "  --require-gpu          Return 1 if CUDA access or a requested GPU benchmark fails\n"
      "  --tool-timeout-ms N    Per-tool deadline, 50..10000 ms (default 5000)\n"
      "  --help                Show this help\n"
      "Exit codes: 0 = report completed, 1 = diagnostic failure, 2 = invalid arguments.\n";
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--skip-benchmark") options.benchmark = false;
    else if (arg == "--skip-system-tools") options.system_tools = false;
    else if (arg == "--require-gpu") options.require_gpu = true;
    else if (arg == "--tool-timeout-ms") {
      if (++i >= argc) throw std::invalid_argument("Missing timeout value");
      const std::string value = argv[i];
      if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
        throw std::invalid_argument("Timeout must be an integer from 50 to 10000");
      }
      const auto timeout = std::stoll(value);
      if (timeout < 50 || timeout > 10000) {
        throw std::invalid_argument("Timeout must be from 50 to 10000 ms");
      }
      options.tool_timeout_ms = static_cast<int>(timeout);
    } else {
      throw std::invalid_argument("Unknown argument: " + arg);
    }
  }
  return options;
}

// Section 2: system identity and the native build environment.
void print_system() {
  section("1. SYSTEM INFORMATION");
  utsname system{};
  if (uname(&system) == 0) {
    std::cout << "OS: " << system.sysname << ' ' << system.release
              << "\nArchitecture: " << system.machine << '\n';
  } else {
    std::cout << "System information unavailable: " << std::strerror(errno) << '\n';
  }
  std::ifstream file("/proc/device-tree/model", std::ios::binary);
  if (file) {
    std::string model{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    model.erase(std::remove(model.begin(), model.end(), '\0'), model.end());
    std::cout << "Device model: " << model << '\n';
    std::transform(model.begin(), model.end(), model.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (model.find("jetson") != std::string::npos) std::cout << "NVIDIA Jetson detected\n";
  } else {
    std::cout << "Device model: not exposed through /proc/device-tree/model\n";
  }
  section("2. C++ BUILD ENVIRONMENT");
  std::cout << "C++ standard: " << __cplusplus << "\nCompiler: " << __VERSION__
            << "\nOpenCV: " << CV_VERSION << '\n';
#ifdef DETECTION_CHECK_GPU_CUDA
  std::cout << "CUDA support compiled in: YES\n";
#else
  std::cout << "CUDA support compiled in: NO\n";
#endif
#ifdef DETECTION_CHECK_GPU_CUBLAS
  std::cout << "cuBLAS benchmark compiled in: YES\n";
#else
  std::cout << "cuBLAS benchmark compiled in: NO\n";
#endif
}

// Section 3: ask CUDA directly, rather than inferring GPU access from a Python package.
struct GpuStatus {
  bool available = false;
  int count = 0;
  std::string reason;
};

#ifdef DETECTION_CHECK_GPU_CUDA
void cuda_check(cudaError_t code, const char* operation) {
  if (code != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
  }
}

std::string cuda_version(int version) {
  return std::to_string(version / 1000) + "." + std::to_string((version % 1000) / 10);
}
#endif

GpuStatus print_cuda() {
  section("3. CUDA DEVICES AND MEMORY");
  GpuStatus status;
#ifdef DETECTION_CHECK_GPU_CUDA
  int version = 0;
  if (cudaRuntimeGetVersion(&version) == cudaSuccess) {
    std::cout << "CUDA runtime version: " << cuda_version(version) << '\n';
  }
  if (cudaDriverGetVersion(&version) == cudaSuccess) {
    std::cout << "Driver-supported CUDA API version: " << cuda_version(version) << '\n';
  }
  const auto result = cudaGetDeviceCount(&status.count);
  if (result != cudaSuccess) {
    status.reason = std::string("CUDA device query failed: ") + cudaGetErrorString(result);
  } else if (status.count == 0) {
    status.reason = "CUDA reports no visible NVIDIA devices";
  } else {
    std::cout << "Visible CUDA devices: " << status.count << '\n';
    for (int i = 0; i < status.count; ++i) {
      try {
        cudaDeviceProp props{};
        cuda_check(cudaGetDeviceProperties(&props, i), "Read device properties");
        cuda_check(cudaSetDevice(i), "Select device");
        std::size_t free_bytes = 0, total_bytes = 0;
        cuda_check(cudaMemGetInfo(&free_bytes, &total_bytes), "Query device memory");
        std::cout << "GPU " << i << ": " << props.name
            << "\n  Compute capability: " << props.major << '.' << props.minor
            << "\n  Multiprocessors: " << props.multiProcessorCount
            << "\n  Total global memory: " << props.totalGlobalMem / kGiB << " GiB"
            << "\n  Current free/total memory: " << free_bytes / kGiB << " / "
            << total_bytes / kGiB << " GiB\n";
        if (i == 0) status.available = true;
      } catch (const std::exception& error) {
        std::cout << "GPU " << i << ": " << error.what() << '\n';
        if (i == 0) status.reason = error.what();
      }
    }
    if (status.available) {
      const auto selected = cudaSetDevice(0);
      if (selected != cudaSuccess) {
        status.available = false;
        status.reason = cudaGetErrorString(selected);
      }
    }
  }
#else
  status.reason = "This executable was built without CUDA support; GPU access was not tested";
#endif
  if (!status.available) std::cout << status.reason << '\n';
  return status;
}

// Section 4: bounded, shell-free subprocess capture for read-only NVIDIA tools.
struct ToolResult {
  int exit_code = -1;
  bool timed_out = false;
  std::string output;
  std::string error;
};

ToolResult run_tool(const std::vector<std::string>& args, int timeout_ms) {
  ToolResult result;
  int descriptors[2];
  if (pipe(descriptors) != 0) {
    result.error = std::strerror(errno);
    return result;
  }
  const int read_fd = descriptors[0], write_fd = descriptors[1];
  if (fcntl(read_fd, F_SETFL, O_NONBLOCK) < 0) {
    result.error = std::strerror(errno);
    close(read_fd); close(write_fd);
    return result;
  }
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attributes;
  const int action_status = posix_spawn_file_actions_init(&actions);
  const int attr_status = posix_spawnattr_init(&attributes);
  int code = action_status != 0 ? action_status : attr_status;
  if (code == 0) code = posix_spawn_file_actions_adddup2(&actions, write_fd, STDOUT_FILENO);
  if (code == 0) code = posix_spawn_file_actions_adddup2(&actions, write_fd, STDERR_FILENO);
  if (code == 0) code = posix_spawn_file_actions_addclose(&actions, read_fd);
  if (code == 0) code = posix_spawn_file_actions_addclose(&actions, write_fd);
  if (code == 0) code = posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
  if (code == 0) code = posix_spawnattr_setpgroup(&attributes, 0);
  std::vector<char*> argv;
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);
  pid_t child = -1;
  if (code == 0) code = posix_spawnp(&child, argv[0], &actions, &attributes, argv.data(), environ);
  if (action_status == 0) posix_spawn_file_actions_destroy(&actions);
  if (attr_status == 0) posix_spawnattr_destroy(&attributes);
  close(write_fd);
  if (code != 0) {
    close(read_fd);
    result.error = std::strerror(code);
    return result;
  }
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  int wait_status = 0;
  bool exited = false, eof = false;
  std::array<char, 2048> buffer{};
  constexpr std::size_t max_output = 16384;
  while (!exited || !eof) {
    // Limit reads per iteration so a noisy child cannot defeat the deadline.
    for (int i = 0; i < 16 && !eof; ++i) {
      const auto bytes = read(read_fd, buffer.data(), buffer.size());
      if (bytes > 0) {
        result.output.append(buffer.data(), std::min<std::size_t>(
            static_cast<std::size_t>(bytes), max_output - result.output.size()));
      } else if (bytes == 0) eof = true;
      else if (errno != EINTR) break;
    }
    if (!exited) {
      const auto waited = waitpid(child, &wait_status, WNOHANG);
      exited = waited == child;
      if (waited < 0 && errno != EINTR) {
        result.error = std::strerror(errno);
        break;
      }
    }
    if (exited && eof) break;
    if (Clock::now() >= deadline) {
      result.timed_out = true;
      kill(-child, SIGKILL);  // Include descendants that inherited the output pipe.
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  close(read_fd);
  if (!exited) {
    kill(-child, SIGKILL);
    pid_t waited;
    do { waited = waitpid(child, &wait_status, 0); } while (waited < 0 && errno == EINTR);
  }
  if (WIFEXITED(wait_status)) result.exit_code = WEXITSTATUS(wait_status);
  else if (WIFSIGNALED(wait_status)) result.exit_code = 128 + WTERMSIG(wait_status);
  return result;
}

void print_tools(const Options& options) {
  section("4. NVIDIA SYSTEM TOOLS");
  if (!options.system_tools) {
    std::cout << "System tools skipped by request\n";
    return;
  }
  for (const auto& args : std::vector<std::vector<std::string>>{
      {"nvidia-smi"}, {"jetson_clocks", "--show"}}) {
    const auto result = run_tool(args, options.tool_timeout_ms);
    std::cout << args.front() << ": ";
    if (!result.error.empty()) std::cout << "unavailable: " << result.error;
    else if (result.timed_out) std::cout << "timed out after " << options.tool_timeout_ms << " ms";
    else std::cout << "exit " << result.exit_code;
    std::cout << '\n';
    if (!result.output.empty()) std::cout << result.output << '\n';
  }
  std::cout << "Tool output does not establish that this executable can use CUDA.\n";
}

// Sections 5-7: distinguish native diagnostics from Python/model validation.
void print_scope_and_environment() {
  section("5. PYTHON FRAMEWORKS");
  std::cout << "PyTorch, cuDNN, TensorFlow, and Ultralytics Python packages are not inspected.\n"
               "Run the original check_gpu.py to diagnose that Python environment.\n";
  section("6. MODEL INFERENCE");
  std::cout << "No YOLO weights are loaded or downloaded. Model inference is not tested.\n"
               "CUDA availability alone does not verify a TensorRT engine or ONNX model.\n";
  section("7. ENVIRONMENT VARIABLES");
  for (const std::string name : {"CUDA_HOME", "CUDA_PATH", "LD_LIBRARY_PATH", "PATH"}) {
    const char* raw = std::getenv(name.c_str());
    std::string value = raw ? raw : "Not set";
    if ((name == "PATH" || name == "LD_LIBRARY_PATH") && value.size() > 100) {
      auto lower = value;
      std::transform(lower.begin(), lower.end(), lower.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      value = lower.find("cuda") != std::string::npos ? "Set (contains CUDA text)" : "Set (no CUDA text)";
    }
    std::cout << name << ": " << value << '\n';
  }
}

#ifdef DETECTION_CHECK_GPU_CUBLAS
void blas_check(cublasStatus_t code, const char* operation) {
  if (code != CUBLAS_STATUS_SUCCESS) {
    throw std::runtime_error(std::string(operation) + ": cuBLAS status " + std::to_string(code));
  }
}

// Resources are released even when allocation, GEMM, or result validation fails.
struct GpuBenchmark {
  float *a = nullptr, *b = nullptr, *c = nullptr;
  cublasHandle_t handle = nullptr;
  ~GpuBenchmark() {
    if (handle) cublasDestroy(handle);
    if (c) cudaFree(c);
    if (b) cudaFree(b);
    if (a) cudaFree(a);
  }
};
#endif

// Section 8: warm up, time repeated FP32 multiplies, and check GPU results.
bool print_benchmark(const Options& options, const GpuStatus& gpu) {
  section("8. QUICK BENCHMARK");
  if (!options.benchmark) {
    std::cout << "Benchmark skipped by request\n";
    return true;
  }
  constexpr int size = 1000;
  constexpr int iterations = 3;
  cv::Mat a(size, size, CV_32F), b(size, size, CV_32F), cpu;
  cv::RNG rng(42);
  rng.fill(a, cv::RNG::UNIFORM, -1.0, 1.0);
  rng.fill(b, cv::RNG::UNIFORM, -1.0, 1.0);
  cv::gemm(a, b, 1, cv::Mat(), 0, cpu);  // CPU warmup.
  const auto cpu_start = Clock::now();
  for (int i = 0; i < iterations; ++i) cv::gemm(a, b, 1, cv::Mat(), 0, cpu);
  const double cpu_ms = std::chrono::duration<double, std::milli>(Clock::now() - cpu_start).count() / iterations;
  if (!cv::checkRange(cpu)) throw std::runtime_error("CPU benchmark produced nonfinite output");
  std::cout << "CPU FP32 1000x1000 matmul, mean of 3: " << cpu_ms << " ms\n";
  if (!gpu.available) {
    std::cout << "GPU benchmark skipped: " << gpu.reason << '\n';
    return false;
  }
#ifdef DETECTION_CHECK_GPU_CUBLAS
  try {
    cuda_check(cudaSetDevice(0), "Select benchmark device");
    GpuBenchmark resources;
    const std::size_t bytes = a.total() * a.elemSize();
    cuda_check(cudaMalloc(reinterpret_cast<void**>(&resources.a), bytes), "Allocate matrix A");
    cuda_check(cudaMalloc(reinterpret_cast<void**>(&resources.b), bytes), "Allocate matrix B");
    cuda_check(cudaMalloc(reinterpret_cast<void**>(&resources.c), bytes), "Allocate matrix C");
    blas_check(cublasCreate(&resources.handle), "Create cuBLAS handle");
    blas_check(cublasSetMathMode(resources.handle, CUBLAS_DEFAULT_MATH), "Select FP32 math");
    cuda_check(cudaMemcpy(resources.a, a.ptr<float>(), bytes, cudaMemcpyHostToDevice), "Copy matrix A");
    cuda_check(cudaMemcpy(resources.b, b.ptr<float>(), bytes, cudaMemcpyHostToDevice), "Copy matrix B");
    const float alpha = 1.0F, beta = 0.0F;
    auto multiply = [&] {
      // OpenCV is row-major; cuBLAS is column-major. C^T = B^T * A^T.
      blas_check(cublasSgemm(resources.handle, CUBLAS_OP_N, CUBLAS_OP_N,
          size, size, size, &alpha, resources.b, size, resources.a, size,
          &beta, resources.c, size), "Multiply matrices");
    };
    multiply();  // GPU warmup, outside the measured interval.
    cuda_check(cudaDeviceSynchronize(), "Synchronize warmup");
    const auto start = Clock::now();
    for (int i = 0; i < iterations; ++i) multiply();
    cuda_check(cudaDeviceSynchronize(), "Synchronize benchmark");
    const double gpu_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count() / iterations;
    cv::Mat actual(size, size, CV_32F);
    cuda_check(cudaMemcpy(actual.ptr<float>(), resources.c, bytes, cudaMemcpyDeviceToHost), "Copy result");
    const double relative_error = cv::norm(actual, cpu, cv::NORM_INF) /
                                  std::max(1.0, cv::norm(cpu, cv::NORM_INF));
    if (!cv::checkRange(actual) || !std::isfinite(relative_error) || relative_error > 1e-3) {
      throw std::runtime_error("GPU result disagrees with CPU reference");
    }
    std::cout << "GPU 0 FP32 matmul, mean of 3: " << gpu_ms << " ms\n"
              << "Maximum error / CPU maximum magnitude: " << std::scientific << relative_error
              << std::fixed << '\n';
    if (gpu_ms > 0) std::cout << "CPU/GPU time ratio: " << cpu_ms / gpu_ms << "x\n";
    std::cout << "GPU matrix result verified. Transfers and setup excluded from timings.\n"
                 "This benchmark does not predict YOLO inference speed.\n";
    return true;
  } catch (const std::exception& error) {
    std::cout << "GPU benchmark failed: " << error.what() << '\n';
    return false;
  }
#else
  std::cout << "GPU benchmark skipped: cuBLAS support was not compiled in\n";
  return false;
#endif
}
}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help") { help(); return 0; }
  }
  Options options;
  try { options = parse_options(argc, argv); }
  catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    help();
    return 2;
  }
  try {
    std::cout.imbue(std::locale::classic());
    std::cout << std::fixed << std::setprecision(2)
              << "GPU DETECTION DIAGNOSTIC TOOL (C++)\n" << std::string(72, '=') << '\n';
    print_system();
    const auto gpu = print_cuda();
    print_tools(options);
    print_scope_and_environment();
    const bool benchmark_ok = print_benchmark(options, gpu);
    section("SUMMARY");
    if (gpu.available) {
      std::cout << "CUDA device access available on GPU 0; recommended device: cuda:0\n";
      if (options.benchmark) std::cout << "GPU benchmark: " << (benchmark_ok ? "PASS" : "NOT VERIFIED") << '\n';
    } else {
      std::cout << "CUDA device access NOT VERIFIED: " << gpu.reason << '\n';
      std::cout << "For native CUDA checks, build with the CUDA Toolkit and expose the NVIDIA GPU.\n";
    }
    std::cout << "This reports the current host/container only; model inference is not verified.\n";
    return options.require_gpu && (!gpu.available || !benchmark_ok) ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << "Diagnostic failed: " << error.what() << '\n';
    return 1;
  }
}
