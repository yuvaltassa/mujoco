// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Benchmark and parity harness for the SceneBridge reconciler.
//
// Renders a simulated trajectory offscreen through the mjr compat layer
// (requires MUJOCO_USE_FILAMENT_MJR_COMPAT=ON) and reports per-stage frame
// times: mjv_updateScene (scene generation), mjr_render (SceneBridge update
// and render submission), and mjr_readPixels (render, GPU sync and readback).
// Optionally dumps every 10th frame as a PPM image so that two builds of this
// tool (e.g. before and after a SceneBridge change) can be compared for
// rendering parity.
//
// Usage: reconciler_benchmark <model.xml> <nframes> [dump_prefix]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>

// Defined in mjr_compat.cc; lets us pick an explicit backend, since the
// platform default backend may not support windowless operation.
extern "C" void mjr_makeFilamentContext(const mjModel* m,
                                        const mjrfContextConfig* cfg,
                                        mjrContext* con);

namespace {

constexpr int kWidth = 512;
constexpr int kHeight = 512;
constexpr double kFrameDt = 0.01;  // simulation time between rendered frames

// Serves "filament:" resources (compiled materials, ibl) from the assets
// directory next to the executable, mirroring what the studio launcher does.
std::string g_asset_dir;

struct AssetBlob {
  std::vector<char> bytes;
};

void RegisterAssetProvider(const char* argv0) {
  const std::string exe(argv0);
  const size_t slash = exe.find_last_of('/');
  g_asset_dir = (slash == std::string::npos ? std::string(".")
                                            : exe.substr(0, slash)) +
                "/assets/";

  mjpResourceProvider provider;
  mjp_defaultResourceProvider(&provider);
  provider.prefix = "filament";
  provider.open = [](mjResource* resource) -> int {
    std::string name(resource->name);
    const size_t colon = name.find(':');
    const std::string path = g_asset_dir + name.substr(colon + 1);
    FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) {
      return 0;
    }
    auto* blob = new AssetBlob();
    std::fseek(file, 0, SEEK_END);
    blob->bytes.resize(std::ftell(file));
    std::fseek(file, 0, SEEK_SET);
    if (std::fread(blob->bytes.data(), 1, blob->bytes.size(), file) !=
        blob->bytes.size()) {
      std::fclose(file);
      delete blob;
      return 0;
    }
    std::fclose(file);
    resource->data = blob;
    return static_cast<int>(blob->bytes.size());
  };
  provider.read = [](mjResource* resource, const void** buffer) -> int {
    auto* blob = static_cast<AssetBlob*>(resource->data);
    *buffer = blob->bytes.data();
    return static_cast<int>(blob->bytes.size());
  };
  provider.close = [](mjResource* resource) {
    delete static_cast<AssetBlob*>(resource->data);
    resource->data = nullptr;
  };
  mjp_registerResourceProvider(&provider);
}

double MsSince(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

void WritePpm(const std::string& path, const unsigned char* rgb, int width,
              int height) {
  FILE* file = std::fopen(path.c_str(), "wb");
  if (!file) {
    std::fprintf(stderr, "Failed to open %s\n", path.c_str());
    std::exit(1);
  }
  std::fprintf(file, "P6\n%d %d\n255\n", width, height);
  std::fwrite(rgb, 1, 3 * width * height, file);
  std::fclose(file);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "Usage: %s <model.xml> <nframes> [dump_prefix]\n",
                 argv[0]);
    return 1;
  }
  const int nframes = std::atoi(argv[2]);
  const std::string dump_prefix = argc > 3 ? argv[3] : "";
  RegisterAssetProvider(argv[0]);

  char error[1024];
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Load error: %s\n", error);
    return 1;
  }
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // Enable contact decor so that the per-frame (pooled) path is exercised.
  mjvOption vopt;
  mjv_defaultOption(&vopt);
  vopt.flags[mjVIS_CONTACTPOINT] = 1;
  vopt.flags[mjVIS_CONTACTFORCE] = 1;

  mjvCamera cam;
  mjv_defaultFreeCamera(model, &cam);

  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(model, &scene, 20000);

  mjrContext context;
  mjr_defaultContext(&context);
  mjrfContextConfig cfg;
  mjrf_defaultContextConfig(&cfg);
  cfg.graphics_api = mjGRAPHICS_API_OPENGL;
  mjr_makeFilamentContext(model, &cfg, &context);
  mjr_setBuffer(mjFB_OFFSCREEN, &context);

  mjrRendererInfo info;
  mjr_getRendererInfo(&info);
  std::printf("renderer: %s (%s)\n", info.renderer, info.backend);

  const mjrRect viewport{0, 0, kWidth, kHeight};
  std::vector<unsigned char> rgb(3 * kWidth * kHeight);

  double update_ms = 0, render_ms = 0, read_ms = 0;
  double first_update_ms = 0, first_render_ms = 0;
  int ngeom = 0;

  for (int frame = 0; frame < nframes; ++frame) {
    while (data->time < frame * kFrameDt) {
      mj_step(model, data);
    }

    auto t0 = std::chrono::steady_clock::now();
    mjv_updateScene(model, data, &vopt, nullptr, &cam, mjCAT_ALL, &scene);
    const double dt_update = MsSince(t0);

    auto t1 = std::chrono::steady_clock::now();
    mjr_render(viewport, &scene, &context);
    const double dt_render = MsSince(t1);

    auto t2 = std::chrono::steady_clock::now();
    mjr_readPixels(rgb.data(), nullptr, viewport, &context);
    const double dt_read = MsSince(t2);

    ngeom = scene.ngeom;
    if (frame == 0) {
      // The first frame creates all persistent state; report it separately.
      first_update_ms = dt_update;
      first_render_ms = dt_render;
    } else {
      update_ms += dt_update;
      render_ms += dt_render;
      read_ms += dt_read;
    }

    if (!dump_prefix.empty() && frame % 10 == 0) {
      WritePpm(dump_prefix + std::to_string(frame) + ".ppm", rgb.data(),
               kWidth, kHeight);
    }
  }

  const double n = nframes > 1 ? nframes - 1 : 1;
  std::printf("model: %s  frames: %d  ngeom: %d\n", argv[1], nframes, ngeom);
  std::printf("first frame:   updateScene %7.3f ms  mjr_render %7.3f ms\n",
              first_update_ms, first_render_ms);
  std::printf("steady state:  updateScene %7.3f ms  mjr_render %7.3f ms  "
              "readPixels %7.3f ms\n",
              update_ms / n, render_ms / n, read_ms / n);

  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
