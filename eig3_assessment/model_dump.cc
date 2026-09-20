// Compile a list of MJCF files and dump every numeric mjModel array, so that
// models compiled by two builds of libmujoco can be compared field by field.
//
//   model_dump <list.txt> <out.bin> [plugin_dir]
//
// Output records: model path, load status, then (name, count, doubles) per field.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

namespace {

void WriteString(std::FILE* f, const std::string& s) {
  uint32_t n = s.size();
  std::fwrite(&n, sizeof(n), 1, f);
  std::fwrite(s.data(), 1, n, f);
}

template <typename T>
void WriteField(std::FILE* f, const char* name, const T* ptr, size_t n) {
  if constexpr (std::is_same_v<T, char>) {
    return;  // names and paths
  } else {
    WriteString(f, name);
    uint64_t count = ptr ? n : 0;
    std::fwrite(&count, sizeof(count), 1, f);
    std::vector<double> buf(count);
    for (size_t i = 0; i < count; i++) buf[i] = (double)ptr[i];
    std::fwrite(buf.data(), sizeof(double), count, f);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: model_dump <list.txt> <out.bin> [plugin_dir]\n");
    return 1;
  }
  if (argc > 3) mj_loadAllPluginLibraries(argv[3], nullptr);

  std::ifstream list(argv[1]);
  std::FILE* out = std::fopen(argv[2], "wb");
  std::string path;
  int nok = 0, nfail = 0;
  while (std::getline(list, path)) {
    if (path.empty()) continue;
    char error[1024] = "";
    mjModel* m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    WriteString(out, path);
    WriteString(out, m ? "" : error);
    if (!m) {
      nfail++;
      continue;
    }
    nok++;

    // sizes
    {
      std::vector<double> sizes;
#define X(name) sizes.push_back((double)m->name);
      MJMODEL_SIZES
#undef X
      WriteField(out, "SIZES", sizes.data(), sizes.size());
    }

    // statistics
    double stat[7] = {(double)m->stat.meaninertia, (double)m->stat.meanmass,
                      (double)m->stat.meansize,    (double)m->stat.extent,
                      (double)m->stat.center[0],   (double)m->stat.center[1],
                      (double)m->stat.center[2]};
    WriteField(out, "stat", stat, 7);

    // all array fields
#undef MJ_M
#define MJ_M(n) m->n
#define X(type, name, nr, nc) \
  WriteField(out, #name, m->name, (size_t)(m->nr) * (size_t)(nc));
    MJMODEL_POINTERS
#undef X
#undef MJ_M
#define MJ_M(n) n

    WriteString(out, "END");
    uint64_t zero = 0;
    std::fwrite(&zero, sizeof(zero), 1, out);
    mj_deleteModel(m);
  }
  std::fclose(out);
  std::fprintf(stderr, "loaded %d, failed %d\n", nok, nfail);
  return 0;
}
