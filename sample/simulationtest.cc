// hy-custom: batch dynamics for GDS reference tables (not a benchmark tool).

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

namespace {

const char* kHelp =
    "\n"
    "Usage: simulationtest <model.xml> [options]\n"
    "\n"
    "  model.xml              path to MJCF or .mjb (required)\n"
    "  --nstep=N              number of mj_step calls\n"
    "  --duration=T           simulate duration in seconds (uses model timestep)\n"
    "  --output=FLAGS         mj | efc | stat | full | all | none (default mj)\n"
    "                         combine with '+' e.g. mj+efc+stat\n"
    "  --help                 print this message\n"
    "\n";

int Finish(const char* msg, mjModel* m) {
  if (m) {
    mj_deleteModel(m);
  }
  if (msg) {
    std::printf("%s\n", msg);
  }
  return msg ? EXIT_FAILURE : EXIT_SUCCESS;
}

void SetXmlPath(mjModel* m, const char* path) {
  if (path && path[0]) {
    strncpy(m->xml_path, path, sizeof(m->xml_path) - 1);
    m->xml_path[sizeof(m->xml_path) - 1] = '\0';
  } else {
    m->xml_path[0] = '\0';
  }
}

int ParseOutputFlags(std::string_view flags) {
  if (flags.empty() || flags == "none") {
    return 0;
  }
  if (flags == "all") {
    return MJ_HY_OUT_ALL;
  }

  int mask = 0;
  std::string token;
  auto flush = [&]() {
    if (token.empty()) {
      return;
    }
    if (token == "mj") {
      mask |= MJ_HY_OUT_MJ;
    } else if (token == "efc") {
      mask |= MJ_HY_OUT_EFC;
    } else if (token == "stat") {
      mask |= MJ_HY_OUT_STAT;
    } else if (token == "full") {
      mask |= MJ_HY_OUT_FULL;
    } else if (token == "none") {
      mask = 0;
    } else if (token == "all") {
      mask = MJ_HY_OUT_ALL;
    } else {
      mask = -1;
    }
    token.clear();
  };

  for (char c : flags) {
    if (c == '+' || c == ',') {
      flush();
      if (mask < 0) {
        return -1;
      }
    } else {
      token.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  flush();
  return mask;
}

}  // namespace

int main(int argc, char** argv) {
  const char* model_path = nullptr;
  int nstep = -1;
  double duration = -1.0;
  int output_mask = MJ_HY_OUT_MJ;
  bool have_nstep = false;
  bool have_duration = false;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      return Finish(kHelp, nullptr);
    }
    if (std::strncmp(argv[i], "--nstep=", 8) == 0) {
      nstep = std::atoi(argv[i] + 8);
      have_nstep = true;
      continue;
    }
    if (std::strncmp(argv[i], "--duration=", 11) == 0) {
      duration = std::atof(argv[i] + 11);
      have_duration = true;
      continue;
    }
    if (std::strncmp(argv[i], "--output=", 9) == 0) {
      output_mask = ParseOutputFlags(argv[i] + 9);
      if (output_mask < 0) {
        return Finish("Invalid --output= value", nullptr);
      }
      continue;
    }
    if (argv[i][0] == '-') {
      return Finish("Unknown option (use --help)", nullptr);
    }
    if (!model_path) {
      model_path = argv[i];
    } else {
      return Finish("Unexpected extra argument (use --help)", nullptr);
    }
  }

  if (!model_path) {
    return Finish(kHelp, nullptr);
  }
  if (have_nstep && have_duration) {
    return Finish("Use only one of --nstep or --duration", nullptr);
  }
  if (!have_nstep && !have_duration) {
    return Finish("Missing --nstep or --duration", nullptr);
  }

  char error[1000] = "Could not load model";
  mjModel* m = nullptr;
  const std::string filename(model_path);
  const bool binary = filename.find(".mjb") != std::string::npos;

  if (binary) {
    m = mj_loadModel(model_path, nullptr);
  } else {
    m = mj_loadXML(model_path, nullptr, error, sizeof(error));
  }
  if (!m) {
    return Finish(error, nullptr);
  }

  SetXmlPath(m, model_path);
  m->opt.hy_debug_output = output_mask;

  if (have_duration) {
    if (duration < 0) {
      return Finish("Invalid --duration", m);
    }
    if (m->opt.timestep <= 0) {
      return Finish("Model timestep must be positive for --duration", m);
    }
    nstep = static_cast<int>(duration / m->opt.timestep + 0.5);
    if (nstep < 1) {
      nstep = 1;
    }
  }

  mjData* d = mj_makeData(m);
  if (!d) {
    return Finish("Could not allocate mjData", m);
  }

  std::printf("simulationtest: %d steps at dt=%g, hy_debug_output=0x%x\n",
              nstep, m->opt.timestep, m->opt.hy_debug_output);

  for (int i = 0; i < nstep; ++i) {
    mj_step(m, d);
  }

  std::printf("done: time=%g\n", d->time);

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
