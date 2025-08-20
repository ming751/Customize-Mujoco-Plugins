#include "inspector.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <optional>

namespace mujoco::plugin::inspector {
namespace {

std::optional<std::string> ReadStringAttr(const mjModel* m, int instance,
                                          const char* key) {
  const char* v = mj_getPluginConfig(m, instance, key);
  if (!v || !v[0]) return std::nullopt;
  return std::string(v);
}

std::optional<double> ReadDoubleAttr(const mjModel* m, int instance,
                                     const char* key) {
  const char* v = mj_getPluginConfig(m, instance, key);
  if (!v || !v[0]) return std::nullopt;
  return std::strtod(v, nullptr);
}

}  // namespace

std::unique_ptr<Inspector> Inspector::Create(const mjModel* m, int instance) {
  InspectorConfig cfg;
  cfg.mode = ReadStringAttr(m, instance, "mode");
  cfg.file = ReadStringAttr(m, instance, "file");
  cfg.rate_hz = ReadDoubleAttr(m, instance, "rate").value_or(10.0);

  void* handle = nullptr;
  if (cfg.mode && *cfg.mode == std::string("file")) {
    const char* path = cfg.file ? cfg.file->c_str() : "inspector.log";
    handle = std::fopen(path, "w");
    if (!handle) {
      mju_warning("inspector: failed to open file: %s", path);
      return nullptr;
    }
  }
  return std::unique_ptr<Inspector>(new Inspector(cfg, handle));
}

Inspector::Inspector(InspectorConfig config, void* file_handle)
  : config_(std::move(config)), file_(file_handle) {}

void Inspector::EmitHeaderOnce(const mjModel* m) {
  if (header_emitted_) return;
  EmitLine("# inspector: joints and sensors");
  EmitLine("# joints: name qpos qvel");
  EmitLine("# sensors: name type dim data...");
  header_emitted_ = true;
}

void Inspector::EmitLine(const std::string& line) {
  if (config_.mode && *config_.mode == std::string("file")) {
    std::fprintf(reinterpret_cast<FILE*>(file_), "%s\n", line.c_str());
    std::fflush(reinterpret_cast<FILE*>(file_));
  } else {
    std::fprintf(stdout, "%s\n", line.c_str());
    std::fflush(stdout);
  }
}

void Inspector::EmitJoints(const mjModel* m, const mjData* d) {
  for (int j = 0; j < m->njnt; ++j) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, j);
    int qposadr = m->jnt_qposadr[j];
    int dofadr  = m->jnt_dofadr[j];
    int dofnum = (m->jnt_type[j] == mjJNT_HINGE || m->jnt_type[j] == mjJNT_SLIDE) ? 1
               : (m->jnt_type[j] == mjJNT_BALL ? 3
               : (m->jnt_type[j] == mjJNT_FREE ? 6 : 0));
    // 仅打印标量关节（HINGE/SLIDE）
    if (dofnum == 1) {
      double q = d->qpos[qposadr];
      double v = d->qvel[dofadr];
      EmitLine(std::string("J ") + (name?name:"(noname)") +
               " qpos=" + std::to_string(q) +
               " qvel=" + std::to_string(v));
    }
  }
}

void Inspector::EmitSensors(const mjModel* m, const mjData* d) {
  for (int s = 0; s < m->nsensor; ++s) {
    const char* name = mj_id2name(m, mjOBJ_SENSOR, s);
    int dim = m->sensor_dim[s];
    int adr = m->sensor_adr[s];
    int type = m->sensor_type[s];
    std::string line = std::string("S ") + (name?name:"(noname)") +
                       " type=" + std::to_string(type) +
                       " dim=" + std::to_string(dim) + " data=";
    for (int k = 0; k < dim; ++k) {
      line += (k?",":"") + std::to_string(d->sensordata[adr+k]);
    }
    EmitLine(line);
  }
}

void Inspector::Compute(const mjModel* m, mjData* d, int /*instance*/) {
  // 限速输出
  double period = (config_.rate_hz > 0 ? 1.0/config_.rate_hz : 0.0);
  if (last_emit_time_ >= 0 && period > 0 && d->time < last_emit_time_ + period) {
    return;
  }
  last_emit_time_ = d->time;

  EmitHeaderOnce(m);
  EmitLine(std::string("t=") + std::to_string(d->time));
  EmitJoints(m, d);
  EmitSensors(m, d);
}

void Inspector::RegisterPlugin() {
  mjpPlugin p;
  mjp_defaultPlugin(&p);

  p.name = "sensor_read_publish";
  p.capabilityflags |= mjPLUGIN_PASSIVE;

  static const char* kAttrs[] = {"mode","file","rate"};
  p.nattribute = 3;
  p.attributes = kAttrs;

  p.nstate = +[](const mjModel*, int){ return 0; };

  p.init = +[](const mjModel* m, mjData* d, int instance){
    auto obj = Inspector::Create(m, instance);
    if (!obj) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(obj.release());
    return 0;
  };

  p.reset = +[](const mjModel*, mjtNum*, void*, int){};

  p.destroy = +[](mjData* d, int instance){
    auto* obj = reinterpret_cast<Inspector*>(d->plugin_data[instance]);
    if (obj) {
      // 若打开了文件，则关闭
      // 这里不在类析构里做，是因为插件对象寿命由 MuJoCo 托管
      if (obj->config_.mode && *obj->config_.mode == std::string("file") && obj->file_) {
        std::fclose(reinterpret_cast<FILE*>(obj->file_));
        obj->file_ = nullptr;
      }
    }
    delete obj;
    d->plugin_data[instance] = 0;
  };

  p.compute = +[](const mjModel* m, mjData* d, int instance, int){
    reinterpret_cast<Inspector*>(d->plugin_data[instance])->Compute(m, d, instance);
  };

  mjp_registerPlugin(&p);
}

}  // namespace mujoco::plugin::inspector

