
#include <chrono>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>

#include "math/vec.h"
#include "scene/camera.h"
#include "scene/ray_triangle.h"
#include "scene/scene.h"
#include "scene/sceneloader.h"

typedef tracer::vec3<float> vec3f;
typedef tracer::ray ray;
const float inf = std::numeric_limits<float>::infinity();

struct AABB {
  vec3f min;
  vec3f max;
  AABB() {
    min = vec3f(inf, inf, inf);
    max = vec3f(-inf, -inf, -inf);
  }
  vec3f center() const {
    return vec3f(0.5f * (min[0] + max[0]), 0.5f * (min[1] + max[1]),
                 0.5f * (min[2] + max[2]));
  }
  void append(const vec3f &v) {
    for (int i = 0; i < 3; i++) {
      min[i] = std::min(min[i], v[i]);
      max[i] = std::max(max[i], v[i]);
    }
  }
  void append(const AABB &x) {
    for (int i = 0; i < 3; i++) {
      min[i] = std::min(min[i], x.min[i]);
      max[i] = std::max(max[i], x.max[i]);
    }
  }

  bool intersect(const tracer::ray &r) const {
    vec3f rmax = vec3f(inf, inf, inf);
    vec3f rmin = vec3f(-inf, -inf, -inf);
    auto &origin = r.origin;
    auto &dir = r.dir;

    for (int i = 0; i < 3; i++) {
      auto d = dir[i];
      if (d != 0) {
        rmin[i] = (min[i] - origin[i]) / d;
        rmax[i] = (max[i] - origin[i]) / d;
        if (d < 0)
          std::swap(rmin[i], rmax[i]);
      }
    }
    float t_min = std::max(std::max(rmin[0], rmin[1]), rmin[2]);
    float t_max = std::min(std::min(rmax[0], rmax[1]), rmax[2]);
    return t_min <= t_max && t_max >= 0;
  }
};
