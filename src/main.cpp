
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

//#include "AABB.h"
#include "BVH.h"
#include "math/vec.h"
#include "scene/camera.h"
#include "scene/ray_triangle.h"
#include "scene/scene.h"
#include "scene/sceneloader.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <deque>
#include <limits>
#include <stack>

std::random_device rd;

bool intersect(const tracer::scene &SceneMesh, const tracer::vec3<float> &ori,
               const tracer::vec3<float> &dir, float &t, float &u, float &v,
               size_t &geomID, size_t &primID) {
  for (auto i = 0; i < SceneMesh.geometry.size(); i++) {
    for (auto f = 0; f < SceneMesh.geometry[i].face_index.size(); f++) {
      auto face = SceneMesh.geometry[i].face_index[f];
      if (tracer::intersect_triangle(
              ori, dir, SceneMesh.geometry[i].vertex[face[0]],
              SceneMesh.geometry[i].vertex[face[1]],
              SceneMesh.geometry[i].vertex[face[2]], t, u, v)) {
        geomID = i;
        primID = f;
        // return true;
      }
    }
  }
  // return false;
  return (geomID != -1 && primID != -1);
}

bool occlusion(const tracer::scene &SceneMesh, const tracer::vec3<float> &ori,
               const tracer::vec3<float> &dir, float &t) {
  float u, v;
  for (auto i = 0; i < SceneMesh.geometry.size(); i++) {
    for (auto f = 0; f < SceneMesh.geometry[i].face_index.size(); f++) {
      auto face = SceneMesh.geometry[i].face_index[f];
      if (tracer::intersect_triangle(
              ori, dir, SceneMesh.geometry[i].vertex[face[0]],
              SceneMesh.geometry[i].vertex[face[1]],
              SceneMesh.geometry[i].vertex[face[2]], t, u, v)) {
        return true;
      }
    }
  }
  return false;
}

tracer::vec3<float> *render(int render_height, int render_width,
                            int image_height, int image_width,
                            tracer::scene SceneMesh, tracer::camera cam,
                            tracer::vec3<float> *image,
                            std::uniform_real_distribution<float> distrib) {

  std::mt19937 gen(rd());

  int h = render_height;
  // int w = render_width;
  // for (int h = image_height - 1; h >= render_height; --h) {
  for (int w = render_width; w < image_width; w++) {
    size_t geomID = -1;
    size_t primID = -1;

    auto is = float(w) / (image_width - 1);
    auto it = float(h) / (image_height - 1);
    auto ray = cam.get_ray(is, it);

    float t = std::numeric_limits<float>::max();
    float u = 0;
    float v = 0;
    if (intersect(SceneMesh, ray.origin, ray.dir, t, u, v, geomID, primID)) {
      auto i = geomID;
      auto f = primID;
      auto face = SceneMesh.geometry[i].face_index[f];
      auto N = normalize(cross(SceneMesh.geometry[i].vertex[face[1]] -
                                   SceneMesh.geometry[i].vertex[face[0]],
                               SceneMesh.geometry[i].vertex[face[2]] -
                                   SceneMesh.geometry[i].vertex[face[0]]));

      if (!SceneMesh.geometry[i].normals.empty()) {
        auto N0 = SceneMesh.geometry[i].normals[face[0]];
        auto N1 = SceneMesh.geometry[i].normals[face[1]];
        auto N2 = SceneMesh.geometry[i].normals[face[2]];
        N = normalize(N1 * u + N2 * v + N0 * (1 - u - v));
      }

      for (auto &lightID : SceneMesh.light_sources) {
        auto light = SceneMesh.geometry[lightID];
        /* light.face_index.size(); */
        std::uniform_int_distribution<int> distrib1(0, light.face_index.size() -
                                                           1);

        int faceID = distrib1(gen);
        const auto &v0 = light.vertex[faceID];
        const auto &v1 = light.vertex[faceID];
        const auto &v2 = light.vertex[faceID];

        auto P = v0 + ((v1 - v0) * float(distrib(gen)) +
                       (v2 - v0) * float(distrib(gen)));

        auto hit =
            ray.origin + ray.dir * (t - std::numeric_limits<float>::epsilon());
        auto L = P - hit;

        auto len = tracer::length(L);

        t = len - std::numeric_limits<float>::epsilon();

        L = tracer::normalize(L);

        auto mat = SceneMesh.geometry[i].object_material;
        auto c =
            (mat.ka * 0.5f + mat.ke) / float(SceneMesh.light_sources.size());

        if (occlusion(SceneMesh, hit, L, t))
          continue;

        auto d = dot(N, L);

        if (d <= 0)
          continue;

        auto H = normalize((N + L) * 2.f);

        c = c + (mat.kd * d + mat.ks * pow(dot(N, H), mat.Ns)) /
                    float(SceneMesh.light_sources.size());

        image[h * image_width + w].r += c.r;
        image[h * image_width + w].g += c.g;
        image[h * image_width + w].b += c.b;
      }
    }
  }
  //}
  return image;
}

template <typename IdxType> class abc {};

int main(int argc, char *argv[]) {
  std::string modelname;
  std::string outputname = "output.ppm";
  bool hasEye{false}, hasLook{false};
  tracer::vec3<float> eye(0, 1, 3), look(0, 1, 0);
  tracer::vec2<uint> windowSize(1024, 768);

  for (int arg = 0; arg < argc; arg++) {
    if (std::string(argv[arg]) == "-m") {
      modelname = std::string(argv[arg + 1]);
      arg++;
      continue;
    }
    if (std::string(argv[arg]) == "-o") {
      outputname = std::string(argv[arg + 1]);
      arg++;
      continue;
    }
    if (std::string(argv[arg]) == "-v") {
      char *token = std::strtok(argv[arg + 1], ",");
      int i = 0;
      while (token != NULL) {
        eye[i++] = atof(token);
        token = std::strtok(NULL, ",");
      }

      if (i != 3)
        throw std::runtime_error("Error parsing view");
      hasEye = true;
      arg++;
      continue;
    }
    if (std::string(argv[arg]) == "-l") {
      char *token = std::strtok(argv[arg + 1], ",");
      int i = 0;

      while (token != NULL) {
        look[i++] = atof(token);
        token = std::strtok(NULL, ",");
      }

      if (i != 3)
        throw std::runtime_error("Error parsing view");
      hasLook = true;
      arg++;
      continue;
    }
    if (std::string(argv[arg]) == "-w") {
      char *token = std::strtok(argv[arg + 1], ",");
      int i = 0;

      while (token != NULL) {
        look[i++] = atof(token);
        token = std::strtok(NULL, ",");
      }

      if (i != 2)
        throw std::runtime_error("Error parsing window size");
      hasLook = true;
      arg++;
      continue;
    }
  }

  tracer::scene SceneMesh;
  bool ModelLoaded = false;

  if (modelname != "") {
    SceneMesh = model::loadobj(modelname);
    ModelLoaded = true;
  }

  /* for (auto geo : SceneMesh.geometry) { */
  /*   std::cout << "Material ka R:" << geo.object_material.ka.r; */
  /*   std::cout << "G:" << geo.object_material.ka.r; */
  /*   std::cout << "B:" << geo.object_material.ka.r << std::endl; */
  /* } */

  // create bvh_tree
  /* std::vector<AABB> scene_geom; */

  /* for (auto geom : SceneMesh.geometry) { */
  /*   AABB ab = AABB(); */
  /*   for (auto abc : geom.vertex) */
  /*     ab.extend(abc); */
  /*   scene_geom.push_back(ab); */
  /* } */

  // BVH tree = BVH(scene_geom);

  int image_width = windowSize.x;
  int image_height = windowSize.y;

  tracer::camera cam(eye, look, tracer::vec3<float>(0, 1, 0), 60,
                     float(image_width) / image_height);

  tracer::vec3<float> *image =
      new tracer::vec3<float>[image_height * image_width];

  std::uniform_real_distribution<float> distrib(0, 1.f);
  std::vector<std::thread> threads;

  auto start_time = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < image_height; i++) {
    // Need to add this to make every ray will de dealt by a thread
    // for (int w = 0; w < image_width; w++) {
    int render_height = image_height - i;
    int render_width = 0;

    /* render(render_height, render_width, image_height, image_width, SceneMesh,
     */
    /* cam, image, distrib); */
    threads.push_back(std::thread(render, render_height, render_width,
                                  image_height, image_width, SceneMesh, cam,
                                  image, distrib));
    //}
  }

  for (auto &thr : threads)
    thr.join();

  auto end_time = std::chrono::high_resolution_clock::now();

  std::cerr << "\n\n Duration : "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                     start_time)
                   .count()
            << std::endl;

  std::ofstream file(outputname, std::ios::out);

  file << "P3\n" << image_width << " " << image_height << "\n255\n";
  for (int h = image_height - 1; h >= 0; --h) {
    for (int w = 0; w < image_width; ++w) {
      auto &img = image[h * image_width + w];
      img.r = (img.r > 1.f) ? 1.f : img.r;
      img.g = (img.g > 1.f) ? 1.f : img.g;
      img.b = (img.b > 1.f) ? 1.f : img.b;

      file << int(img.r * 255) << " " << int(img.g * 255) << " "
           << int(img.b * 255) << "\n";
    }
  }
  delete[] image;
  return 0;
}
