#pragma once
#include "AABB.h"
//#include "scene/scene.h"
#include <vector>
#include <memory>

typedef tracer::vec3<float> vec3f;
typedef tracer::ray ray;

class BVH {
public:
  struct node {
    AABB bounds;
    std::vector<const AABB *> geom;
    std::unique_ptr<node> left = nullptr;
    std::unique_ptr<node> right = nullptr;
  };

public:
  BVH(std::vector<AABB> &b);
  bool intersect(const ray &r, std::vector<int> &result);

private:
  void intersect_r(node *root, const ray &r, std::vector<int> &result);
  node *build(std::vector<const AABB *> &geom, int depth = 0);

private:
  std::unique_ptr<node> root;
  std::vector<AABB> buf;
};
