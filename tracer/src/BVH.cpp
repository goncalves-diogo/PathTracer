
#include "AABB.h"
#include <algorithm>
#include <cassert>
#include <future>
#include <memory>
#include <thread>
#include <vector>

struct Node {
  Node() = default;
  Node(Node *x);
  AABB bounds;
  std::vector<const AABB *> objs;
  std::unique_ptr<Node> left = nullptr;
  std::unique_ptr<Node> right = nullptr;
};

Node *bvh_build(std::vector<const AABB *> &objs, int depth) {
  Node *root = new Node();
  auto &bounds = root->bounds;
  for (auto ptr : objs)
    bounds.append(*ptr);
  if (objs.size() < 5) {
    root->left = nullptr;
    root->right = nullptr;
    root->objs = objs;
    return root;
  }
  int dim = 2;
  auto diagonal = bounds.max - bounds.min;
  if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
    dim = 0;
  else if (diagonal.y > diagonal.z)
    dim = 1;
  std::sort(objs.begin(), objs.end(), [dim](auto a, auto b) {
    return a->center()[dim] < b->center()[dim];
  });

  auto b = objs.begin();
  auto m = b + (objs.size() / 2);
  auto e = objs.end();

  auto left = std::vector<const AABB *>(b, m);
  auto right = std::vector<const AABB *>(m, e);

  auto lefty =
      std::async(std::launch::async, &bvh_build, std::ref(left), depth + 1);
  root->left.reset(lefty.get());
  auto righty =
      std::async(std::launch::async, &bvh_build, std::ref(right), depth + 1);
  root->right.reset(righty.get());

  return root;
}

void node_intersect_r(Node *root, const ray &r, std::vector<int> &result) {
  std::vector<AABB> buf;
  if (!root->bounds.intersect(r))
    return;
  if (root->left.get() == nullptr) {
    assert(root->right.get() == nullptr);
    const AABB *start = &buf[0];
    for (auto ptr : root->objs)
      result.emplace_back(ptr - start);
    return;
  }
  node_intersect_r(root->left.get(), r, result);
  node_intersect_r(root->right.get(), r, result);
}

bool node_intersect(Node *root, const ray &r, std::vector<int> &result) {
  result.clear();
  node_intersect_r(root, r, result);
  return result.size();
}
