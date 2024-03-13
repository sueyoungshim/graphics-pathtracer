#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

int indexOfLargestExtent(const Vector3D& extent) {
  if (extent.x > extent.y) {
    if (extent.x > extent.z) {
      return 0;
    } else {
      return 2;
    }
  } else {
    if (extent.y > extent.z) {
      return 1;
    } else {
      return 2;
    }
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  
  BBox bbox;
  Vector3D centroid_sum;
  for (auto p = start; p != end; p++) {
    if (*p == nullptr) {
        // Handle null pointer (skip or error log)
        continue;
      }
    BBox bb = (*p)->get_bbox();
    centroid_sum += bb.centroid();
    bbox.expand(bb);
  }
  
  size_t primitives_count = std::distance(start, end);

  if (primitives_count <= max_leaf_size) {
    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;
    return node;
  }
  
  Vector3D centroid = centroid_sum / static_cast<double>(primitives_count);

  BBox centroid_bbox;
  for (auto p = start; p != end; ++p) {
    centroid_bbox.expand((*p)->get_bbox().centroid());
  }
  
  int split_axis = indexOfLargestExtent(centroid_bbox.extent);

  auto partition_point = std::partition(start, end, [&](const Primitive* p) {
    return p->get_bbox().centroid()[split_axis] < centroid[split_axis];
  });

  BVHNode *left = construct_bvh(start, partition_point, max_leaf_size);
  BVHNode *right = construct_bvh(partition_point, end, max_leaf_size);

  BVHNode *node = new BVHNode(bbox);
  node->l = left;
  node->r = right;
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  
  if (node == nullptr) {
    return false;
  }
  
  if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
    return false;
  }

  if (node->isLeaf()) {
    for (auto p : primitives) {
      total_isects++;
      if (p->has_intersection(ray))
        return true;
    }
    return false;
  }
  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  if (node == nullptr) {
    return false; // Base case: if node is null, no intersection.
  }
  
  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false; // If the ray misses the bounding box, no need to check further.
  }

  if (node->isLeaf()) {
    bool hit = false; // Keep track of any hit.
    for (auto p = node->start; p != node->end; ++p) {
      total_isects++;
      Intersection temp_isect;
      if ((*p)->intersect(ray, &temp_isect)) {
        hit = true; // Intersection found.
        // Update the closest hit intersection only if it's closer.
        if (temp_isect.t < i->t) {
          *i = temp_isect;
          ray.max_t = temp_isect.t; // Update the max_t to the closest t found.
        }
      }
    }
    return hit; // Return whether we hit anything.
  } else {
    // Internal node: check both children and return true if either does.
    Intersection i_left = *i; // Create a copy for the left child intersection.
    Intersection i_right = *i; // Create a copy for the right child intersection.

    bool hit_left = intersect(ray, &i_left, node->l);
    bool hit_right = intersect(ray, &i_right, node->r);

    if (hit_left && hit_right) {
      if (i_left.t < i_right.t) {
        *i = i_left;
      } else {
        *i = i_right;
      }
      return true;
    } else if (hit_left) {
      *i = i_left;
      return true;
    } else if (hit_right) {
      *i = i_right;
      return true;
    }
    return false; // No hits in either child.
  }
}

} // namespace SceneObjects
} // namespace CGL
