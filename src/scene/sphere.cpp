#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  
  Vector3D oc = r.o - o;
  double a = dot(r.d, r.d);
  double b = 2 * dot(oc, r.d);
  double c = dot(oc, oc) - r2;
  
  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0) {
    return false;
  }

  t1 = (-b - sqrt(discriminant)) / (2 * a);
  t2 = (-b + sqrt(discriminant)) / (2 * a);
  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  if (!test(r, t1, t2)) {
    return false;
  }
  return (t1 >= r.min_t && t1 <= r.max_t) || (t2 >= r.min_t && t2 <= r.max_t);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
  if (!test(r, t1, t2)) {
    return false;
  }

  double t = t1;
  if (t < r.min_t || t > r.max_t) {
    t = t2;
    if (t < r.min_t || t > r.max_t) {
      return false;
    }
  }
  r.max_t = t;

  i->t = t;
  i->n = (r.at_time(t) - o).unit();
  i->primitive = this;
  i->bsdf = get_bsdf();

  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
