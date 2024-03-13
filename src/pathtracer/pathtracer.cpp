#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  for (int i = 0; i < num_samples; i++) {
    Vector3D wi_local = hemisphereSampler->get_sample();
    Vector3D wi = o2w * wi_local;

    Ray light_ray(hit_p + wi * EPS_F, wi);
    light_ray.min_t = EPS_F;

    Intersection light_isect;
    if (bvh->intersect(light_ray, &light_isect)) {
      Vector3D emission = light_isect.bsdf->get_emission();
      if (emission == Vector3D(0, 0, 0)) continue;
      
      double cos_theta = dot(wi, isect.n);
      L_out += emission * isect.bsdf->f(w_out, wi_local) * cos_theta;
    }
  }
  L_out /= (num_samples * (1.0 / (2.0 * PI)));
  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  for (auto& light : scene->lights) {
    Vector3D wi; // Direction from hit point to light
    double distToLight; // Distance from hit point to light
    double pdf; // Probability density function for the sampled direction
    int num_samples = ns_area_light;
    
    if (light->is_delta_light()) {
      num_samples = 1;
    }
    
    for (int i = 0; i < num_samples; i++) {
      Vector3D radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);
      Vector3D wi_local = w2o * wi;
      
      Ray shadow_ray(hit_p + wi * EPS_F, wi, distToLight - EPS_F);
      shadow_ray.min_t = EPS_F;
      Intersection shadow_isect;
      
      if (!bvh->intersect(shadow_ray, &shadow_isect)) {
        double cosTheta = dot(isect.n, wi);
        if (cosTheta > 0.0 && pdf > 0.0) {
          Vector3D f = isect.bsdf->f(w_out, wi_local);
          L_out += (f * radiance * cosTheta) / pdf;
        }
      }
    }
    L_out /= num_samples;
  }
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  return estimate_direct_lighting_importance(r, isect);
}


Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r, const Intersection &isect) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D w_out = w2o * (-r.d);

    Vector3D L_out(0, 0, 0);

    if (isAccumBounces || max_ray_depth == 0) {
        L_out += one_bounce_radiance(r, isect);
    }

//    if (coin_flip(0.3)) {
        Vector3D w_in_local;
        double pdf;
        Vector3D f = isect.bsdf->sample_f(w_out, &w_in_local, &pdf);

        if (pdf > 0) {
            Vector3D w_in_world = o2w * w_in_local;
            Ray new_ray(hit_p + w_in_world * EPS_F, w_in_world, INFINITY);
            new_ray.depth = r.depth - 1;

            Intersection new_isect;
            if (bvh->intersect(new_ray, &new_isect)) {
                Vector3D L_indirect = at_least_one_bounce_radiance(new_ray, new_isect);
//                L_out += (f * L_indirect * std::abs(w_in_local.z) / (pdf * 0.7));
                L_out += (f * L_indirect * std::abs(w_in_local.z) / (pdf * 0.7));
            }
        }
//    }
    return L_out;
}


//Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
//                                                  const Intersection &isect) {
//  Matrix3x3 o2w;
//  make_coord_space(o2w, isect.n);
//  Matrix3x3 w2o = o2w.T();
//
//  Vector3D hit_p = r.o + r.d * isect.t;
//  Vector3D w_out = w2o * (-r.d);
//
//  Vector3D L_out(0, 0, 0);
//
//  // TODO: Part 4, Task 2
//  // Returns the one bounce radiance + radiance from extra bounces at this point.
//  // Should be called recursively to simulate extra bounces.
//  
//  if (coin_flip(0.3)) {
//    if (r.depth == 0) {
//      return L_out;
//    }
//    
//    if (!isAccumBounces && r.depth == 1) {
//      return one_bounce_radiance(r, isect);
//    }
//
//    if (isAccumBounces) {
//      L_out += one_bounce_radiance(r, isect);
//    }
//
//    Vector3D w_in_local;
//    double pdf;
//    Vector3D f = isect.bsdf->sample_f(w_out, &w_in_local, &pdf);
//
//    if (pdf > 0) {
//      Vector3D w_in_world = o2w * w_in_local;
//
//      Ray new_ray(hit_p + w_in_world * EPS_F, w_in_world, INFINITY);
//      new_ray.depth = r.depth - 1;
//
//      Intersection new_isect;
//      if (bvh->intersect(new_ray, &new_isect)) {
//        Vector3D L_indirect = at_least_one_bounce_radiance(new_ray, new_isect);
//        
//        if (!isAccumBounces && new_ray.depth == 0) {
//          L_out = L_indirect;
//        } else {
//          L_out += f * L_indirect * w_in_local.z / pdf;
//          L_out += f * L_indirect * w_in_local.z / (pdf * 0.7);
//        }
//      }
//    }
//  }
//  return L_out;
//}


Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // TODO (Part 3): Return the direct illumination.
  if (!bvh->intersect(r, &isect)) {
    return envLight ? envLight->sample_dir(r) : Vector3D();
  }
  
  if (isAccumBounces || max_ray_depth == 0) {
    L_out += zero_bounce_radiance(r, isect);
  }
//  L_out += one_bounce_radiance(r, isect);
  L_out += at_least_one_bounce_radiance(r, isect);
  return L_out;
}

//void PathTracer::raytrace_pixel(size_t x, size_t y) {
//  // TODO (Part 1.2):
//  // Make a loop that generates num_samples camera rays and traces them
//  // through the scene. Return the average Vector3D.
//  // You should call est_radiance_global_illumination in this function.
//  
//  int num_samples = ns_aa;
//  Vector3D radiance_sum(0, 0, 0);
//  
//  for (int i = 0; i < num_samples; i++) {
//    Vector2D sample = gridSampler->get_sample();
//    double sample_x = (x + sample.x) / sampleBuffer.w;
//    double sample_y = (y + sample.y) / sampleBuffer.h;
//    
//    Ray ray = camera->generate_ray(sample_x, sample_y);
//    ray.depth = max_ray_depth;
//    Vector3D radiance = est_radiance_global_illumination(ray);
//    radiance_sum += radiance;
//  }
//
//  Vector3D radiance_avg = radiance_sum / num_samples;
//  
//  sampleBuffer.update_pixel(radiance_avg, x, y);
////  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
//  
//
//  // TODO (Part 5):
//  // Modify your implementation to include adaptive sampling.
//  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
////  int num_samples = ns_aa;          // total samples to evaluate
//  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
//
//
////  sampleBuffer.update_pixel(Vector3D(0.2, 1.0, 0.8), x, y);
//  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
//}

float illum_f(Vector3D v) {
    return 0.2126f * v.r + 0.7152f * v.g + 0.0722f * v.b;
}

//
//
//void PathTracer::raytrace_pixel(size_t x, size_t y) {
//  Vector2D origin = Vector2D(x, y);
//  float illum_sum;
//  float illum_sq_sum;
//  int sample_count = 0;
//
//  while (sample_count < ns_aa) {
//    int batch_samples = std::min(samplesPerBatch, ns_aa - sample_count);
//    for (int i = 0; i < batch_samples; ++i) {
//      Vector2D sample = gridSampler->get_sample();
//      double sample_x = (x + sample.x) / double(sampleBuffer.w); // Ensure floating-point division
//      double sample_y = (y + sample.y) / double(sampleBuffer.h); // Ensure floating-point division
//      
//      Ray ray = camera->generate_ray(sample_x, sample_y);
//      ray.depth = max_ray_depth;
//      Vector3D radiance = est_radiance_global_illumination(ray);
//      float illum = illum_f(radiance);
//      
//      illum_sum += illum;
//      illum_sq_sum += illum * illum;
//    }
//    sample_count += batch_samples;
//
//    if (sample_count > 1) {
//      float mean = illum_sum / sample_count;
//      float variance = (illum_sq_sum - (illum_sum * illum_sum) / sample_count) / (sample_count - 1);
//      
//      float I = 1.96 * sqrt(variance / sample_count);
//      
//      if (I <= maxTolerance * mean) {
//        break;
//      }
//    }
//  }
//
//  Vector3D illum_avg = illum_sum / sample_count;
//  sampleBuffer.update_pixel(illum_avg, x, y);
//  sampleCountBuffer[x + y * sampleBuffer.w] = sample_count;
//}


void PathTracer::raytrace_pixel(size_t x, size_t y) {
  Vector2D origin = Vector2D(x, y);
  Vector3D illum_sum(0, 0, 0);
  Vector3D illum_sq_sum(0, 0, 0);
  int sample_count = 0;

  while (sample_count < ns_aa) {
    int batch_samples = std::min(samplesPerBatch, ns_aa - sample_count);
    for (int i = 0; i < batch_samples; ++i) {
      Vector2D sample = gridSampler->get_sample();
      double sample_x = (x + sample.x) / double(sampleBuffer.w);
      double sample_y = (y + sample.y) / double(sampleBuffer.h);
      
      Ray ray = camera->generate_ray(sample_x, sample_y);
      ray.depth = max_ray_depth;
      Vector3D illum = est_radiance_global_illumination(ray);
      
      illum_sum += illum;
      illum_sq_sum += illum * illum;
    }
    sample_count += batch_samples;

    if (sample_count > 1) {
      Vector3D mean = illum_sum / sample_count;
      Vector3D variance = (illum_sq_sum - (illum_sum * illum_sum) / sample_count) / (sample_count - 1);
      
      Vector3D I = 1.96 * Vector3D(sqrt(variance.x), sqrt(variance.y), sqrt(variance.z)) / std::sqrt(sample_count);
      
      if ((I.x <= maxTolerance * mean.x) && (I.y <= maxTolerance * mean.y) && (I.z <= maxTolerance * mean.z)) {
        break;
      }
    }
  }
  Vector3D illum_avg = illum_sum / sample_count;
  sampleBuffer.update_pixel(illum_avg, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = sample_count;
}


void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
