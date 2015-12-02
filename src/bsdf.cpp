#include "bsdf.h"

#include <assert.h>
#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CMU462 {

  void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
  }

  // Diffuse BSDF //

  Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return albedo * (1.0 / PI);
  }

  Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    *wi = sampler.get_sample();
    *pdf = 1.0 / (2.0 * M_PI);
    return albedo * (1.0 / PI);
  }

  // Mirror BSDF //

  Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    Vector3D _wi;

    reflect(wo, &_wi);
    
    // Because mirror is a delta and these are float values, this 
    // should have a 0% chance of happening.
    //if (wi.x == _wi.x && wi.z == _wi.z && wi.z == _wi.z) return reflectance;

    return Spectrum();
  }

  Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    reflect(wo, wi);
    *pdf = 1;
    return reflectance * (1 / fabs(wi->z));
  }
  
  // Glossy BSDF //

  /*
    Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
    }

    Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    *pdf = 1.0f;
    return reflect(wo, wi, reflectance);
    }
  */

  // Refraction BSDF //

  Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
  }

  Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

    

    return Spectrum();
  }

  // Glass BSDF //

  Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
  }

  Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    // Compute Fresnel coefficient and either reflect or refract based on it.

    if (!refract(wo, wi, ior)) {
      reflect(wo, wi);
      *pdf = 1;

      return reflectance * (1 / fabs(wi->z));;
    } 

    double ni, nt;

    if (wo.z > 0) {
      ni = 1;
      nt = ior;
    } else {
      ni = ior;
      nt = 1;
    }

    double fresnel; 
    double r_par, r_per;

    double cos_i = fabs(wo.z);
    double cos_t = fabs(wi->z);

    r_par = ((nt*cos_i) - (ni*cos_t)) / ((nt*cos_i) + (ni*cos_t));
    r_per = ((ni*cos_i) - (nt*cos_t)) / ((ni*cos_i) + (nt*cos_t));

    fresnel = 0.5 * (r_par*r_par + r_per*r_per);

    float rand = (float)(std::rand()) / RAND_MAX;
    bool refract = rand > ((float)fresnel);

    *pdf = refract ? 1-fresnel : fresnel;
    
    if (!refract) {
      reflect(wo, wi);
      return fresnel * reflectance * (1 / fabs(wi->z));
    }

    double trans_coeff = (1 - fresnel) * (nt / ni) * (nt / ni) / fabs(wi->z);

    return transmittance * trans_coeff;
  }

  void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {  
    *wi = -wo;
    wi->z += 2*wo.z;
  }

  bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

    // TODO:
    // Use Snell's Law to refract wo surface and store result ray in wi.
    // Return false if refraction does not occur due to total internal reflection
    // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
    // ray entering the surface through vacuum.

    double index;
    double cos_t;
    double cos_i;

    cos_i = wo.z;

    if (wo.z > 0) {
      index = 1.d / (double) ior;
    } else {
      index = (double) ior;
    }

    double discr = 1.d - ((index*index) * (1.d - (cos_i*cos_i)));

    if (discr < 0) return false;

    *wi = -wo*index + Vector3D(0, 0, 1)*(cos_i*index + ((cos_i > 0) ? -sqrt(discr) : sqrt(discr)));

    wi->normalize();

    return true;
  }

  // Emission BSDF //

  Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
    return Spectrum();
  }

  Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
    *wi  = sampler.get_sample(pdf);
    return Spectrum();
  }

} // namespace CMU462
