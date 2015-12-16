#include "light.h"

#include <iostream>

#include "../sampler.h"

namespace CMU462 { namespace StaticScene {

    // Directional Light //

    DirectionalLight::DirectionalLight(const Spectrum& rad,
				       const Vector3D& lightDir)
      : radiance(rad) {
      dirToLight = -lightDir.unit();
    }

    Spectrum DirectionalLight::sample_L(const Vector3D& p, Vector3D* wi,
					float* distToLight, float* pdf) const {
      *wi = dirToLight;
      *distToLight = INF_D;
      *pdf = 1.0;
      return radiance;
    }

    Ray DirectionalLight::sample_ray(Spectrum& L_out, float& pdf) const {
      return Ray();
    }

    // Infinite Hemisphere Light //

    InfiniteHemisphereLight::InfiniteHemisphereLight(const Spectrum& rad)
      : radiance(rad) {
      sampleToWorld[0] = Vector3D(1,  0,  0);
      sampleToWorld[1] = Vector3D(0,  0, -1);
      sampleToWorld[2] = Vector3D(0,  1,  0);
    }

    Spectrum InfiniteHemisphereLight::sample_L(const Vector3D& p, Vector3D* wi,
					       float* distToLight,
					       float* pdf) const {

      Vector3D dir = sampler.get_sample();

      *wi = sampleToWorld* dir;
      *distToLight = INF_D;
      *pdf = 1.0 / (2.0 * M_PI);
      return radiance;
    }

    Ray InfiniteHemisphereLight::sample_ray(Spectrum& L_out, float& pdf) const {
      return Ray();
    }

    // Point Light //

    PointLight::PointLight(const Spectrum& rad, const Vector3D& pos) : 
      radiance(rad), position(pos) { }

    Spectrum PointLight::sample_L(const Vector3D& p, Vector3D* wi,
				  float* distToLight,
				  float* pdf) const {

      Vector3D d = position - p;
      *wi = d.unit();
      *distToLight = d.norm();
      *pdf = 1.0;
      return radiance;
    }

    Ray PointLight::sample_ray(Spectrum& L_out, float& pdf) const {
      return Ray();
    }

    // Spot Light //

    SpotLight::SpotLight(const Spectrum& rad, const Vector3D& pos,
			 const Vector3D& dir, float angle) {

    }

    Spectrum SpotLight::sample_L(const Vector3D& p, Vector3D* wi,
				 float* distToLight, float* pdf) const {
      return Spectrum();
    }

    Ray SpotLight::sample_ray(Spectrum& L_out, float& pdf) const {
      return Ray();
    }

    // Area Light //

    AreaLight::AreaLight(const Spectrum& rad, 
			 const Vector3D& pos,   const Vector3D& dir, 
			 const Vector3D& dim_x, const Vector3D& dim_y)
      : radiance(rad), position(pos), direction(dir),
	dim_x(dim_x), dim_y(dim_y), area(dim_x.norm() * dim_y.norm()) { }

    Spectrum AreaLight::sample_L(const Vector3D& p, Vector3D* wi, 
				 float* distToLight, float* pdf) const {

      Vector2D sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);

      Vector3D d = position + sample.x * dim_x + sample.y * dim_y - p;
      float cosTheta = dot(d, direction);
      float sqDist = d.norm2();
      float dist = sqrt(sqDist);
      *wi = d / dist;
      *distToLight = dist;
      *pdf = sqDist / (area * fabs(cosTheta));
      return cosTheta < 0 ? radiance : Spectrum();
    };

    Ray AreaLight::sample_ray(Spectrum& L_out, float& pdf) const {
      Vector2D sample = sampler.get_sample() - Vector2D(0.5f, 0.5f);
      
      Vector3D dir = sampler_hemi.get_sample();
      Matrix3x3 o2w;
      make_coord_space(o2w, direction);
      dir = (o2w * dir).unit();

      Vector3D origin = position + sample.x * dim_x + sample.y * dim_y;

      float cosTheta = dot(dir, direction);
      
      pdf = 1.0 / (area * fabs(cosTheta) * 2.0 * M_PI);
      L_out = radiance;

      return Ray(origin + dir*0.01, dir);
    }


    // Sphere Light //

    SphereLight::SphereLight(const Spectrum& rad, const SphereObject* sphere) {

    }

    Spectrum SphereLight::sample_L(const Vector3D& p, Vector3D* wi, 
				   float* distToLight, float* pdf) const {

      return Spectrum();
    }

    Ray SphereLight::sample_ray(Spectrum& L_out, float& pdf) const {
      return Ray();
    }

    // Mesh Light

    MeshLight::MeshLight(const Spectrum& rad, const Mesh* mesh) {

    }

    Spectrum MeshLight::sample_L(const Vector3D& p, Vector3D* wi, 
				 float* distToLight, float* pdf) const {
      return Spectrum();
    }

    Ray MeshLight::sample_ray(Spectrum& L_out, float& pdf) const {
      return Ray();
    }

  } // namespace StaticScene
} // namespace CMU462
