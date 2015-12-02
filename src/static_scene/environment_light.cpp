#include "environment_light.h"
#include <assert.h>

namespace CMU462 { namespace StaticScene {

    EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
      : envMap(envMap) {

      double dt = (PI) / envMap->h;
      double dp = (2*PI) / envMap->w ;

      cdf_theta = std::vector<double>(envMap->h);
      cdf_phi_theta = std::vector<double>(envMap->h * envMap->w);

      p_theta = std::vector<double>(envMap->h);
      p_phi_theta = std::vector<double>(envMap->h * envMap->w);

      total_l_out = 0;

      // Calculate p(phi | theta) and p(theta)
      double sum_ptheta = 0;
      for (int y = 0; y < envMap->h; y++) {
	double theta = dt * ((double)y) + (dp / 2);
	double ptheta = 0;
	for (int x = 0; x < envMap->w; x++) {
	  double phi = dp * ((double)x) + (dp / 2);
	  p_phi_theta[x + envMap->w * y] = (envMap->data[x + envMap->w * y]).illum() * sin(theta);
	  total_l_out += p_phi_theta[x + envMap->w * y] * dp * dt;
	  ptheta += p_phi_theta[x + envMap->w * y];
	}
	for (int x = 0; x < envMap->w; x++) {
	  p_phi_theta[x + envMap->w * y] *= 1 / ptheta;
	}
	sum_ptheta += ptheta;
	p_theta[y] = ptheta;
      }
      for (int y = 0; y < envMap->h; y++) {
	p_theta[y] *= 1 / sum_ptheta;
      }

      // Calculate CDFs
      sum_ptheta = 0;
      for (int y = 0; y < envMap->h; y++) {
	sum_ptheta += p_theta[y];
	cdf_theta[y] = sum_ptheta;
	double sum_phi_theta = 0;
	for (int x = 0; x < envMap->w; x++) {
	  sum_phi_theta += p_phi_theta[x + envMap->w * y];
	  cdf_phi_theta[x + envMap->w * y] = sum_phi_theta;
	}
      }
    }

    Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
					float* distToLight,
					float* pdf) const {

      double Xi1 = (double)(std::rand()) / RAND_MAX;
      double Xi2 = (double)(std::rand()) / RAND_MAX;

      double dt = (PI) / envMap->h;
      double dp = (2*PI) / envMap->w ;

      int x, y;

      *distToLight = INFINITY;

      std::vector<double>::iterator y_idx, x_idx;

      y_idx = std::lower_bound(cdf_theta.begin(), cdf_theta.end(), Xi1);
      y = y_idx - cdf_theta.begin();

      x_idx = std::lower_bound((std::vector<double>::iterator)(&cdf_phi_theta[envMap->w * y]),
			       (std::vector<double>::iterator)(&cdf_phi_theta[envMap->w * (y+1)]), Xi1);
      x = x_idx - (std::vector<double>::iterator)(&cdf_phi_theta[envMap->w * y]);

      double theta = dt * y;
      double phi = dp * x;

      *pdf = envMap->data[x + envMap->w * y].illum() / (total_l_out * dt*dp*sin(theta));

      double zs = sin(theta) * cos(phi);
      double xs = sin(theta) * sin(phi);
      double ys = cos(theta);

      *wi = Vector3D(xs, ys, zs);

      return sample_dir(Ray(p, (*wi).unit()));
    }

    Spectrum EnvironmentLight::sample_dir(const Ray& r) const {

      double theta = acos(r.d.y);
      double phi = PI + atan2(r.d.x, -r.d.z);

      double x = phi / (2*PI);
      double y = theta / (PI);

      double sax = (((double)envMap->w) * x);
      double say = (((double)envMap->h) * y);

      // Bilinear interpolation
      Spectrum sample_0, sample_1, sample_2, sample_3, result;

      double u, v;
      sample_0 = envMap->data[floor(sax) + envMap->w * floor(say)];
      sample_1 = envMap->data[floor(sax) + envMap->w * ceil(say)];
      sample_2 = envMap->data[ceil(sax) + envMap->w * floor(say)];
      sample_3 = envMap->data[ceil(sax) + envMap->w * ceil(say)];

      u = sax - (double)floor(sax);
      v = say - (double)floor(say);

      result = (sample_0*(1-u)*(1-v) + sample_1*(1-u)*(v) + sample_2*(u)*(1-v) + sample_3*(u)*(v));

      if (result.r < 0 || result.g < 0 || result.b < 0) return Spectrum(0,0,0);

      return result;
    }

  } // namespace StaticScene
} // namespace CMU462
