#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

    bool Sphere::test(const Ray& r, double& t1, double& t2) const {

      // TODO:
      // Implement ray - sphere intersection test.
      // Return true if there are intersections and writing the
      // smaller of the two intersection times in t1 and the larger in t2.

      // Build quadratic form and solve for time of intersects
  
      // Vector from origin of ray to center point of sphere
      Vector3D oc = r.o - o;
      double a = dot(r.d, r.d);
      double b = 2 * dot(r.d, oc);
      double c = dot(oc, oc) - (this->r * this->r);

      // Calculate descriminant of quadratic
      double des = b*b - 4*a*c;

      if (des < 0) return false;

      double tminus = (-b - sqrt(des)) / (2*a);
      double tplus = (-b + sqrt(des)) / (2*a);

      if (tminus < 0 && tplus < 0) return false; 

      t1 = std::min(tminus, tplus);
      t2 = std::max(tminus, tplus);

      return true;

    }

    bool Sphere::intersect(const Ray& r) const {

      // TODO:
      // Implement ray - sphere intersection.
      // Note that you might want to use the the Sphere::test helper here.

      double t1, t2;
      bool hit = test(r, t1, t2);

      if (!hit) return false;

      if ((t1 >= r.min_t && t1 <= r.max_t) || (t2 >= r.min_t && t2 <= r.max_t))
	return true;

      return false;
    }

    bool Sphere::intersect(const Ray& r, Intersection *i) const {

      // TODO:
      // Implement ray - sphere intersection.
      // Note again that you might want to use the the Sphere::test helper here.
      // When an intersection takes place, the Intersection data should be updated
      // correspondingly.
      double t1, t2;
      bool hit = test(r, t1, t2);

      if (!hit) return false;

      if (t1 >= r.min_t && t1 <= r.max_t) {
	i->t = t1;
	i->n = normal(r.o + t1 * r.d);
	i->primitive = this;
	i->bsdf = get_bsdf();
	return true;
      } else if (t2 >= r.min_t && t2 <= r.max_t) {
	i->t = t2;
	i->n = normal(r.o + t2 * r.d);
	i->primitive = this;
	i->bsdf = get_bsdf();
	return true;
      }

      return false;
    }

    void Sphere::draw(const Color& c) const {
      Misc::draw_sphere_opengl(o, r, c);
    }

    void Sphere::drawOutline(const Color& c) const {
      //Misc::draw_sphere_opengl(o, r, c);
    }


  } // namespace StaticScene
} // namespace CMU462
