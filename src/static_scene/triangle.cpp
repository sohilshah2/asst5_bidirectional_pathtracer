#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

    Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
      mesh(mesh), v1(v1), v2(v2), v3(v3) { }

    BBox Triangle::get_bbox() const {

      // TODO:
      // compute the bounding box of the triangle

      Vector3D min, max;
      
      min.x = std::min(mesh->positions[v1].x, std::min(mesh->positions[v3].x, mesh->positions[v2].x));
      min.y = std::min(mesh->positions[v1].y, std::min(mesh->positions[v3].y, mesh->positions[v2].y));
      min.z = std::min(mesh->positions[v1].z, std::min(mesh->positions[v3].z, mesh->positions[v2].z));

      max.x = std::max(mesh->positions[v1].x, std::max(mesh->positions[v3].x, mesh->positions[v2].x));
      max.y = std::max(mesh->positions[v1].y, std::max(mesh->positions[v3].y, mesh->positions[v2].y));
      max.z = std::max(mesh->positions[v1].z, std::max(mesh->positions[v3].z, mesh->positions[v2].z));

      return BBox(min, max);
    }

    bool Triangle::intersect(const Ray& r) const {

      // TODO: implement ray-triangle intersection

      Vector3D e1 = mesh->positions[v2] - mesh->positions[v1];
      Vector3D e2 = mesh->positions[v3] - mesh->positions[v1];
      Vector3D s  = r.o - mesh->positions[v1];
      Vector3D d  = r.d;

      Vector3D sxe2 = cross(e2, s);
      Vector3D e1xd = cross(e1, d);
      double den  = dot(e1xd, e2);

      if (den == 0) return false;

      double u = dot(sxe2,  d) / den;
      double v = dot(e1xd,  s) / den;
      double t = dot(sxe2, e1) / den;

      return u > 0 && v > 0 && (u + v) < 1 && t >= r.min_t && t <= r.max_t;
    }

    bool Triangle::intersect(const Ray& r, Intersection *isect) const {

      // TODO:
      // implement ray-triangle intersection. When an intersection taekes
      // place, the Intersection data should be updated accordingly

      isect->primitive = this;
      isect->bsdf = mesh->get_bsdf();

      Vector3D e1 = mesh->positions[v2] - mesh->positions[v1];
      Vector3D e2 = mesh->positions[v3] - mesh->positions[v1];
      Vector3D s  = r.o - mesh->positions[v1];
      Vector3D d  = r.d;

      Vector3D sxe2 = cross(e2, s);
      Vector3D e1xd = cross(e1, d);
      double den  = dot(e1xd, e2);

      if (den == 0) return false;

      double u = dot(sxe2,  d) / den;
      double v = dot(e1xd,  s) / den;
      double t = dot(sxe2, e1) / den;

      isect->t = t;

      if (u > 0 && v > 0 && (u + v) < 1 && t >= r.min_t && t <= r.max_t) {
	(&r)->max_t = t;
	isect->n = ((1-u-v)*mesh->normals[v1] + u*mesh->normals[v2] + v*mesh->normals[v3]);
	if (dot(isect->n, d) > 0) isect->n = -isect->n;

	return true;
      } else {
	return false;
      }
    }

    void Triangle::draw(const Color& c) const {
      glColor4f(c.r, c.g, c.b, c.a);
      glBegin(GL_TRIANGLES);
      glVertex3d(mesh->positions[v1].x,
		 mesh->positions[v1].y,
		 mesh->positions[v1].z);
      glVertex3d(mesh->positions[v2].x,
		 mesh->positions[v2].y,
		 mesh->positions[v2].z);
      glVertex3d(mesh->positions[v3].x,
		 mesh->positions[v3].y,
		 mesh->positions[v3].z);
      glEnd();
    }

    void Triangle::drawOutline(const Color& c) const {
      glColor4f(c.r, c.g, c.b, c.a);
      glBegin(GL_LINE_LOOP);
      glVertex3d(mesh->positions[v1].x,
		 mesh->positions[v1].y,
		 mesh->positions[v1].z);
      glVertex3d(mesh->positions[v2].x,
		 mesh->positions[v2].y,
		 mesh->positions[v2].z);
      glVertex3d(mesh->positions[v3].x,
		 mesh->positions[v3].y,
		 mesh->positions[v3].z);
      glEnd();
    }



  } // namespace StaticScene
} // namespace CMU462
