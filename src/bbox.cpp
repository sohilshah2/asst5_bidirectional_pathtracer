#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

  bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

    // TODO:
    // Implement ray - bounding box intersection test
    // If the ray intersected the bouding box within the range given by
    // t0, t1, update t0 and t1 with the new intersection times.

    double tminx, tmaxx, tminy, tmaxy, tminz, tmaxz;

    double tmin, tmax;

    tminx = (min.x - r.o.x) * r.inv_d.x;
    tmaxx = (max.x - r.o.x) * r.inv_d.x;

    tmin = std::min(tminx, tmaxx);
    tmax = std::max(tminx, tmaxx);

    tminy = (min.y - r.o.y) * r.inv_d.y;
    tmaxy = (max.y - r.o.y) * r.inv_d.y;

    tmin = std::max(tmin, std::min(tminy, tmaxy));
    tmax = std::min(tmax, std::max(tminy, tmaxy));

    tminz = (min.z - r.o.z) * r.inv_d.z;
    tmaxz = (max.z - r.o.z) * r.inv_d.z;

    tmin = std::max(tmin, std::min(tminz, tmaxz));
    tmax = std::min(tmax, std::max(tminz, tmaxz));
    
    if (tmax >= tmin && tmin >= t0) {
      t0 = tmin;
      t1 = tmax;
      return true;
    } else {
      return false;
    }
  }

  void BBox::draw(Color c) const {

    glColor4f(c.r, c.g, c.b, c.a);

    // top
    glBegin(GL_LINE_STRIP);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(max.x, max.y, max.z);
    glEnd();

    // bottom
    glBegin(GL_LINE_STRIP);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, min.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glEnd();

    // side
    glBegin(GL_LINES);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(min.x, min.y, max.z);
    glEnd();

  }

  std::ostream& operator<<(std::ostream& os, const BBox& b) {
    return os << "BBOX(" << b.min << ", " << b.max << ")";
  }

} // namespace CMU462
