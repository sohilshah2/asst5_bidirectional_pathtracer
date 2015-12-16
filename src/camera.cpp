#include "camera.h"

#include "CMU462/misc.h"
#include "CMU462/vector3D.h"
#include "CMU462/vector2D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;

namespace CMU462 {

  using Collada::CameraInfo;

  void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
    this->screenW = screenW;
    this->screenH = screenH;
    nClip = info.nClip;
    fClip = info.fClip;
    hFov = info.hFov;
    vFov = info.vFov;

    double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
    ar = static_cast<double>(screenW) / screenH;
    if (ar1 < ar) {
      // hFov is too small
      hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
    } else if (ar1 > ar) {
      // vFov is too small
      vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
    }
    screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));
  }

  void Camera::place(const Vector3D& targetPos, const double phi,
		     const double theta, const double r, const double minR,
		     const double maxR) {
    double r_ = min(max(r, minR), maxR);
    double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
    this->targetPos = targetPos;
    this->phi = phi_;
    this->theta = theta;
    this->r = r_;
    this->minR = minR;
    this->maxR = maxR;
    compute_position();
  }

  void Camera::copy_placement(const Camera& other) {
    pos = other.pos;
    targetPos = other.targetPos;
    phi = other.phi;
    theta = other.theta;
    minR = other.minR;
    maxR = other.maxR;
    c2w = other.c2w;
  }

  void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
    this->screenW = screenW;
    this->screenH = screenH;
    ar = 1.0 * screenW / screenH;
    hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
    vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
  }

  void Camera::move_by(const double dx, const double dy, const double d) {
    const double scaleFactor = d / screenDist;
    const Vector3D& displacement =
      c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);
    pos += displacement;
    targetPos += displacement;
  }

  void Camera::move_forward(const double dist) {
    double newR = min(max(r - dist, minR), maxR);
    pos = targetPos + ((pos - targetPos) * (newR / r));
    r = newR;
  }

  void Camera::rotate_by(const double dPhi, const double dTheta) {
    phi = clamp(phi + dPhi, 0.0, (double) PI);
    theta += dTheta;
    compute_position();
  }

  void Camera::compute_position() {
    double sinPhi = sin(phi);
    if (sinPhi == 0) {
      phi += EPS_F;
      sinPhi = sin(phi);
    }
    const Vector3D dirToCamera(r * sinPhi * sin(theta),
			       r * cos(phi),
			       r * sinPhi * cos(theta));
    pos = targetPos + dirToCamera;
    Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
    Vector3D screenXDir = cross(upVec, dirToCamera);
    screenXDir.normalize();
    Vector3D screenYDir = cross(dirToCamera, screenXDir);
    screenYDir.normalize();

    c2w[0] = screenXDir;
    c2w[1] = screenYDir;
    c2w[2] = dirToCamera.unit();   // camera's view direction is the
    // opposite of of dirToCamera, so
    // directly using dirToCamera as
    // column 2 of the matrix takes [0 0 -1]
    // to the world space view direction
  }

  Ray Camera::generate_ray(double x, double y) const {
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.

    double newX = 0.5 - x;
    double newY = 0.5 - y;

    newX *= tan(radians(this->hFov)/2.f) * 2.f;
    newY *= tan(radians(this->vFov)/2.f) * 2.f;

    Vector3D origin = Vector3D(newX, newY, 1);
    Vector3D pinhole = Vector3D(0, 0, 0);

    origin = this->c2w * origin;
    pinhole = this->c2w * pinhole;

    origin += this->pos;
    pinhole += this->pos;

    Vector3D direction = pinhole - origin;
    direction.normalize();

    Ray r = Ray(origin, direction);

    return r;
  }

  Vector2D Camera::intersect_ray(const Ray &r) const {
    Vector3D dir = r.d;

    Matrix3x3 w2c = this->c2w.T();

    dir = (w2c * dir).unit();
    
    dir = dir * (1.f / dir.z);

    dir.x = -dir.x / (tan(radians(this->hFov)/2.f) * 2.f);
    dir.y = -dir.y / (tan(radians(this->vFov)/2.f) * 2.f);

    dir.x += 0.5;
    dir.y += 0.5;

    return Vector2D(clamp(dir.x, 0.d, 1.d), clamp(dir.y, 0.d, 1.d));
  }

} // namespace CMU462
