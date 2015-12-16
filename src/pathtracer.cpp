#include "pathtracer.h"
#include "bsdf.h"
#include "ray.h"

#include <stack>
#include <random>
#include <algorithm>
#include <assert.h>

#include "CMU462/CMU462.h"
#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/lodepng.h"

#include "GL/glew.h"

#include "static_scene/sphere.h"
#include "static_scene/triangle.h"
#include "static_scene/light.h"

using namespace CMU462::StaticScene;

using std::min;
using std::max;

namespace CMU462 {

  //#define ENABLE_RAY_LOGGING 1

  PathTracer::PathTracer(size_t ns_aa,
			 size_t max_ray_depth, size_t ns_area_light,
			 size_t ns_diff, size_t ns_glsy, size_t ns_refr,
			 size_t num_threads, HDRImageBuffer* envmap) {
    state = INIT,
      this->ns_aa = ns_aa;
    this->max_ray_depth = max_ray_depth;
    this->ns_area_light = ns_area_light;
    this->ns_diff = ns_diff;
    this->ns_glsy = ns_diff;
    this->ns_refr = ns_refr;

    if (envmap) {
      this->envLight = new EnvironmentLight(envmap);
    } else {
      this->envLight = NULL;
    }

    bvh = NULL;
    scene = NULL;
    camera = NULL;

    gridSampler = new UniformGridSampler2D();
    hemisphereSampler = new UniformHemisphereSampler3D();

    show_rays = true;

    imageTileSize = 32;
    numWorkerThreads = num_threads;
    workerThreads.resize(numWorkerThreads);

    tm_gamma = 2.2f;
    tm_level = 1.0f;
    tm_key = 0.18;
    tm_wht = 5.0f;

  }

  PathTracer::~PathTracer() {

    delete bvh;
    delete gridSampler;
    delete hemisphereSampler;

  }

  void PathTracer::set_scene(Scene *scene) {

    if (state != INIT) {
      return;
    }

    if (this->scene != nullptr) {
      delete scene;
      delete bvh;
      selectionHistory.pop();
    }

    if (this->envLight != nullptr) {
      scene->lights.push_back(this->envLight);
    }

    this->scene = scene;
    build_accel();

    if (has_valid_configuration()) {
      state = READY;
    }
  }

  void PathTracer::set_camera(Camera *camera) {

    if (state != INIT) {
      return;
    }

    this->camera = camera;
    if (has_valid_configuration()) {
      state = READY;
    }

  }

  void PathTracer::set_frame_size(size_t width, size_t height) {
    if (state != INIT && state != READY) {
      stop();
    }
    sampleBuffer.resize(width, height);
    frameBuffer.resize(width, height);
    if (has_valid_configuration()) {
      state = READY;
    }
  }

  bool PathTracer::has_valid_configuration() {
    return scene && camera && gridSampler && hemisphereSampler &&
      (!sampleBuffer.is_empty());
  }

  void PathTracer::update_screen() {
    switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      visualize_accel();
      break;
    case RENDERING:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
    case DONE:
      //sampleBuffer.tonemap(frameBuffer, tm_gamma, tm_level, tm_key, tm_wht);
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
    }
  }

  void PathTracer::stop() {
    switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      while (selectionHistory.size() > 1) {
        selectionHistory.pop();
      }
      state = READY;
      break;
    case RENDERING:
      continueRaytracing = false;
    case DONE:
      for (int i=0; i<numWorkerThreads; i++) {
	workerThreads[i]->join();
	delete workerThreads[i];
      }
      state = READY;
      break;
    }
  }

  void PathTracer::clear() {
    if (state != READY) return;
    delete bvh;
    bvh = NULL;
    scene = NULL;
    camera = NULL;
    selectionHistory.pop();
    sampleBuffer.resize(0, 0);
    frameBuffer.resize(0, 0);
    state = INIT;
  }

  void PathTracer::start_visualizing() {
    if (state != READY) {
      return;
    }
    state = VISUALIZE;
  }

  void PathTracer::start_raytracing() {
    if (state != READY) return;

    rayLog.clear();
    workQueue.clear();

    state = RENDERING;
    continueRaytracing = true;
    workerDoneCount = 0;

    sampleBuffer.clear();
    frameBuffer.clear();
    num_tiles_w = sampleBuffer.w / imageTileSize + 1;
    num_tiles_h = sampleBuffer.h / imageTileSize + 1;
    tile_samples.resize(num_tiles_w * num_tiles_h);
    memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

    // populate the tile work queue
    for (size_t y = 0; y < sampleBuffer.h; y += imageTileSize) {
      for (size_t x = 0; x < sampleBuffer.w; x += imageTileSize) {
	workQueue.put_work(WorkItem(x, y, imageTileSize, imageTileSize));
      }
    }

    // launch threads
    fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);
    for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i] = new std::thread(&PathTracer::worker_thread, this);
    }
  }


  void PathTracer::build_accel() {

    // collect primitives //
    fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
    timer.start();
    vector<Primitive *> primitives;
    for (SceneObject *obj : scene->objects) {
      const vector<Primitive *> &obj_prims = obj->get_primitives();
      primitives.reserve(primitives.size() + obj_prims.size());
      primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
    }
    timer.stop();
    fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

    // build BVH //
    fprintf(stdout, "[PathTracer] Building BVH... "); fflush(stdout);
    timer.start();
    bvh = new BVHAccel(primitives);
    timer.stop();
    fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

    // initial visualization //
    selectionHistory.push(bvh->get_root());
  }

  void PathTracer::log_ray_miss(const Ray& r) {
    rayLog.push_back(LoggedRay(r, -1.0));
  }

  void PathTracer::log_ray_hit(const Ray& r, double hit_t) {
    rayLog.push_back(LoggedRay(r, hit_t));
  }

  void PathTracer::visualize_accel() const {

    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glLineWidth(1);
    glEnable(GL_DEPTH_TEST);

    // hardcoded color settings
    Color cnode = Color(.5, .5, .5, .25);
    Color cnode_hl = Color(1., .25, .0, .6);
    Color cnode_hl_child = Color(1., 1., 1., .6);

    Color cprim_hl_left = Color(.6, .6, 1., 1);
    Color cprim_hl_right = Color(.8, .8, 1., 1);
    Color cprim_hl_edges = Color(0., 0., 0., 0.5);

    BVHNode *selected = selectionHistory.top();

    // render solid geometry (with depth offset)
    glPolygonOffset(1.0, 1.0);
    glEnable(GL_POLYGON_OFFSET_FILL);

    if (selected->isLeaf()) {
      for (size_t i = 0; i < selected->range; ++i) {
	bvh->primitives[selected->start + i]->draw(cprim_hl_left);
      }
    } else {
      if (selected->l) {
	BVHNode* child = selected->l;
	for (size_t i = 0; i < child->range; ++i) {
	  bvh->primitives[child->start + i]->draw(cprim_hl_left);
	}
      }
      if (selected->r) {
	BVHNode* child = selected->r;
	for (size_t i = 0; i < child->range; ++i) {
	  bvh->primitives[child->start + i]->draw(cprim_hl_right);
	}
      }
    }

    glDisable(GL_POLYGON_OFFSET_FILL);

    // draw geometry outline
    for (size_t i = 0; i < selected->range; ++i) {
      bvh->primitives[selected->start + i]->drawOutline(cprim_hl_edges);
    }

    // keep depth buffer check enabled so that mesh occluded bboxes, but
    // disable depth write so that bboxes don't occlude each other.
    glDepthMask(GL_FALSE);

    // create traversal stack
    stack<BVHNode *> tstack;

    // push initial traversal data
    tstack.push(bvh->get_root());

    // draw all BVH bboxes with non-highlighted color
    while (!tstack.empty()) {

      BVHNode *current = tstack.top();
      tstack.pop();

      current->bb.draw(cnode);
      if (current->l) tstack.push(current->l);
      if (current->r) tstack.push(current->r);
    }

    // draw selected node bbox and primitives
    if (selected->l) selected->l->bb.draw(cnode_hl_child);
    if (selected->r) selected->r->bb.draw(cnode_hl_child);

    glLineWidth(3.f);
    selected->bb.draw(cnode_hl);

    // now perform visualization of the rays
    if (show_rays) {
      glLineWidth(1.f);
      glBegin(GL_LINES);

      for (size_t i=0; i<rayLog.size(); i+=500) {

	const static double VERY_LONG = 10e4;
	double ray_t = VERY_LONG;

	// color rays that are hits yellow
	// and rays this miss all geometry red
	if (rayLog[i].hit_t >= 0.0) {
	  ray_t = rayLog[i].hit_t;
	  glColor4f(1.f, 1.f, 0.f, 0.1f);
	} else {
	  glColor4f(1.f, 0.f, 0.f, 0.1f);
	}

	Vector3D end = rayLog[i].o + ray_t * rayLog[i].d;

	glVertex3f(rayLog[i].o[0], rayLog[i].o[1], rayLog[i].o[2]);
	glVertex3f(end[0], end[1], end[2]);
      }
      glEnd();
    }

    glDepthMask(GL_TRUE);
    glPopAttrib();
  }

  void PathTracer::key_press(int key) {

    BVHNode *current = selectionHistory.top();
    switch (key) {
    case ']':
      ns_aa *=2;
      printf("Samples per pixel changed to %lu\n", ns_aa);
      //tm_key = clamp(tm_key + 0.02f, 0.0f, 1.0f);
      break;
    case '[':
      //tm_key = clamp(tm_key - 0.02f, 0.0f, 1.0f);
      ns_aa /=2;
      if (ns_aa < 1) ns_aa = 1;
      printf("Samples per pixel changed to %lu\n", ns_aa);
      break;
    case KEYBOARD_UP:
      if (current != bvh->get_root()) {
	selectionHistory.pop();
      }
      break;
    case KEYBOARD_LEFT:
      if (current->l) {
	selectionHistory.push(current->l);
      }
      break;
    case KEYBOARD_RIGHT:
      if (current->l) {
	selectionHistory.push(current->r);
      }
      break;
    case 'a':
    case 'A':
      show_rays = !show_rays;
    default:
      return;
    }
  }

  Spectrum PathTracer::trace_camera_ray(const Ray &r, const struct LightPath& p, int bounce, bool includeLe) {

    Intersection isect;

    if (!bvh->intersect(r, &isect)) {  
    
      if (!this->envLight) return Spectrum(0,0,0);

      Spectrum light = this->envLight->sample_dir(r);
      
      return light;
    }

    Spectrum L_out = includeLe ? isect.bsdf->get_emission() : Spectrum();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D hit_n = (isect.n).unit();

    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);

    Matrix3x3 w2o = o2w.T();

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    Vector3D w_out = w2o * (r.o - hit_p);
    w_out.normalize();

    Vector3D dir_to_light;
    float dist_to_light;
    float pdf;

    for (SceneLight* light : scene->lights) {
      Spectrum L_temp = Spectrum(0, 0, 0);
      // no need to take multiple samples from a directional source
      int num_light_samples = light->is_delta_light() ? 1 : ns_area_light;

      // integrate light over the hemisphere about the normal
      double scale = 1.0 / num_light_samples;
      for (int i=0; i<num_light_samples; i++) {

	// returns a vector 'dir_to_light' that is a direction from
	// point hit_p to the point on the light source.  It also returns
	// the distance from point x to this point on the light source.
	// (pdf is the probability of randomly selecting the random
	// sample point on the light source -- more on this in part 2)
	Spectrum light_L = light->sample_L(hit_p, &dir_to_light, &dist_to_light, &pdf);

	// convert direction into coordinate space of the surface, where
	// the surface normal is [0 0 1]
	Vector3D w_in = w2o * dir_to_light;

	// note that computing dot(n,w_in) is simple
	// in surface coordinates since the normal is [0 0 1]
	double cos_theta = std::max(0.0, w_in[2]);

	// evaluate surface bsdf
	Spectrum f = isect.bsdf->f(w_out, w_in);

	// Construct a shadow ray and compute whether the intersected surface is
	// in shadow and accumulate reflected radiance
	Vector3D origin = hit_p + EPS_D * dir_to_light;
	Ray shadow = Ray(origin, dir_to_light);
	Intersection shadow_isect;
	shadow.max_t = dist_to_light;
	if (!(bvh->intersect(shadow, &shadow_isect))) {
	  L_temp += light_L * f * cos_theta * (1/pdf);
	}
      }
      if (num_light_samples) L_out += L_temp * scale;
      //if (p.length || bounce) L_out = L_out * (1.f / (p.length + bounce));
    }

    // Compute bidirectional lighting by combining with path from light
    for (int index = 0; index < p.length; index++) {
      if (p.pdf[index] > 0) {
	
	Spectrum illum_bidir = p.illum[index];
	float pdf_bidir = p.pdf[index];
	Intersection inter_bidir = p.isect[index];
	Ray ray_bidir = p.ray[index];
	Vector3D hit_bidir = ray_bidir.o + ray_bidir.d * inter_bidir.t;
	Vector3D dir_bidir = (hit_bidir - hit_p).unit();
	
	if (isect.bsdf->is_delta() || inter_bidir.bsdf->is_delta()) continue;

	double distance = (hit_p - hit_bidir).norm() - EPS_D;

	Vector3D origin = hit_p + EPS_D * dir_bidir;
	Ray shadow = Ray(origin, dir_bidir);
	Intersection shadow_isect;
	shadow.max_t = distance;
	if (!(bvh->intersect(shadow, &shadow_isect))) {
	  Vector3D w_in = (w2o * dir_bidir).unit();
	  double cos_theta = std::max(0.0, w_in[2]);
	  Spectrum f_origin = isect.bsdf->f(w_out.unit(), w_in);
	  f_origin = f_origin * cos_theta;

	  Matrix3x3 o2w_bidir;
	  make_coord_space(o2w_bidir, inter_bidir.n);
      
	  Matrix3x3 w2o_bidir = o2w_bidir.T();
      
	  Vector3D w_out_bidir = (w2o_bidir * (ray_bidir.o - hit_bidir)).unit();
	  w_in = w2o_bidir * (-dir_bidir);
	  cos_theta = std::max(0.0, w_in[2]);
	  Spectrum f_target = inter_bidir.bsdf->f(w_out_bidir, w_in);
	  f_target = f_target * cos_theta;
	
	  L_out += f_origin * f_target * illum_bidir * (1.f / (p.length + bounce));
	}
      }
    }

    // compute an indirect lighting estimate using pathtracing with Monte Carlo.
    // Note that Ray objects have a depth field now; you should use this to avoid
    // traveling down one path forever.
    if (++bounce >= max_ray_depth) return L_out;

    // Random direction to shoot ray in
    Vector3D wi;    
    Spectrum light_bounce = isect.bsdf->sample_f(w_out, &wi, &pdf);

    float term_prob;
    term_prob = (1.f - clamp(light_bounce.illum(), 0, 1)) * 0.65;

    if (((float)(std::rand()) / RAND_MAX) < term_prob) return L_out;

    wi = (o2w * wi).unit();
    Ray indirect_ray = Ray(hit_p + wi*EPS_D, wi);
    
    return L_out + (light_bounce * trace_camera_ray(indirect_ray, p, bounce, isect.bsdf->is_delta())
		    * fabs(dot(wi, hit_n))) * (1.f / (pdf * (1.f-term_prob)));
  }

  void PathTracer::trace_light_ray (SceneLight *light, struct LightPath& p, int bounces) {

    p.illum = vector<Spectrum>(bounces);
    p.pdf = vector<float>(bounces);
    p.isect = vector<Intersection>(bounces);
    p.ray = vector<Ray>(bounces);

    Spectrum L_out;
    float pdf;    
    Ray r = light->sample_ray(L_out, pdf);
    Spectrum light_bounce = Spectrum(1, 1, 1);
    float term_prob = 0.f;

    p.length = 0;

    for (int i = 0; i < bounces; i++) {
      p.ray[i] = r;

      if (!bvh->intersect(r, &p.isect[i])) {
	p.pdf[i] = -1;
	return;
      }

      Vector3D hit_p = r.o + r.d * p.isect[i].t;
      Vector3D hit_n = (p.isect[i].n).unit();

      // make a coordinate system for a hit point
      // with N aligned with the Z direction.
      Matrix3x3 o2w;
      make_coord_space(o2w, p.isect[i].n);

      Matrix3x3 w2o = o2w.T();

      // w_out points towards the source of the ray (e.g.,
      // toward the camera if this is a primary ray)
      Vector3D w_out = r.o - hit_p;
      w_out.normalize();

      L_out = L_out * light_bounce * fabs(dot(w_out, hit_n)) * (1.f / (pdf * (1.f-term_prob)));
      p.illum[i] = L_out;
      p.pdf[i] = pdf * (1.f-term_prob);

      term_prob = (1.f - clamp(light_bounce.illum(), 0, 1)) * 0.65;

      if (!p.isect[i].bsdf->is_delta()) {
	double dist_camera = (camera->position() - hit_p).norm();
	Vector3D dir_cam = (camera->position() - hit_p).unit();
	Vector3D origin = hit_p + EPS_D * dir_cam;
	Ray shadow = Ray(origin, dir_cam);
	Intersection shadow_isect;
	shadow.max_t = dist_camera;
	if (!(bvh->intersect(shadow, &shadow_isect))) {
	  if (dot(dir_cam, (camera->view_point() - camera->position())) < 0) {

	    Spectrum L_camera = L_out;
	    Vector3D w_in = w2o * dir_cam;
	    L_camera = L_camera * (p.isect[i].bsdf->f((w2o * w_out).unit(), w_in));
	    L_camera = L_camera * (1.f / (pdf * (1 - term_prob) * (p.length + 16)));

	    Vector2D uv = camera->intersect_ray(shadow);
	    size_t x = floor(uv.x * sampleBuffer.w);
	    size_t y = floor(uv.y * sampleBuffer.h);
	    sampleBuffer.add_pixel(L_camera, x, y);
	  }
	}
      }

      if (((float)(std::rand()) / RAND_MAX) < term_prob) return;

      p.length++;

      // Random direction to shoot ray in
      Vector3D wi;
      float pdf;
      w_out = (w2o * w_out).unit();
      light_bounce = p.isect[i].bsdf->sample_f(w_out, &wi, &pdf);

      wi = (o2w * wi).unit();
      r = Ray(hit_p + wi*EPS_D, wi);
    }
    
    p.length--;
  }

  Spectrum PathTracer::raytrace_pixel(size_t x, size_t y) {

    // Sample the pixel with coordinate (x,y) and return the result spectrum.
    // The sample rate is given by the number of camera rays per pixel.

    int num_samples = ns_aa;

    Vector2D p = Vector2D();

    Spectrum result = Spectrum(0,0,0);
    for (int i = 0; i < num_samples; i++) {
      Vector2D e = this->gridSampler->get_sample();
      p.x = ((double)x+e.x) / (double)sampleBuffer.w;
      p.y = ((double)y+e.y) / (double)sampleBuffer.h;
      
      Ray r = camera->generate_ray(p.x, p.y);      
      for (SceneLight* light : scene->lights) {
	struct LightPath path;
	trace_light_ray(light, path, max_ray_depth);
	
	result += trace_camera_ray(r, path, 0, true);
      }
    }
    result = result * (1 / (double)num_samples);
    return result;
  }
  
  void PathTracer::raytrace_tile(int tile_x, int tile_y,
				 int tile_w, int tile_h) {

    size_t w = sampleBuffer.w;
    size_t h = sampleBuffer.h;

    size_t tile_start_x = tile_x;
    size_t tile_start_y = tile_y;

    size_t tile_end_x = std::min(tile_start_x + tile_w, w);
    size_t tile_end_y = std::min(tile_start_y + tile_h, h);

    size_t tile_idx_x = tile_x / imageTileSize;
    size_t tile_idx_y = tile_y / imageTileSize;
    size_t num_samples_tile = tile_samples[tile_idx_x + tile_idx_y * num_tiles_w];

    for (size_t y = tile_start_y; y < tile_end_y; y++) {
      if (!continueRaytracing) return;
      for (size_t x = tile_start_x; x < tile_end_x; x++) {
        Spectrum s = raytrace_pixel(x, y);
        sampleBuffer.update_pixel(s, x, y, 0.8);
      }
    }

    tile_samples[tile_idx_x + tile_idx_y * num_tiles_w] += 1;
    sampleBuffer.toColor(frameBuffer, 0, 0, w, h);
  }

  void PathTracer::worker_thread() {

    Timer timer;
    timer.start();

    WorkItem work;
    while (continueRaytracing && workQueue.try_get_work(&work)) {
      raytrace_tile(work.tile_x, work.tile_y, work.tile_w, work.tile_h);
    }

    workerDoneCount++;
    if (!continueRaytracing && workerDoneCount == numWorkerThreads) {
      timer.stop();
      fprintf(stdout, "Canceled!\n");
      state = READY;
    }

    if (continueRaytracing && workerDoneCount == numWorkerThreads) {
      timer.stop();
      fprintf(stdout, "Done! (%.4fs)\n", timer.duration());
      state = DONE;
    }
  }

  void PathTracer::increase_area_light_sample_count() {
    ns_area_light *= 2;
    fprintf(stdout, "[PathTracer] Area light sample count increased to %zu!\n", ns_area_light);
  }

  void PathTracer::decrease_area_light_sample_count() {
    if (ns_area_light > 1) ns_area_light /= 2;
    fprintf(stdout, "[PathTracer] Area light sample count decreased to %zu!\n", ns_area_light);
  }

  void PathTracer::save_image() {

    if (state != DONE) return;

    time_t rawtime;
    time (&rawtime);

    string filename = "Screen Shot ";
    filename += string(ctime(&rawtime));
    filename.erase(filename.end() - 1);
    filename += string(".png");

    uint32_t* frame = &frameBuffer.data[0];
    size_t w = frameBuffer.w;
    size_t h = frameBuffer.h;
    uint32_t* frame_out = new uint32_t[w * h];
    for(size_t i = 0; i < h; ++i) {
      memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
    }

    fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
    lodepng::encode(filename, (unsigned char*) frame_out, w, h);
    fprintf(stderr, "Done!\n");
  }

}  // namespace CMU462
