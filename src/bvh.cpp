#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <assert.h>
#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 { namespace StaticScene {

    BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
		       size_t max_leaf_size) {

      this->primitives = _primitives;

      // TODO:
      // Construct a BVH from the given vector of primitives and maximum leaf
      // size configuration. The starter code build a BVH aggregate with a
      // single leaf node (which is also the root) that encloses all the
      // primitives.

      BBox bb = BBox();

      for (size_t i = 0; i < primitives.size(); ++i) {
	bb.expand(primitives[i]->get_bbox());
      }

      Vector3D centroids[primitives.size()];
      BBox bboxes[primitives.size()];

      this->root = new BVHNode(bb, 0, primitives.size());
      makeTree(root, 0, primitives.size(), max_leaf_size, centroids, bboxes);
    }
    
    void BVHAccel::makeTree(BVHNode *top, size_t start, size_t range, size_t max_leaf_size,
			    Vector3D *centroids, BBox *bboxes) {

      if (range <= max_leaf_size) return;

      for (size_t i = 0; i < primitives.size(); ++i) {
	bboxes[i] = primitives[i]->get_bbox();
        centroids[i] = bboxes[i].centroid();
      }

      // Lowest partition found
      double minCost = INF_D;
      double splitIdx = 0;
      bool xSplit = false;
      bool ySplit = false;
      bool zSplit = false;
      BBox minLeft=BBox(), minRight=BBox();

      // Check partition planes in x-axis
      for (size_t i = start; i < start+range; i++) {
	double cost = 0;
	double plane = centroids[i].x;
	BBox left=BBox(), right=BBox();
	size_t size_left=0, size_right=0;

	for (size_t j = start; j < start+range; j++) {
	  if (plane > centroids[j].x) {
	    left.expand(bboxes[j]);
	    size_left++;
	  } else {
	    right.expand(bboxes[j]);
	    size_right++;
	  }
	}
	
	if (size_right && size_left) {
	  cost = size_left*left.surface_area();
	  cost += size_right*right.surface_area();
	  cost = cost;

	  if (cost < minCost) {	   
	    xSplit = true;
	    minCost = cost;
	    splitIdx = plane;
	    minLeft = left;
	    minRight = right;
	  }
	}
      }

      // Check partition planes in y-axis
      for (size_t i = start; i < start+range; i++) {
	double cost = 0;
	double plane = centroids[i].y;
	BBox left=BBox(), right=BBox();
	size_t size_left=0, size_right=0;

	for (size_t j = start; j < start+range; j++) {
	  if (plane > centroids[j].y) {
	    left.expand(bboxes[j]);
	    size_left++;
	  } else {
	    right.expand(bboxes[j]);
	    size_right++;
	  }
	}
	
	if (size_right && size_left) {
	  cost = size_left*left.surface_area();
	  cost += size_right*right.surface_area();
	  cost = cost;
	
	  if (cost < minCost) {
	    xSplit = false;
	    ySplit = true;
	    minCost = cost;
	    splitIdx = plane;
	    minLeft = left;
	    minRight = right;
	  }
	}
      }

      // Check partition planes in z-axis
      for (size_t i = start; i < start+range; i++) {
	double cost = 0;
	double plane = centroids[i].z;
	BBox left=BBox(), right=BBox();
	size_t size_left=0, size_right=0;

	for (size_t j = start; j < start+range; j++) {
	  if (plane > centroids[j].z) {
	    left.expand(bboxes[j]);
	    size_left++;
	  } else {
	    right.expand(bboxes[j]);
	    size_right++;
	  }
	}
	
	if (size_right && size_left) {
	  cost = size_left*left.surface_area();
	  cost += size_right*right.surface_area();
	  cost = cost;
	  
	  if (cost < minCost) {
	    xSplit = false;
	    ySplit = false;
	    zSplit = true;
	    minCost = cost;
	    splitIdx = plane;
	    minLeft = left;
	    minRight = right;
	  }
	}
      }

      Primitive** p = std::partition(&primitives[start], &primitives[start+range], 
				     [xSplit, ySplit, zSplit, splitIdx](Primitive *p) -> bool { 
				       if (xSplit) return splitIdx > p->get_bbox().centroid().x;
				       if (ySplit) return splitIdx > p->get_bbox().centroid().y;
				       return splitIdx > p->get_bbox().centroid().z;
				     });
      
      size_t mid = ((std::vector<Primitive*>::iterator)p) - primitives.begin();
      
      BVHNode *left = new BVHNode(minLeft, start, mid - start);
      BVHNode *right = new BVHNode(minRight, mid, range - mid + start);
      
      top->l = left;
      top->r = right;      
      
      makeTree(left, start, mid - start, max_leaf_size, centroids, bboxes);
      makeTree(right, mid, range - mid + start, max_leaf_size, centroids, bboxes);
    }

    BVHAccel::~BVHAccel() {

      // TODO:
      // Implement a proper destructor for your BVH accelerator aggregate

    }

    BBox BVHAccel::get_bbox() const {
      return root->bb;
    }

    bool BVHAccel::intersect(const Ray &ray) const {

      // TODO:
      // Implement ray - bvh aggregate intersection test. A ray intersects
      // with a BVH aggregate if and only if it intersects a primitive in
      // the BVH that is not an aggregate.

      Intersection i;
      double t0=ray.min_t, t1=ray.max_t;
      if (root->bb.intersect(ray, t0, t1)) {
	return findClosestHit(ray, root, &i);	
      } else {
	return false;
      }
    }

    bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

      // TODO:
      // Implement ray - bvh aggregate intersection test. A ray intersects
      // with a BVH aggregate if and only if it intersects a primitive in
      // the BVH that is not an aggregate. When an intersection does happen.
      // You should store the non-aggregate primitive in the intersection data
      // and not the BVH aggregate itself.
      
      double t0=ray.min_t, t1=ray.max_t;
      if (root->bb.intersect(ray, t0, t1)) {
	return findClosestHit(ray, root, i);	
      } else {
	return false;
      }
    }
    
    bool BVHAccel::findClosestHit(const Ray& r, BVHNode *node, Intersection *i) const {
      if (node->isLeaf()) {

	bool hit = false;

	for (size_t p = node->start; p < node->start + node->range; p++) {
	  Intersection temp;
	  if (primitives[p]->intersect(r, &temp)) {
	    if (temp.t < i->t) {
	      hit = true;
	      i->t = temp.t;
	      i->n = temp.n;
	      i->primitive = temp.primitive;
	      i->bsdf = temp.bsdf;
	    }
	  }
	}

	return hit;

      } else {
	double t0_l=r.min_t, t1_l=r.max_t;
	double t0_r=r.min_t, t1_r=r.max_t;

	bool hit_left = node->l->bb.intersect(r, t0_l, t1_l);
	bool hit_right = node->r->bb.intersect(r, t0_r, t1_r);
	
	if (!hit_left && !hit_right) return false;

	if (!hit_left) return findClosestHit(r, node->r, i);

	if (!hit_right) return findClosestHit(r, node->l, i);
	
	BVHNode *first = (t0_l <= t0_r) ? node->l : node->r;
	BVHNode *second = (t0_l <= t0_r) ? node->r : node->l;

	bool hit_first = findClosestHit(r, first, i);
	bool hit_second = false;
	
	if (!hit_first || std::max(t0_l, t0_r) < i->t)
	  hit_second = findClosestHit(r, second, i);

	return hit_first || hit_second;
      }

    }

  }  // namespace StaticScene
}  // namespace CMU462
