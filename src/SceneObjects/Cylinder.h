#ifndef __CYLINDER_H__
#define __CYLINDER_H__

#include "../scene/scene.h"

class Cylinder : public SceneObject {
public:
  Cylinder(Scene *scene, Material *mat)
      : SceneObject(scene, mat), capped(true) {}

  virtual bool intersectLocal(ray &r, isect &i) const;
  virtual bool hasBoundingBoxCapability() const { return true; }

  virtual BoundingBox ComputeLocalBoundingBox() {
    BoundingBox localbounds;
    localbounds.setMin(glm::dvec3(-1.0f, -1.0f, 0.0f));
    localbounds.setMax(glm::dvec3(1.0f, 1.0f, 1.0f));
    return localbounds;
  }

  bool intersectBody(const ray &r, isect &i) const;
  bool intersectCaps(const ray &r, isect &i) const;

  void setCapped(bool capped) { this->capped = capped; }

protected:
  bool capped;

protected:
  void glDrawLocal(int quality, bool actualMaterials,
                   bool actualTextures) const;
};

#endif // __CYLINDER_H__
