#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>

using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3 &) const {
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}

glm::dvec3 DirectionalLight::shadowAttenuation(const ray &r,
                                               const glm::dvec3 &p) const {
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  // ask chatgpt about shadow attenuation: for non-transperent is zero for transparent is something 
  // attenuate by k_t^d 

  glm::dvec3 attenuation(1.0, 1.0, 1.0);
  ray shadowRay = r;
  isect i;

  // trace direcntional lights until infinite
  //while(shadowRay.getScene()->intersect(shadowRay, i)){
  while(getScene()->intersect(shadowRay, i)){
    const Material &mat = i.getMaterial();
    if (!mat.Trans()){
      return glm::dvec3(0.0, 0.0, 0.0);
    }

    double d = i.getT(); // d is distance travelled INSIDE object
    glm::dvec3 kt = mat.kt(i); 
    attenuation *= glm::pow(kt, glm::dvec3(d)); //attenuate by kt^d

    shadowRay = ray(shadowRay.at(i.getT()) + RAY_EPSILON*shadowRay.getDirection(), shadowRay.getDirection(), shadowRay.getAtten(), ray::SHADOW); // continue ray past intersection
  }
  return attenuation; 

  //return glm::dvec3(1.0, 1.0, 1.0);
}

glm::dvec3 DirectionalLight::getColor() const { return color; }

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3 &) const {
  return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3 &P) const {

  // YOUR CODE HERE

  // Calculate distance form light to point
  // only works for point light
  // there are two classes of light, directonal light and point light, d is only for point light
  double d = glm::length(position-P);
  double a0 = constantTerm;
  double a1 = linearTerm;
  double a2 = quadraticTerm;
  double denom = a0 + a1*d + a2*d*d;

  return std::min(1.0, 1.0/denom);


  // You'll need to modify this method to attenuate the intensity
  // of the light based on the distance between the source and the
  // point P.  For now, we assume no attenuation and just return 1.0

}

glm::dvec3 PointLight::getColor() const { return color; }

glm::dvec3 PointLight::getDirection(const glm::dvec3 &P) const {
  return glm::normalize(position - P);
}

glm::dvec3 PointLight::shadowAttenuation(const ray &r,
                                         const glm::dvec3 &p) const {                                     
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  glm::dvec3 attenuation(1.0, 1.0, 1.0);

  
  glm::dvec3 toLight = position-p;// DIrection  from point to light
  double maxDist = glm::length(toLight);
  glm::dvec3 dir = glm::normalize(toLight);

  ray shadowRay(p+RAY_EPSILON*dir, dir, glm::dvec3(1.0), ray::SHADOW);
  isect i;

  while (scene->intersect(shadowRay,i)){
    double t = i.getT();
    if (t>=maxDist){
      break; // ignore if intersection is beyonf light
    }
    const Material &m = i.getMaterial();
    glm::dvec3 kt = m.kt(i);

    if(glm::length(kt) == 0.0){
      return glm::dvec3(0.0); // opaque objects completely blocks light
    }

    double d = t; // d is distance travled inside the object
    attenuation *= glm::pow(kt, glm::dvec3(d));

    shadowRay = ray(shadowRay.at(t) + RAY_EPSILON*dir, dir, shadowRay.getAtten(), ray::SHADOW); // go past interseciton
  }
  return attenuation;

  return glm::dvec3(1, 1, 1);
}

#define VERBOSE 0

