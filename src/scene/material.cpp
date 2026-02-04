#include "material.h"
#include "../ui/TraceUI.h"
#include "light.h"
#include "ray.h"
extern TraceUI *traceUI;

#include "../fileio/images.h"
#include <glm/gtx/io.hpp>
#include <iostream>

using namespace std;
extern bool debugMode;

Material::~Material() {}

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
glm::dvec3 Material::shade(Scene *scene, const ray &r, const isect &i) const {
  // YOUR CODE HERE

  // For now, this method just returns the diffuse color of the object.
  // This gives a single matte color for every distinct surface in the
  // scene, and that's it.  Simple, but enough to get you started.
  // (It's also inconsistent with the phong model...)

  // Your mission is to fill in this method with the rest of the phong
  // shading model, including the contributions of all the light sources.
  // You will need to call both distanceAttenuation() and
  // shadowAttenuation()
  // somewhere in your code in order to compute shadows and light falloff.
  //	if( debugMode )
  //		std::cout << "Debugging Phong code..." << std::endl;

  // When you're iterating through the lights,
  // you'll want to use code that looks something
  // like this:
  //
  // for ( const auto& pLight : scene->getAllLights() )
  // {
  //              // pLight has type Light*
  // 		.
  // 		.
  // 		.
  // }
  // /Users/kneenaugh/Desktop/Ray-Tracer-Milestone-1/assets/scenes/cone.json
  // ./ray -r 5 /Users/kneenaugh/Desktop/Ray-Tracer-Milestone-1/assets/scenes/cone.json output.png
  glm::dvec3 P = r.at(i.getT());
  glm::dvec3 N = glm::normalize(i.getN()); // N is the normal
  glm::dvec3 V = glm::normalize(-r.getDirection()); // V is the in direciton, might need to check the sign

  glm::dvec3 color(0.0, 0.0, 0.0);

  // emmissive term
  glm::dvec3 emmisiveTerm = ke(i);

  // ambient term
  glm::dvec3 ambientTerm = ka(i) * scene->ambient();

  color += ambientTerm + emmisiveTerm;

  for (const auto& pLight : scene->getAllLights()){
    glm::dvec3 L = glm::normalize(pLight->getDirection(P)); // shading light from intersection to light
    glm::dvec3 R = glm::reflect(-L, N); // CHECK: what is R? the reflection of L_j about N? R is the reflected direcltion if the light 
    glm::dvec3 I = pLight->getColor(); //yes!

    // Diffuse Term
    double NdotL = std::max(glm::dot(N,L), 0.0);

    // Specular Term
    double VdotR = std::max(glm::dot(V,R), 0.0);
    double spec = pow(VdotR, shininess(i));

    // Distance attenuation: min(1, 1/(a0 + a1*dj + a2*dj^2))
    double lightAttenuation = pLight->distanceAttenuation(P); // FIX: fix code in distanceAttenuation in light.cpp
    //FIX: also need to call shadowAttenuation, but where?? 
    glm::dvec3 shadowAtten = pLight->shadowAttenuation(r,P); // where do I use this though?

    // multiply shadow attentuation and distance attentuation


    color += I * lightAttenuation * shadowAtten * (kd(i)*NdotL + ks(i)*spec);

  }

  //return kd(i);
  return color;
}

TextureMap::TextureMap(string filename) {
  data = readImage(filename.c_str(), width, height);
  if (data.empty()) {
    width = 0;
    height = 0;
    string error("Unable to load texture map '");
    error.append(filename);
    error.append("'.");
    throw TextureMapException(error);
  }
}

glm::dvec3 TextureMap::getMappedValue(const glm::dvec2 &coord) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.
  // What this function should do is convert from
  // parametric space which is the unit square
  // [0, 1] x [0, 1] in 2-space to bitmap coordinates,
  // and use these to perform bilinear interpolation
  // of the values.

  return glm::dvec3(1, 1, 1);
}

glm::dvec3 TextureMap::getPixelAt(int x, int y) const {
  // YOUR CODE HERE
  //
  // In order to add texture mapping support to the
  // raytracer, you need to implement this function.

  return glm::dvec3(1, 1, 1);
}

glm::dvec3 MaterialParameter::value(const isect &is) const {
  if (0 != _textureMap)
    return _textureMap->getMappedValue(is.getUVCoordinates());
  else
    return _value;
}

double MaterialParameter::intensityValue(const isect &is) const {
  if (0 != _textureMap) {
    glm::dvec3 value(_textureMap->getMappedValue(is.getUVCoordinates()));
    return (0.299 * value[0]) + (0.587 * value[1]) + (0.114 * value[2]);
  } else
    return (0.299 * _value[0]) + (0.587 * _value[1]) + (0.114 * _value[2]);
}
