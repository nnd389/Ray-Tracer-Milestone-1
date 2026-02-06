// The main ray tracer.

#pragma warning(disable : 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/JsonParser.h"
#include "parser/Parser.h"
#include "parser/Tokenizer.h"
#include <json.hpp>

#include "ui/TraceUI.h"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <string.h> // for memset

#include <fstream>
#include <iostream>

using namespace std;
extern TraceUI *traceUI;

// Use this variable to decide if you want to print out debugging messages. Gets
// set in the "trace single ray" mode in TraceGLWindow, for example.
bool debugMode = false;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates
// (x,y), through the projection plane, and out into the scene. All we do is
// enter the main ray-tracing method, getting things started by plugging in an
// initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

glm::dvec3 RayTracer::trace(double x, double y) {
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) {
    scene->clearIntersectCache();
  }

  ray r(glm::dvec3(0, 0, 0), glm::dvec3(0, 0, 0), glm::dvec3(1, 1, 1),
        ray::VISIBILITY);
  scene->getCamera().rayThrough(x, y, r);
  double dummy;
  glm::dvec3 ret =
      traceRay(r, glm::dvec3(1.0, 1.0, 1.0), traceUI->getDepth(), dummy);
  ret = glm::clamp(ret, 0.0, 1.0);
  return ret;
}

glm::dvec3 RayTracer::tracePixel(int i, int j) {
  glm::dvec3 col(0, 0, 0);

  if (!sceneLoaded())
    return col;

  double x = double(i) / double(buffer_width);
  double y = double(j) / double(buffer_height);

  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  col = trace(x, y);

  pixel[0] = (int)(255.0 * col[0]);
  pixel[1] = (int)(255.0 * col[1]);
  pixel[2] = (int)(255.0 * col[2]);
  return col;
}

#define VERBOSE 0

// Do recursive ray tracing! You'll want to insert a lot of code here (or places
// called from here) to handle reflection, refraction, etc etc.
glm::dvec3 RayTracer::traceRay(ray &r, const glm::dvec3 &thresh, int depth, double &t) {
  isect i;
  glm::dvec3 colorC;
  //std::cerr << "traceRay called\n";

#if VERBOSE
  std::cerr << "== current depth: " << depth << std::endl;
#endif

// Tyler's code for reflection/refraction
#if 1
  if (scene->intersect(r, i)) {
    // YOUR CODE HERE

    // An intersection occurred!  We've got work to do. For now, this code gets
    // the material for the surface that was intersected, and asks that material
    // to provide a color for the ray.

    // This is a great place to insert code for recursive ray tracing. Instead
    // of just returning the result of shade(), add some more steps: add in the
    // contributions from reflected and refracted rays.

    const Material &m = i.getMaterial();
    colorC = m.shade(scene.get(), r, i);
    t = i.getT();


    //////////////////////
    ///// REFLECTION /////
    //////////////////////
    if (depth > 0 && m.Refl()) { // depths checks if we still have recursion depth left, and the Refl() function checks if the material is reflective
    //if (depth > 0) {
      //std::cerr << "REFLECTION BLOCK depth=" << depth << "\n";
      glm::dvec3 kr = m.kr(i); // This is the reflection coefficient (RGB)
      //glm::dvec3 kr(1.0, 1.0, 1.0); // For testing reflection without material, set kr to (1,1,1) for perfect reflection
      // kr = (0,0,0) means no reflection, kr = (1,1,1) means perfect reflection
    
      // Checks if the reflection is worth doing -- if the reflection coefficient is very small, it won't contribute much to the final color, so we can skip it
      //if (kr.x > thresh.x || kr.y > thresh.y || kr.z > thresh.z) {
      if (glm::compMax(kr) > 1e-6) {
        glm::dvec3 P = r.at(i.getT()); // This is the point of intersection
        glm::dvec3 N = glm::normalize(i.getN()); // unit surface normal at intersection
        glm::dvec3 I = glm::normalize(r.getDirection()); // direction the ray travels

        if (glm::dot(I, N) > 0.0) N = -N; // flip normal if ray is inside the surface

        //glm::dvec3 Rdir = glm::normalize(glm::reflect(I, N)); // Where we could use glm::reflect, but required to do it manually for the assignment
        // Manual mirror reflection using r = 2(l·n)n − l  (from formula sheet)
        //glm::dvec3 l = -I; // convert ray-travel direction to the "incoming-to-surface" direction used in the notes
        //glm::dvec3 Rdir = glm::normalize(2.0 * glm::dot(l, N) * N - l);
        glm::dvec3 Rdir = glm::normalize(I - 2.0 * glm::dot(I, N) * N); 


        // Creates the reflected ray with a small offset to avoid self-intersection
        const double eps = RAY_EPSILON; // Definied in ray.h
        //const double eps = 1e-3; // A larger epsilon for testing
        //ray Rray(P + eps * N, Rdir, glm::dvec3(1.0, 1.0, 1.0), ray::VISIBILITY);
        //ray Rray(P + eps * Rdir, Rdir, glm::dvec3(1.0, 1.0, 1.0), ray::REFLECTION);
        ray Rray(P + eps * N, Rdir, glm::dvec3(1.0), ray::REFLECTION);


        // Recursively traces the reflected ray
        double tR;
        glm::dvec3 Rcol = traceRay(Rray, thresh, depth - 1, tR);
        //std::cerr << "tR=" << tR << " Rcol=" << Rcol.x << "," << Rcol.y << "," << Rcol.z << "\n";
        colorC += kr * Rcol; // Adds the reflection contribution to the final color, scaled by the reflection coefficient
      }
    }
    
    //////////////////////
    ///// REFRACTION /////
    //////////////////////
    if (depth > 0 && m.Trans()) { // depth checks if we have recursion left and Trans checks if the material has a nonzero transmissive term
      glm::dvec3 kt = m.kt(i);

      // Skip if essentially zero
      if (glm::compMax(kt) > 1e-6) {
        glm::dvec3 P = r.at(i.getT());
        glm::dvec3 N = glm::normalize(i.getN());
        glm::dvec3 I = glm::normalize(r.getDirection());

        // Indices of refraction
        double n1 = 1.0; // IOR of medium the ray is currently in
        double n2 = m.index(i); // IOR of the objects material

        // Determine if we're entering or exiting
        // If dot(I, N) > 0, ray is inside the object heading out -> flip N and swap IORs
        if (glm::dot(I, N) > 0.0) {
          N = -N;
          std::swap(n1, n2);
        }

        double eta = n1 / n2;

        // cosi = cos(theta_i) with theta_i measured from the normal
        double cosi = -glm::dot(N, I); // should be >= 0 after orientation
        
        // This avoids arcsin out of bounds err
        // k = 1 - eta^2 (1 - cosi^2)
        double sin2t = eta * eta * (1.0 - cosi * cosi);
        double k = 1.0 - sin2t;

        if (k < 0.0) {
          // Total Internal Reflection: fall back to reflection
          glm::dvec3 Rdir = glm::normalize(I - 2.0 * glm::dot(I, N) * N);
          const double eps = RAY_EPSILON;
          ray Rray(P + eps * N, Rdir, glm::dvec3(1.0), ray::REFLECTION);

          double tR;
          glm::dvec3 Rcol = traceRay(Rray, thresh, depth - 1, tR);

          colorC += kt * Rcol;
        } else {
          double cost = std::sqrt(k);

          // Transmitted direction (Snell)
          glm::dvec3 Tdir = glm::normalize(eta * I + (eta * cosi - cost) * N);

          const double eps = RAY_EPSILON;

          // Offset the origin slightly *into the transmitted side*.
          ray Tray(P - eps * N, Tdir, glm::dvec3(1.0), ray::REFRACTION);

          double tT;
          glm::dvec3 Tcol = traceRay(Tray, thresh, depth - 1, tT);

          colorC += kt * Tcol;
        }
      }
    }
  } else {
    // No intersection. This ray travels to infinity, so we color
    // it according to the background color, which in this (simple)
    // case is just black.
    //
    // FIXME: Add CubeMap support here.
    // TIPS: CubeMap object can be fetched from
    // traceUI->getCubeMap();
    //       Check traceUI->cubeMap() to see if cubeMap is loaded
    //       and enabled.
    t = std::numeric_limits<double>::infinity();
    //colorC = glm::dvec3(0.0, 0.0, 0.0);
    colorC = glm::dvec3(1.0, 0.0, 1.0);
  }
#endif
// End of Tyler's code for reflection/refraction

#if VERBOSE
  std::cerr << "== depth: " << depth + 1 << " done, returning: " << colorC
            << std::endl;
#endif
  return colorC;
}

RayTracer::RayTracer()
    : scene(nullptr), buffer(0), thresh(0), buffer_width(0), buffer_height(0),
      m_bBufferReady(false) {
}

RayTracer::~RayTracer() {}

void RayTracer::getBuffer(unsigned char *&buf, int &w, int &h) {
  buf = buffer.data();
  w = buffer_width;
  h = buffer_height;
}

double RayTracer::aspectRatio() {
  return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene(const char *fn) {
  ifstream ifs(fn);
  if (!ifs) {
    string msg("Error: couldn't read scene file ");
    msg.append(fn);
    traceUI->alert(msg);
    return false;
  }

  // Check if fn ends in '.ray'
  bool isRay = false;
  const char *ext = strrchr(fn, '.');
  if (ext && !strcmp(ext, ".ray"))
    isRay = true;

  // Strip off filename, leaving only the path:
  string path(fn);
  if (path.find_last_of("\\/") == string::npos)
    path = ".";
  else
    path = path.substr(0, path.find_last_of("\\/"));

  if (isRay) {
    // .ray Parsing Path
    // Call this with 'true' for debug output from the tokenizer
    Tokenizer tokenizer(ifs, false);
    Parser parser(tokenizer, path);
    try {
      scene.reset(parser.parseScene());
    } catch (SyntaxErrorException &pe) {
      traceUI->alert(pe.formattedMessage());
      return false;
    } catch (ParserException &pe) {
      string msg("Parser: fatal exception ");
      msg.append(pe.message());
      traceUI->alert(msg);
      return false;
    } catch (TextureMapException e) {
      string msg("Texture mapping exception: ");
      msg.append(e.message());
      traceUI->alert(msg);
      return false;
    }
  } else {
    // JSON Parsing Path
    try {
      JsonParser parser(path, ifs);
      scene.reset(parser.parseScene());
    } catch (ParserException &pe) {
      string msg("Parser: fatal exception ");
      msg.append(pe.message());
      traceUI->alert(msg);
      return false;
    } catch (const json::exception &je) {
      string msg("Invalid JSON encountered ");
      msg.append(je.what());
      traceUI->alert(msg);
      return false;
    }
  }

  if (!sceneLoaded())
    return false;

  return true;
}

void RayTracer::traceSetup(int w, int h) {
  size_t newBufferSize = w * h * 3;
  if (newBufferSize != buffer.size()) {
    bufferSize = newBufferSize;
    buffer.resize(bufferSize);
  }
  buffer_width = w;
  buffer_height = h;
  std::fill(buffer.begin(), buffer.end(), 0);
  m_bBufferReady = true;

  /*
   * Sync with TraceUI
   */

  threads = traceUI->getThreads();
  block_size = traceUI->getBlockSize();
  thresh = traceUI->getThreshold();
  samples = traceUI->getSuperSamples();
  aaThresh = traceUI->getAaThreshold();

  // YOUR CODE HERE
  // FIXME: Additional initializations
}

/*
 * RayTracer::traceImage
 *
 *	Trace the image and store the pixel data in RayTracer::buffer.
 *
 *	Arguments:
 *		w:	width of the image buffer
 *		h:	height of the image buffer
 *
 */
void RayTracer::traceImage(int w, int h) {
  // Always call traceSetup before rendering anything.
  traceSetup(w, h);

  for (int y=0;y<h;y++){
    for (int x=0;x<w;x++){
      double px = x+0.5;
      double py = y+0.5;
      tracePixel(x,y);
    }
  }
  setReady(true);

  // YOUR CODE HERE

  // FIX: this will automatically generate - loop over all the pixels -- 
  // FIXME: Start one or more threads for ray tracing
  //
  // TIPS: Ideally, the traceImage should be executed asynchronously,
  //       i.e. returns IMMEDIATELY after working threads are launched.
  //
  //       An asynchronous traceImage lets the GUI update your results
  //       while rendering.
}

int RayTracer::aaImage() {
  // YOUR CODE HERE
  // FIXME: Implement Anti-aliasing here
  //
  // TIP: samples and aaThresh have been synchronized with TraceUI by
  //      RayTracer::traceSetup() function
  return 0;
}

bool RayTracer::checkRender() {
  // YOUR CODE HERE
  // FIXME: Return true if tracing is done.
  //        This is a helper routine for GUI.
  //
  // TIPS: Introduce an array to track the status of each worker thread.
  //       This array is maintained by the worker threads.
  return true;
}

void RayTracer::waitRender() {
  // YOUR CODE HERE
  // FIXME: Wait until the rendering process is done.
  //        This function is essential if you are using an asynchronous
  //        traceImage implementation.
  //
  // TIPS: Join all worker threads here.
}

glm::dvec3 RayTracer::getPixel(int i, int j) {
  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;
  return glm::dvec3((double)pixel[0] / 255.0, (double)pixel[1] / 255.0,
                    (double)pixel[2] / 255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color) {
  unsigned char *pixel = buffer.data() + (i + j * buffer_width) * 3;

  pixel[0] = (int)(255.0 * color[0]);
  pixel[1] = (int)(255.0 * color[1]);
  pixel[2] = (int)(255.0 * color[2]);
}
