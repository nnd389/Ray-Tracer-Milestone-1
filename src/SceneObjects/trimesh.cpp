#include "trimesh.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <float.h>
#include <string.h>
#include "../ui/TraceUI.h"
extern TraceUI *traceUI;
extern TraceUI *traceUI;

using namespace std;

Trimesh::~Trimesh() {
  for (auto f : faces)
    delete f;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex(const glm::dvec3 &v) { vertices.emplace_back(v); }

void Trimesh::addNormal(const glm::dvec3 &n) { normals.emplace_back(n); }

void Trimesh::addColor(const glm::dvec3 &c) { vertColors.emplace_back(c); }

void Trimesh::addUV(const glm::dvec2 &uv) { uvCoords.emplace_back(uv); }

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace(int a, int b, int c) {
  int vcnt = vertices.size();

  if (a >= vcnt || b >= vcnt || c >= vcnt)
    return false;

  TrimeshFace *newFace = new TrimeshFace(this, a, b, c);
  if (!newFace->degen)
    faces.push_back(newFace);
  else
    delete newFace;

  // Don't add faces to the scene's object list so we can cull by bounding
  // box
  return true;
}

// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
const char *Trimesh::doubleCheck() {
  if (!vertColors.empty() && vertColors.size() != vertices.size())
    return "Bad Trimesh: Wrong number of vertex colors.";
  if (!uvCoords.empty() && uvCoords.size() != vertices.size())
    return "Bad Trimesh: Wrong number of UV coordinates.";
  if (!normals.empty() && normals.size() != vertices.size())
    return "Bad Trimesh: Wrong number of normals.";

  return 0;
}

bool Trimesh::intersectLocal(ray &r, isect &i) const {
  bool have_one = false;
  for (auto face : faces) {
    isect cur;
    if (face->intersectLocal(r, cur)) {
      if (!have_one || (cur.getT() < i.getT())) {
        i = cur;
        have_one = true;
      }
    }
  }
  if (!have_one)
    i.setT(1000.0);
  return have_one;
}

bool TrimeshFace::intersect(ray &r, isect &i) const {
  return intersectLocal(r, i);
}


// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray &r, isect &i) const {
  // YOUR CODE HERE
  //
  // FIXME: Add ray-trimesh intersection

  /* To determine the color of an intersection, use the following rules:
     - If the parent mesh has non-empty `uvCoords`, barycentrically interpolate
       the UV coordinates of the three vertices of the face, then assign it to
       the intersection using i.setUVCoordinates().
     - Otherwise, if the parent mesh has non-empty `vertexColors`,
       barycentrically interpolate the colors from the three vertices of the
       face. Create a new material by copying the parent's material, set the
       diffuse color of this material to the interpolated color, and then 
       assign this material to the intersection.
     - If neither is true, assign the parent's material to the intersection.
  */
  // see Real-Time_Collision_Detection_----_(Chapter_5_Basic_Primitive_Tests).pdf
  // section 5.3.6 Intersecting Ray or Segment Against Triangle
  glm::dvec3 A = parent->vertices[ids[0]];
  glm::dvec3 B = parent->vertices[ids[1]];
  glm::dvec3 C = parent->vertices[ids[2]];

  glm::dvec3 AB = B-A;
  glm::dvec3 AC = C-A;

  // calculate denomenator
  double d = glm::dot(-r.getDirection(), glm::cross(AB,AC)); // d = (P-Q) dot ((B-A)x(C-A)),    (P-Q)=-Direction
  if (fabs(d)<1e-8) return false; // if d=0, the segment runs parallel to the triangle

  // calculate barycentric coordinates u, v, w 
  double u = glm::dot(-r.getDirection(), glm::cross(r.getPosition()-A, AC)) / d; // u = [(P-Q) dot ((P-A)x(C-A)) / d],     d is denominator defined above
  double v = glm::dot(-r.getDirection(), glm::cross(AB, r.getPosition()-A)) / d; // v = [(P-Q) dot ((B-A)x(P-A)) / d],     d is denominator defined above
  double w = 1-u-v;
  if (u<0.0 || v<0.0 || w<0.0) return false; // if u, v, or w are negative we are outside the triangle

  // solve for t to get intersecting point
  double t = glm::dot(r.getPosition()-A, glm::cross(AB, AC)) / d; // t = [(P-A) dot ((B-A)x(C-A)) / d],     d is denominator defined above

  if(t<RAY_EPSILON) return false;

  //fill intersection
  i.setT(t);
  i.setObject(parent);
  i.setN(normal);

  if(!parent->uvCoords.empty()){
    glm::dvec2 uA = parent->uvCoords[ids[0]];
    glm::dvec2 uB = parent->uvCoords[ids[1]];
    glm::dvec2 uC = parent->uvCoords[ids[2]];
    glm::dvec2 uv = (1-u-v)*uA + u*uB + v*uC;
    i.setUVCoordinates(uv);
  } else if(!parent->vertColors.empty()){
    glm::dvec3 c0 = parent->vertColors[ids[0]];
    glm::dvec3 c1 = parent->vertColors[ids[1]];
    glm::dvec3 c2 = parent->vertColors[ids[2]];
    glm::dvec3 color = (1-u-v)*c0 + u*c1 + v*c2;
    Material mat(parent->getMaterial());
    mat.setDiffuse(color);
    i.setMaterial(mat);
  } else{
    i.setMaterial(parent->getMaterial());
  }
  return true;
}



// Once all the verts and faces are loaded, per vertex normals can be
// generated by averaging the normals of the neighboring faces.
void Trimesh::generateNormals() {
  int cnt = vertices.size();
  normals.resize(cnt);
  std::vector<int> numFaces(cnt, 0);

  for (auto face : faces) {
    glm::dvec3 faceNormal = face->getNormal();

    for (int i = 0; i < 3; ++i) {
      normals[(*face)[i]] += faceNormal;
      ++numFaces[(*face)[i]];
    }
  }

  for (int i = 0; i < cnt; ++i) {
    if (numFaces[i])
      normals[i] /= numFaces[i];
  }

  vertNorms = true;
}

