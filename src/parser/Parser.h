#pragma warning(disable : 4786)

/** This file is deprecated and is kept to support parsing of legacy .ray files.
    See JsonParser.{cpp,h} for the new parsing code. **/

#ifndef __PARSER_H__

#define __PARSER_H__

#include <map>
#include <string>

#include "ParserException.h"
#include "Tokenizer.h"

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "../SceneObjects/Box.h"
#include "../SceneObjects/Cone.h"
#include "../SceneObjects/Cylinder.h"
#include "../SceneObjects/Sphere.h"
#include "../SceneObjects/Square.h"
#include "../SceneObjects/trimesh.h"
#include "../scene/light.h"
#include "../scene/scene.h"

typedef std::map<string, Material> mmap;

/* While parsing the file, we will end up traversing a DFS of a transformation
tree. We use the utility classes TransformNode and TransformRoot to track the
transformations. */
class TransformNode {
protected:
  // information about this node's transformation
  glm::dmat4 xform;

  // information about parent & children
  TransformNode *parent;
  std::vector<TransformNode *> children;

public:
  typedef std::vector<TransformNode *>::iterator child_iter;
  typedef std::vector<TransformNode *>::const_iterator child_citer;

  ~TransformNode() {
    for (auto c : children)
      delete c;
  }

  TransformNode *createChild(const glm::dmat4 &xform) {
    TransformNode *child = new TransformNode(this, xform);
    children.push_back(child);
    return child;
  }

  const glm::dmat4 &transform() const { return xform; }
  MatrixTransform toMatrixTransform() const { return MatrixTransform(xform); }

protected:
  // protected so that users can't directly construct one of these...
  // force them to use the createChild() method.  Note that they CAN
  // directly create a TransformRoot object.
  TransformNode(TransformNode *parent, const glm::dmat4 &xform) : children() {
    this->parent = parent;
    if (parent == NULL)
      this->xform = xform;
    else
      this->xform = parent->xform * xform;
  }
};

class TransformRoot : public TransformNode {
public:
  TransformRoot() : TransformNode(NULL, glm::dmat4(1.0)) {}
};

/*
  class Parser:
    The Parser is where most of the heavy lifting in parsing
    goes.  This particular parser reads in a stream of tokens
    from the Tokenizer and converts them into a scene in
    memory.

    If you really want to know, this parser is written
    as a top-down parser with one symbol of lookahead.
    See the docs on the website if you're interested in
    modifying this somehow.
*/

class Parser {
public:
  // We need the path for referencing files from the
  // base file.
  Parser(Tokenizer &tokenizer, string basePath)
      : _tokenizer(tokenizer), _basePath(basePath) {}

  // Parse the top-level scene
  Scene *parseScene();

private:
  // Highest level parsing routines
  void parseTransformableElement(Scene *scene, TransformNode *transform,
                                 const Material &mat);
  void parseGroup(Scene *scene, TransformNode *transform, const Material &mat);
  void parseCamera(Scene *scene);

  void parseGeometry(Scene *scene, TransformNode *transform,
                     const Material &mat);

  // Parse lights
  PointLight *parsePointLight(Scene *scene);
  DirectionalLight *parseDirectionalLight(Scene *scene);
  void parseAmbientLight(Scene *scene);

  // Parse geometry
  void parseSphere(Scene *scene, TransformNode *transform, const Material &mat);
  void parseBox(Scene *scene, TransformNode *transform, const Material &mat);
  void parseSquare(Scene *scene, TransformNode *transform, const Material &mat);
  void parseCylinder(Scene *scene, TransformNode *transform,
                     const Material &mat);
  void parseCone(Scene *scene, TransformNode *transform, const Material &mat);
  void parseTrimesh(Scene *scene, TransformNode *transform,
                    const Material &mat);
  void parseFaces(std::list<glm::dvec3> &faces);

  // Parse transforms
  void parseTranslate(Scene *scene, TransformNode *transform,
                      const Material &mat);
  void parseRotate(Scene *scene, TransformNode *transform, const Material &mat);
  void parseScale(Scene *scene, TransformNode *transform, const Material &mat);
  void parseTransform(Scene *scene, TransformNode *transform,
                      const Material &mat);

  // Helper functions for parsing expressions of the form:
  //   keyword = value;
  double parseScalarExpression();
  glm::dvec3 parseVec3dExpression();
  glm::dvec4 parseVec4dExpression();
  bool parseBooleanExpression();
  Material *parseMaterialExpression(Scene *scene, const Material &mat);
  string parseIdentExpression();

  MaterialParameter parseVec3dMaterialParameter(Scene *scene);
  MaterialParameter parseScalarMaterialParameter(Scene *scene);

  // Helper functions for parsing things like vectors
  // and idents.
  double parseScalar();
  std::list<double> parseScalarList();
  glm::dvec3 parseVec3d();
  glm::dvec4 parseVec4d();
  bool parseBoolean();
  Material *parseMaterial(Scene *scene, const Material &parent);
  string parseIdent();

  TransformRoot transformRoot;

private:
  Tokenizer &_tokenizer;
  mmap materials;
  std::string _basePath;
};

#endif
