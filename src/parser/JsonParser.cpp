#include "JsonParser.h"
#include "ParserException.h"

#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_DOUBLE
#include "tiny_obj_loader.h"
#include <sstream>

#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <unordered_map>

#include <json.hpp>
using json = nlohmann::json;

// 1.5GB of memory at ~300B per Material
constexpr size_t MAX_RECOMMENDED_VERTS = 5'000'000;

/*
A helper macro which lets us ignore any JSON exceptions that occur.
The code that is wrapped in IGNORE_MISSING will silently not run if the
it accesses a missing key. This is useful when trying to overwrite
default values without having to check if the key exists first.
*/

#define IGNORE_MISSING(code)                                                   \
  do {                                                                         \
    try {                                                                      \
      code;                                                                    \
    } catch (const json::exception &) {                                        \
    }                                                                          \
  } while (0)

// A helper macro for loading materials. If the JSON object has a "material"
// key, parse and return it. Otherwise, return the current material.
#define GET_MAT_W_CUR(j, pd)                                                   \
  hasKey(j, "material") ? parseMaterial(j.at("material"), pd) : pd.cur_mat;

// Special deserializers for vec3. Must be within the nlohmann namespace.
namespace glm {
static void from_json(const json &j, glm::dvec3 &vec) {
  vec.x = j.at(0).get<double>();
  vec.y = j.at(1).get<double>();
  vec.z = j.at(2).get<double>();
}
} // namespace nlohmann

bool hasKey(const json &obj, const std::string &key) {
  return obj.find(key) != obj.end();
}

glm::dmat4 ParseData::getCurrentTransform() {
  glm::dmat4 transform = glm::dmat4(1.0);
  for (auto &t : transformStack) {
    transform = transform * t;
  }
  return transform;
}

Camera parseCamera(const json &j) {
  Camera c;
  IGNORE_MISSING(c.setEye(j.at("position").get<glm::dvec3>()));
  // If a camera has an updir, it must have a viewdir. We intentionally
  // let the json library throw an exception if this is not the case.
  if (hasKey(j, "updir") || hasKey(j, "viewdir")) {
    auto updir = j.at("updir").get<glm::dvec3>();
    auto viewdir = j.at("viewdir").get<glm::dvec3>();
    c.setLook(viewdir, updir);
  }
  IGNORE_MISSING(c.setFOV(j.at("fov").get<double>()));
  IGNORE_MISSING(c.setAspectRatio(j.at("aspectRatio").get<double>()));
  return c;
}

MaterialParameter parseMaterialParameter(const json &j, ParseData &pd) {
  MaterialParameter p;
  if (hasKey(j, "constant")) {
    p = MaterialParameter(j.at("constant").get<glm::dvec3>());
  } else if (hasKey(j, "mapped")) {
    auto texName = j.at("mapped").get<std::string>();
    auto texPath = pd.scene_dir / texName;
    // Due to lifetime issues, this must be constructed in-place in
    // the scene using getTexture. Bad things happen if you try
    // something else.
    p = MaterialParameter(pd.s->getTexture(texPath.string()));
  } else {
    auto s = j.begin().key();
    throw ParserException(
        "Material parameter must be either constant or mapped "
        "but it is " +
        s);
  }
  return p;
}

Material parseMaterial(const json &j, ParseData &pd) {
  // Base new material off of "current" (top-level) material
  Material m = pd.cur_mat;
  IGNORE_MISSING(m.setAmbient(parseMaterialParameter(j.at("ambient"), pd)));
  IGNORE_MISSING(m.setDiffuse(parseMaterialParameter(j.at("diffuse"), pd)));
  IGNORE_MISSING(m.setSpecular(parseMaterialParameter(j.at("specular"), pd)));
  IGNORE_MISSING(
      m.setReflective(parseMaterialParameter(j.at("reflective"), pd)));
  IGNORE_MISSING(
      m.setTransmissive(parseMaterialParameter(j.at("transmissive"), pd)));
  IGNORE_MISSING(m.setEmissive(parseMaterialParameter(j.at("emissive"), pd)));
  IGNORE_MISSING(m.setShininess(j.at("shininess").get<double>()));
  IGNORE_MISSING(m.setIndex(j.at("index").get<double>()));
  return m;
}

DirectionalLight *parseDirectionalLight(const json &j, ParseData &pd) {
  glm::dvec3 color = j.at("color").get<glm::dvec3>();
  glm::dvec3 direction = j.at("direction").get<glm::dvec3>();
  return new DirectionalLight(pd.s, direction, color);
}

PointLight *parsePointLight(const json &j, ParseData &pd) {
  glm::dvec3 color = j.at("color").get<glm::dvec3>();
  glm::dvec3 position = j.at("position").get<glm::dvec3>();
  float atten_pow_0 = 0.0f;
  float atten_pow_1 = 0.0f;
  float atten_pow_2 = 1.0f;
  IGNORE_MISSING(j.at("constant_attenuation_coeff").get_to(atten_pow_0));
  IGNORE_MISSING(j.at("linear_attenuation_coeff").get_to(atten_pow_1));
  IGNORE_MISSING(j.at("quadratic_attenuation_coeff").get_to(atten_pow_2));
  return new PointLight(pd.s, position, color, atten_pow_0, atten_pow_1,
                        atten_pow_2);
}

glm::dvec3 parseAmbientLight(const json &j) {
  glm::dvec3 color = j.at("color").get<glm::dvec3>();
  return color;
}

Sphere *parseSphereBody(const json &j, ParseData &pd) {
  Material m = GET_MAT_W_CUR(j, pd);
  auto s = new Sphere(pd.s, &m);
  s->setTransform(pd.getCurrentTransform());
  return s;
}

Box *parseBoxBody(const json &j, ParseData &pd) {
  Material m = GET_MAT_W_CUR(j, pd);
  auto b = new Box(pd.s, &m);
  b->setTransform(pd.getCurrentTransform());
  return b;
}

Square *parseSquareBody(const json &j, ParseData &pd) {
  Material m = GET_MAT_W_CUR(j, pd);
  auto s = new Square(pd.s, &m);
  s->setTransform(pd.getCurrentTransform());
  return s;
}

Cylinder *parseCylinderBody(const json &j, ParseData &pd) {
  Material m = GET_MAT_W_CUR(j, pd);
  auto c = new Cylinder(pd.s, &m);
  c->setTransform(pd.getCurrentTransform());
  IGNORE_MISSING(c->setCapped(j.at("capped").get<bool>()));
  return c;
}

Cone *parseConeBody(const json &j, ParseData &pd) {
  Material m = GET_MAT_W_CUR(j, pd);

  double bottomRadius = 1.0;
  double topRadius = 0.0;
  double height = 1.0;
  bool capped = true; // Capped by default

  IGNORE_MISSING(j.at("bottom_radius").get_to(bottomRadius));
  IGNORE_MISSING(j.at("top_radius").get_to(topRadius));
  IGNORE_MISSING(j.at("height").get_to(height));
  IGNORE_MISSING(j.at("capped").get_to(capped));

  auto c = new Cone(pd.s, &m, height, bottomRadius, topRadius, capped);
  c->setTransform(pd.getCurrentTransform());
  return c;
}

Trimesh *parseTrimeshBody(const json &j, ParseData &pd) {
  Material m = GET_MAT_W_CUR(j, pd);
  auto t = new Trimesh(pd.s, &m, pd.getCurrentTransform());
  bool genNormals = false;

  glm::dvec3 point;
  for (const json &pt_json : j.at("points")) {
    pt_json.get_to(point);
    t->addVertex(point);
  }

  std::vector<int> face;
  for (const json &f_json : j.at("faces")) {
    bool success = false;
    f_json.get_to(face);
    if (face.size() == 3) {
      success = t->addFace(face[0], face[1], face[2]);
    } else if (face.size() == 4) {
      success = t->addFace(face[0], face[1], face[2]);
      success &= t->addFace(face[0], face[2], face[3]);
    } else {
      auto s = std::to_string(face.size());
      throw ParserException("Got " + s +
                            " indices in a face: must be 3 or 4 indices");
    }

    if (!success) {
      throw ParserException("Error while adding face " + to_string(f_json) +
                            ". Maybe the point doesn't exist?");
    }
  }

  if (hasKey(j, "normals")) {
    glm::dvec3 normal;
    for (const json &n_json : j.at("normals")) {
      n_json.get_to(normal);
      t->addNormal(normal);
    }
    t->vertNorms = true;
  }

  if (hasKey(j, "materials")) {
    throw ParserException("\"materials\" key is no longer supported: use "
                          "obj_mesh instead");
  }

  IGNORE_MISSING(j.at("gennormals").get_to(genNormals));
  if (t->doubleCheck() != nullptr) {
    throw ParserException("Error in mesh: " + std::string(t->doubleCheck()));
  }
  if (genNormals) {
    t->generateNormals();
  }



  return t;
}

std::vector<Geometry *> parseGeometry(const json &j, ParseData &pd) {
  if (j.size() != 1) {
    std::stringstream ss;
    ss << "Geometry must be a one-element object, got " << j.size()
       << " elements with keys ";
    for (const auto &[k, v] : j.items()) {
      ss << k << " ";
    }
    throw ParserException(ss.str());
  }

  auto it = j.begin();
  std::string key = it.key();
  json val = it.value();

  if (key == "sphere") {
    return {parseSphereBody(val, pd)};
  } else if (key == "box") {
    return {parseBoxBody(val, pd)};
  } else if (key == "square") {
    return {parseSquareBody(val, pd)};
  } else if (key == "cylinder") {
    return {parseCylinderBody(val, pd)};
  } else if (key == "cone") {
    return {parseConeBody(val, pd)};
  } else if (key == "tri_mesh") {
    return {parseTrimeshBody(val, pd)};
  } else if (key == "obj_mesh") {
    std::vector<Trimesh *> trimeshes = parseObjmeshBody(val, pd);
    return std::vector<Geometry *>(trimeshes.begin(), trimeshes.end());
  } else {
    throw ParserException("Unknown geometry type: " + key);
  }
}

const std::string transformKeys[4] = {"rotate", "scale", "translate",
                                      "transform"};
bool isTransformKey(const std::string &s) {
  return std::find(std::begin(transformKeys), std::end(transformKeys), s) !=
         std::end(transformKeys);
}
const std::string geomKeys[7] = {"sphere", "box",      "square",  "cylinder",
                                 "cone",   "tri_mesh", "obj_mesh"};
bool isGeometryKey(const std::string &s) {
  return std::find(std::begin(geomKeys), std::end(geomKeys), s) !=
         std::end(geomKeys);
}

// Because transformations can have multiple children, we return a vector of
// Geometry. We build up this vector recursively by DFSing the tree built
// by the transformations, using ParseData to track the transform stack.
std::vector<Geometry *> parseTransform(const json &j, ParseData &pd) {
  auto it = j.begin();
  std::string key = it.key();
  json val = it.value();

  std::vector<Geometry *> geoms;
  // For all except "rotate", this is the right location for the child
  // data. We will need to overwrite this when dealing with a rotate key.
  json &children = val.at(1);

  if (key == "rotate") {
    glm::dvec3 axis = val.at(0).get<glm::dvec3>();
    double angle = val.at(1).get<double>();
    glm::dmat4 transform = glm::rotate(glm::dmat4(1.0), angle, axis);
    pd.transformStack.push_back(transform);
    children = val.at(2);
  } else if (key == "scale") {
    glm::dvec3 scale = val.at(0).get<glm::dvec3>();
    glm::dmat4 transform = glm::scale(glm::dmat4(1.0), scale);
    pd.transformStack.push_back(transform);
  } else if (key == "translate") {
    glm::dvec3 translate = val.at(0).get<glm::dvec3>();
    glm::dmat4 transform = glm::translate(glm::dmat4(1.0), translate);
    pd.transformStack.push_back(transform);
  } else if (key == "transform") {
    std::vector<double> tvector = val.at(0).get<std::vector<double>>();
    glm::dmat4 transform = glm::make_mat4(tvector.data());
    pd.transformStack.push_back(transform);
  } else {
    throw ParserException("Unknown transform type: " + key);
  }

  // Recursively process each child element which has this transform
  // applied.
  for (const auto &obj : children) {
    std::string key = obj.begin().key();
    if (isTransformKey(key)) {
      auto subgeoms = parseTransform(obj, pd);
      geoms.insert(geoms.end(), subgeoms.begin(), subgeoms.end());
    } else {
      auto subgeoms = parseGeometry(obj, pd);
      geoms.insert(geoms.end(), subgeoms.begin(), subgeoms.end());
    }
  }

  pd.transformStack.pop_back();
  return geoms;
}

Scene *JsonParser::parseScene() {
  // Allow comments and exceptions while parsing
  json j = json::parse(this->contents);

  Scene *scene = new Scene();
  ParseData pd;
  pd.s = scene;
  pd.scene_dir = this->fileDirPath;

  for (const auto &object : j) {
    std::string key = object.begin().key();
    json val = object.begin().value();

    if (key == "camera") {
      scene->getCamera() = parseCamera(val);
    } else if (key == "material") {
      // Need to reset the top-level material so that we don't
      // pollute the new material with old values
      pd.cur_mat = Material{};
      pd.cur_mat = parseMaterial(val, pd);
    } else if (key == "ambient_light") {
      scene->addAmbient(parseAmbientLight(val));
    } else if (key == "directional_light") {
      scene->add(parseDirectionalLight(val, pd));
    } else if (key == "point_light") {
      scene->add(parsePointLight(val, pd));
    } else if (isTransformKey(key)) {
      auto geoms = parseTransform(object, pd);
      for (auto g : geoms) {
        scene->add(g);
      }
    } else if (isGeometryKey(key)) {
      auto geoms = parseGeometry(object, pd);
      for (const auto &geom : geoms) {
        scene->add(geom);
      }
    } else {
      throw ParserException("Unknown scene object type: " + key);
    }
  }


  return scene;
}

// Helper function to set parts of a Material from a tinyobj material
void MaterialFromTinyObj(Material *target, tinyobj::material_t mat) {
  target->setAmbient(glm::make_vec3(mat.ambient));
  target->setDiffuse(glm::make_vec3(mat.diffuse));
  target->setSpecular(glm::make_vec3(mat.specular));
  target->setEmissive(glm::make_vec3(mat.emission));
  target->setTransmissive(glm::make_vec3(mat.transmittance));
  target->setShininess(mat.shininess);
  target->setIndex(mat.ior);
}

// Cantor hash function. We need at least 64 bits in the computation, since the
// 32-bit computation potentially overflows when a + b = 2**16 (way too small)
size_t hash2(uint64_t a, uint64_t b) { return ((a + b) * (a + b + 1) / 2) + b; }
size_t hash3(uint64_t a, uint64_t b, uint64_t c) {
  return hash2(hash2(a, b), c);
}
struct TinyObjIndexHash {
  std::size_t operator()(const tinyobj::index_t &i) const {
    return hash3(i.vertex_index, i.normal_index, i.texcoord_index);
  }
};
struct TinyObjIndexEq {
  bool operator()(const tinyobj::index_t &a, const tinyobj::index_t &b) const {
    return a.vertex_index == b.vertex_index &&
           a.normal_index == b.normal_index &&
           a.texcoord_index == b.texcoord_index;
  }
};

/* The full OBJ file format is chaotic neutral. To try to tame some of this, we
only support certain features. See jsonformat.md for the limitations.
*/
Trimesh *loadObjToTrimesh(const tinyobj::ObjReader &rdr,
                          const tinyobj::shape_t &s, Trimesh *t,
                          ParseData &pd) {
  auto &attrib = rdr.GetAttrib();
  auto &materials = rdr.GetMaterials();

  /* Faces in OBJ files can use different indices for
     UV/normals/positions. For example, naively you can specify a face as
     (1, 2, 3), meaning use vertex positions 1/2/3, UV coordinates 1/2/3,
     etc. However, we can also mix and match, e.g. (1/1/1, 2/2/2, 3/1/1)
     would use the same vertex positions, but use the first vertex's
     UV/normal on the third vertex.

     Instead of dealing with this during rendering, we preprocess the OBJ
     file so that every unique combination of v/vt/vn gets its own index
     in the Trimesh. This increases memory usage slightly, but most renderers
     (incl. OpenGL) need separate arrays of indices anyways.
  */

  bool warned = false;
  std::unordered_map<tinyobj::index_t, int, TinyObjIndexHash, TinyObjIndexEq>
      indexMap;

  // If this v/vt/vn combination has been seen before, return the linear
  // index. Otherwise, create it by inserting the combination into the
  // mesh.
  auto getOrCreateLinearIndex = [&attrib, &indexMap, &warned,
                                 t](tinyobj::index_t i) {
    if (indexMap.find(i) == indexMap.end()) {
      if (indexMap.size() > MAX_RECOMMENDED_VERTS && !warned) {
        std::cerr << "WARN: Detected many vertices in OBJ input. This may "
                     "cause memory problems. Consider decimating the mesh."
                  << std::endl;
        warned = true;
      }

      indexMap[i] = indexMap.size();
      t->addVertex(glm::make_vec3(&attrib.vertices[3 * i.vertex_index]));
      if (i.normal_index != -1) {
        auto n = glm::make_vec3(&attrib.normals[3 * i.normal_index]);
        // OBJ normals are not required to be normalized; ours are
        t->addNormal(glm::normalize(n));
      }

      if (i.texcoord_index != -1) {
        t->addUV(glm::make_vec2(&attrib.texcoords[2 * i.texcoord_index]));
      }
      if (attrib.colors.size() > 0) {
        t->addColor(glm::make_vec3(&attrib.colors[3 * i.vertex_index]));
      }
    }
    return indexMap[i];
  };

  // TinyOBJ triangulates for us, so we don't have to check for larger
  // faces
  for (long unsigned f = 0; f < s.mesh.indices.size(); f += 3) {
    auto i0 = getOrCreateLinearIndex(s.mesh.indices[f]);
    auto i1 = getOrCreateLinearIndex(s.mesh.indices[f + 1]);
    auto i2 = getOrCreateLinearIndex(s.mesh.indices[f + 2]);
    t->addFace(i0, i1, i2);
  }

  /* Finished parsing geometry, now parse materials. The parser currently
  only supports a single material per mesh, because to do otherwise
  would require modification of the Trimesh class itself.

  If you want to support multiple materials, you need to do the following:

       1. Modify the Trimesh class to support multiple materials in an
          array or vector
       2. Loop over the `materials` array here and place each material in
          the materials vector in the Trimesh
       3. For each face with index f, access s.mesh.material_ids[f] to get
          the index of the material for that face. Record this
          information in the Trimesh somehow (perhaps modifying the
          addFace method)
       4. When rendering an intersection with a face, look up the
          corresponding material in the Trimesh, then use that as the
          material for the intersection phase (including barycentric
          interpolation of the material if it is called for).

// Face f should render using materials[k]
auto k = s.mesh.material_ids[f];
*/

  // Take the first material associated with the mesh and use it.
  Material *m = new Material();
  if (materials.size() > 0) {
    tinyobj::material_t mtl = materials[0];
    m->setDiffuse(glm::make_vec3(mtl.diffuse));
    m->setSpecular(glm::make_vec3(mtl.specular));
    m->setAmbient(glm::make_vec3(mtl.ambient));
    m->setTransmissive(glm::make_vec3(mtl.transmittance));
    m->setEmissive(glm::make_vec3(mtl.emission));
    m->setShininess(mtl.shininess);
    m->setIndex(mtl.ior);

    if (!mtl.diffuse_texname.empty()) {
      std::string texPath = (pd.scene_dir / mtl.diffuse_texname).string();
      m->setDiffuse(MaterialParameter(pd.s->getTexture(texPath)));
    }

    if (!mtl.specular_texname.empty()) {
      std::string texPath = (pd.scene_dir / mtl.specular_texname).string();
      m->setSpecular(MaterialParameter(pd.s->getTexture(texPath)));
    }
  }

  t->setMaterial(m);

  if (attrib.normals.size() > 0) {
    t->vertNorms = true;
  }

  const char *err = t->doubleCheck();
  if (err != nullptr) {
    throw ParserException("Error while parsing OBJ file: " + std::string(err));
  }

  return t;
}

std::vector<Trimesh *> parseObjmeshBody(const json &j, ParseData &pd) {
  std::string objFile = j.at("objfile").get<std::string>();
  std::string path = (pd.scene_dir / objFile).string();
  bool genNormals = false;
  IGNORE_MISSING(j.at("gennormals").get_to(genNormals));

  std::vector<Trimesh *> results;

  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = pd.scene_dir.string();
  reader_config.triangulate = true;
  reader_config.vertex_color = false; // Populate vertex colors only if
                                      // *all* vertices have associated colors
  tinyobj::ObjReader reader;
  bool success = reader.ParseFromFile(path, reader_config);

  if (!success) {
    if (!reader.Error().empty()) {
      throw ParserException("Error while parsing OBJ file: " + reader.Error());
    } else {
      throw ParserException("Error while parsing OBJ file: unknown "
                            "(tinyobj returned an error with no message)");
    }
  }
  if (!reader.Warning().empty()) {
    std::cerr << "TinyObj warnings: " << reader.Warning();
  }

  auto &attrib = reader.GetAttrib();
  auto &shapes = reader.GetShapes();

  if (attrib.vertices.size() / 3 > MAX_RECOMMENDED_VERTS) {
    std::cerr << "Warning: OBJ file " << objFile << " has "
              << attrib.vertices.size() / 3 << " vertices. "
              << "This may cause an out-of-memory condition. "
              << "Consider reducing the number of vertices in the "
                 "OBJ file."
              << std::endl;
  }

  for (const tinyobj::shape_t &s : shapes) {
    Trimesh *t = new Trimesh(pd.s, &pd.cur_mat, pd.getCurrentTransform());

    loadObjToTrimesh(reader, s, t, pd);

    if (genNormals) {
      t->generateNormals();
    }



    results.push_back(t);
  }
  return results;
}
