JSON RayTracer File Format
==========================

This document describes the file format of the JSON RayTracer input. This format
was derived by mechanical translation from the earlier .ray format. The exact
.ray format used in this project is undocumented, but it is closely related to
the CMU .ray format, whose documentation can be found 
[online](https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/15864-s04/www/assignment4/format.html).

The information in this file is not sufficient to obtain a fully-working scene
reader, because there are undocumented default values for several types of
objects. For example, `point_light`s need three attenuation coefficients, but
some rayfiles do not list all three coefficients. To deal with this, the default
C++ parser has fallback values within the parser that are used when an
attentuation coefficient is not specified by the file. Similar things happen for
material values, or for the parameters for certain shapes. In order to write a
scene reader which unambiguously parses all scenes correctly, you will need
information on the correct default values taken from the C++ Ray parser.

## Basic Information

The raytracer input is a JSON format. The top-level entry in the file is
always an array, which contains various JSON objects. A JSON object in this
top-level array is always a single-key object, whose key describes the type of
object, and whose values describe information about the object.

For example, the following object declaration describes a directional light,
whose direction and color are determined by the values of the object:

```
    "directional_light": {
      "direction": [0.0, -1.0, 0.0],
      "color": [1.0, 1.0, 1.0]
    }
```

The valid object types (keys) are:
  - `camera`
  - `ambient_light`
  - `point_light`
  - `directional_light`
  - `sphere`
  - `box`
  - `square`
  - `cylinder`
  - `cone`
  - `tri_mesh`
  - `obj_mesh`
  - `material`
  - `transform`

The rest of this document describes each of these object types in slightly more
detail. Most of the objects are fairly mundane and specify a single entity. 
The `tri_mesh`, `material`, and `transform` objects are a little more involved 
and affect other objects.

Vectors are represented by lists of numbers of the appropriate length, i.e.
3-vectors are a list of 3 numbers, and 4-vectors are a list of 4 numbers. Other
numerical type representations are described below when they are relevant.

The order of objects in the top-level array does not matter, **except when
dealing with materials**. See the material section for further details.

## Lights and Camera

Lights determine the lighting within a scene, while a camera captures the
camera parameters. While the original rayfile format supports both "color"
and "colour" as keys, the JSON only uses "color". Colors are 3-vectors 
consisting of RGB components in the range `[0.0, 1.0]`.

#### camera

The camera declaration describes the position and orientation of the virtual
camera and the geometry of the frustum. It has the following parameters: 

- `position`: the 3D position of the eye.
- `viewdir`: the direction in which the camera is looking.
- `updir`: the orientation of the camera with respect to viewdir (which way is up?).
- `aspectratio`: the aspect ratio of the projection plane (width/height).
- `fov`: the *vertical* angle of the vertex of the frustum, measured in degrees.

There should only be one camera in a scene, and the behavior is unspecified if
there are multiple. Most implementations will simply use the last camera.

Example:

```json
{
  "camera": {
    "position": [1.0, 1.0, -4.0],
    "viewdir": [0.0, 0.0, 1.0],
    "updir": [0.0, 1.0, 0.0],
    "aspectratio": 1.0
  }
}
```

#### ambient_light

An ambient_light corresponds to an ambient light source. When computing the
ambient light for the scene, the sum of all the ambient lights is taken.

Example:

```json
{ "ambient_light": { "color": [1.0, 0.0, 0.1] } }
```

#### directional_light

A directional_light is a light source where energy radiates equally in a 
direction from infinitely far away. It has two parameters:

- `direction`: the direction vector of the directional source.
- `color`: the color of the emitted light.

Example:

```json
{
  "directional_light": {
    "direction": [0.0, 1.0, 0.0],
    "color": [0.2, 0.2, 0.2]
  }
}
```

#### point_light

A point_light is a light source where energy radiates equally in all directions
from a single point. It has the following parameters:

- `position`: the 3D position of the point source.
- `color`: the color of the emitted light.
- `constant_attenuation_coeff`, `linear_attenuation_coeff`, `quadratic_attenuation_coeff`: Coefficients controlling the distance attenuation of the light.

Example:

```json
{
  "point_light": {
    "position": [4.0, 4.0, 0.0],
    "color": [0.5, 0.5, 0.5],
    "constant_attenuation_coeff": 0.25,
    "linear_attenuation_coeff": 0.003372407,
    "quadratic_attenuation_coeff": 0.000045492
  }
}
```

## Geometries

These objects specify things in the scene to be raytraced. Some of them have a 
limited number of options to control their geometry, but if you want to modify
the size/location of these entities, you should use transforms.

All geometries accept a material (discussed below) as a member, which affects
the surface material of that geometry.

#### sphere

A sphere is a unit (radius 1) sphere centered at the origin. It has no intrinsic
parameters.

Example:

```json
{
  "sphere": {
    "material": {
      "diffuse": [0.34, 0.07, 0.56],
    }
  }
}
```

#### box

A box is a unit (side length 1) cube centered at the origin (it goes from
(-0.5,-0.5,-0.5) to (0.5,0.5,0.5)). Like the sphere, it has no intrinsic
parameters. 

Example:

```json
{
  "box": {
    "material": {
      "ambient": [0.5, 0.5, 0.5],
      "diffuse": [0.0, 0.858943, 0.174108]
    }
  }
}
```

#### square

A square is a unit (side length 1) square centered at the origin and lying in
the XY plane (its opposite corners are (-0.5,-0.5,0) and (0.5,0.5,0)). It has no
intrinsic parameters. 

```json
{ 
  "square": { 
    "material": { 
      "diffuse": [0.4, 0.4, 0.4] 
    } 
  }
}
```

#### cylinder

A cylinder is a radius 1 cylinder. Its central axis lies on the Z axis and its
ends are at Z = 0 and Z = 1. It has one intrinsic parameter:

- `capped`: a boolean indicating whether the cylinder has circular caps or not.

Example:

```json
{
  "cylinder": {
    "material": {
      "specular": [0.9, 0.4, 0.0],
      "diffuse": [0.8, 0.3, 0.1],
      "shininess": 76.8
    }
    "capped": false
  }
}
```

#### cone

A cone is a generalized cylinder with central axis on the Z axis. It takes a
height parameter, and runs from Z = 0 to Z = height. The radius of each endpoint
can be specified, and caps can be turned on and off.

- `capped`: a boolean indicating whether the cone has caps or not.
- `height`: a scalar giving the height of the cone.
- `bottom_radius`: a scalar giving the radius at Z = 0.
- `top_radius`: a scalar giving the radius at Z = height.

Example:

```json
{
  "cone": {
    "height": 2.0,
    "bottom_radius": 0.25,
    "top_radius": 0.25,
    "material": { "diffuse": [0.5, 0.5, 0.5] }
  }
}
```

#### tri_mesh

A tri_mesh is a container for polygonal data. To specify a tri_mesh, specify an array
of vertices, followed by an array of faces, using the parameters below:

- `points`: A list of the points in the mesh.
- `normals`: A list with one normal for each point. (optional)
- `faces`: A list of faces. A face is a list of indices. Each index specifies
   one of the points (counting from zero), and each tuple of indices specifies 
   the points in a face in counter-clockwise order.
- `gennormals`: A boolean. If this is set to be true then per-vertex normals 
   will be automatically generated for the mesh, overwriting existing normals
   if any exist.

In spite of their name, tri_meshes admit quad data as well, so faces may be
either three or four indices.

Notes:
 - If `normals` is specified or more than one material is given, the lists must
   have the same number of elements as the `points` list.
 - If normals are not provided and `gennormals` is false, there will be no
   normal information for the mesh, meaning certain parts of shading models
   (e.g. diffuse for Phong shading) are impossible to compute.

Example:

```json
"tri_mesh": {
  "points": [
    [0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [1.0, 1.0, 1.0],
    [1.0, 0.0, 1.0]
  ],
  "faces": [
    [0, 1, 2],
    [6, 5, 4],
    [3, 2, 6],
    [4, 0, 3],
    [4, 3, 7]
  ],
  "material": [
    {
      "ambient": [0.0, 0.0, 0.0],
      "specular": [0.0, 0.0, 0.0],
      "diffuse": [0.1, 0.1, 1.0]
    }
  ]
}
```

#### obj_mesh

An obj_mesh is a container for polygonal data. Instead of explicitly listing
points and faces, it takes in a Wavefront OBJ file and parses it to get triangle
data.

The following parameters are supported:
  - `objfile`: The name of the OBJ file to load.
  - `gennormals`: A boolean. If this is set to be true then per-vertex normals 
     will be automatically generated for the mesh, overwriting existing normals
     if any exist.

Note that the OBJ file format is a terrible mess. It allows things like 
multiple meshes per file, multiple materials per mesh, different rendering
models, polynomial splines instead of flat surfaces, etc. etc. In order to
stay sane and avoid having to implement high-order nonsense, we restrict
the OBJs accepted by the parser to the following:

- Fewer than 5,000,000 vertices
- Per-vertex colors are **allowed**, see below for details.
- Using vertices, vertex textures, and vertex normals (`v`, `vt`, and `vn`)
- **Must have only one material per mesh**
- Only the following keys are supported for materials, all others are ignored:
   + `Kd` (diffusive)
   + `Ks` (specular)
   + `Ka` (ambient)
   + `Tf` (transmissive)
   + `Ke` (emissive)
   + `Ns` (shininess)
   + `Ni` (index of refraction)
   + `map_Kd` (texture-mapped diffusive)
   + `map_Ks` (texture-mapped specular)

In practice, the one-material-per-mesh requirement in bold is the one which 
most severely limits the set of OBJ files that we can render. Fortunately, once
you understand how mesh rendering works, it is not difficult to add support
for multiple materials. If you would like to do this, see the giant comment
in `loadObjToTrimesh` in `JsonParser.cpp`.

This parser does support per-vertex colors, which is a nonstandard extension to
the OBJ file format. If you would like to specify per-vertex colors, give the
`v` line six arguments instead of 3: the first 3 are x,y,z values and the last
3 are r,g,b values in the range of [0.0, 1.0]. Note that such OBJs may not be
readable by other software, since this is a nonstandard extension.

Texture files specified using `map_Kd` or `map_Ks` must be either in BMP or PNG
format, as we do not have a library for loading JPEGs. If you need to convert,
the ImageMagick utilities `convert` and `mogrify` may be useful. You will also
need to edit the filepath specified by the `map_*` directives.

Finally, if you are attempting to load OBJ files downloaded from the internet,
note that most OBJ-exporting software is hot garbage. For example, there are 
a surprising number of OBJs out there that specify a path to an MTL file which 
includes a username. Naturally, this is not going to work on anyone's computer
except the person who exported it. You should open both the OBJ and any MTL
files in the export and check them to make sure no such nonsense has occurred.

## Transformations

Transformations are used to transform objects. Logically, transformations have
two components:

- Data which is used to specify the transformation itself.
- One or more children. These children can either be geometries (in which case
the transformation is applied directly to that geometry) or more transformations,
which causes the transformations to be composed.

Due to issues with JSON parsing, these are represented by a single-key object
whose key is the type of transformation, and whose value is an array containing
transformation information. The layout of the array depends on the type of
transformation, and is described further below.

#### translate

A translate moves its children by the specified translation. The first element
of the array is a 3-vector giving the translation, and the second element is an
array containing all the children.

Example:

```json
{
  "translate": [
    [1, 2, 3],
    [
      {"sphere": { }},
      {"box": { }}
    ]
  ]
}
```

#### scale

A scale scales the objects. All scales are specified by a 3-vector which gives
the scaling in each axis--this is the first element of the array. The second
element is an array containing all the children.

Example:

```json
{
  "scale": [
    [1, 0.8, 1.2],
    [
      {"sphere": { }},
      {"box": { }}
    ]
  ]
}
```

#### rotate

A generalized rotation about the given axis. The first element of the array is
a 3-vector giving the axis of rotation. The second element of the array is a
scalar specifying the angle of rotation in radians. The third element of the
array contains all the children.

Example:

```json
{
  "rotate": [
    [1, 0, 0],
    0.731,
    [
      {"sphere": { }},
      {"box": { }}
    ]
  ]
}
```


#### transform

This is a generalized transform, which applies an arbitrary 4x4 matrix to all
its children. The matrix is given as a list of 16 numbers, to be interpreted as
a matrix written in column-major order. This is the first element of the array.
The second element of the array contains all the children of the transform.

This example applies the following transformation matrix to a sphere:

```
1 0 0 0
2 1 0 0
3 0 1 -35
0 0 0 1
```

```json
{
  "transform": [
    [
        1.0, 2.0, 3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        -35.0, 1.0
    ],
    [
      { "sphere": { } }
    ]
  ]
}
```

## Materials

Materials describe what the surface of an object looks like. It has the following
keys:

- `emissive`: $k_e$, the emissive color.
- `ambient`: $k_a$, the ambient color.
- `specular`: $k_s$, the specular color.
- `reflective`: $k_r$, the reflective color.
- `diffuse`: $k_d$, the diffuse color.
- `transmissive`: $k_t$, the ability for this material to transmit light in each channel.
- `shininess`: $n_s$, the shininess (between 0 and 1).
- `index`: the material's index of refraction.


For all of the keys except for `shininess` and `index`, the material parameter
is a JSON object with a single key, which must be either `"constant"` or
`"mapped"`. If it is `"constant"`, then the associated value must be a list of 3
numbers, interpreted as an RGB value. As an example, 
`"diffuse": { "constant": [0.0, 1.0, 0.1] }` is a valid diffuse parameter, 
but `"diffuse": [0.0, 1.0, 0.1]` is not. This is a little clunky when writing
files by hand, but makes parsing of the resulting JSON much simpler.

If the parameter's key is `"mapped"`, the associated value is a string, which
is interpreted as a file path relative to the JSON file. This file is loaded and
used as a texture map for that parameter. Note that only `box`es currently
admit a natural (u,v) parameterization for texture mapping. If you would like
to map a mesh, look at the `obj_mesh` geometry.

Finally, materials can be declared as part of a geometry, or in the top-level
array of the file, not associated with any specific geometry. When declared as 
part of a geometry, the material only affects that particular geometry. When 
declared at the top level of the file, it sets default material parameters for 
all geometries that occur after the material declaration until another 
top-level material declaration (or the end of the file).

This example shows both specular and diffuse parameters being specified for a
box, with the diffuse parameter being mapped from a file while the specular
is set from a vector.

```json
{
  "box": {
    "material": {
      "specular": {"constant" : [0.9, 0.4, 0.0] },
      "diffuse": { "mapped": "box_map1.bmp" },
      "shininess": 76.8
    }
  }
} 
```

For an example of top-level material declarations, look at `tentacles.json`, where
a top-level material sets the ambient and specular default values, while
subsequent geometry declarations set their own diffuse parameters.
