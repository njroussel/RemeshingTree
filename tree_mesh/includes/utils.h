#include <surface_mesh/Surface_mesh.h>
#include <surface_mesh/types.h>
#include "mesh_processing/mesh_processing.h"
#include <Eigen/Sparse>
#include <cfloat>

typedef surface_mesh::Surface_mesh Mesh;

Mesh::Vertex get_highest_point(const Mesh &mesh);
Mesh::Vertex get_lowest_point(const Mesh &mesh);
