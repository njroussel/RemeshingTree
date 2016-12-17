#include <surface_mesh/Surface_mesh.h>
#include <surface_mesh/types.h>
#include "mesh_processing/mesh_processing.h"
#include <Eigen/Sparse>
#include <cfloat>

typedef surface_mesh::Surface_mesh Mesh;

/* Returns the highest vertex of a given mesh. */ 
Mesh::Vertex get_highest_point(const Mesh &mesh);

/* Returns the lowest vertex of a given mesh. */ 
Mesh::Vertex get_lowest_point(const Mesh &mesh);
