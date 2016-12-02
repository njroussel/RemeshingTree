#include <surface_mesh/Surface_mesh.h>
#include <surface_mesh/types.h>
#include "mesh_processing/mesh_processing.h"
#include <Eigen/Sparse>
#include <cfloat>

typedef surface_mesh::Surface_mesh Mesh;

surface_mesh::Vec3 get_highest_point(const Mesh &mesh);
surface_mesh::Vec3 get_lowest_point(const Mesh &mesh);
