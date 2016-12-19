#pragma once
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


template <class T> void remove_vertex_property_with_name(Mesh &mesh, const std::string &name) {
    Mesh::Vertex_property<T> p = mesh.get_vertex_property<T>(name);
    mesh.remove_vertex_property<T>(p);
}

template <class T> void remove_edge_property_with_name(Mesh &mesh, const std::string &name) {
    Mesh::Edge_property<T> p = mesh.get_edge_property<T>(name);
    mesh.remove_edge_property(p);
}
