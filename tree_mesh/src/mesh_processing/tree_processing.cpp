#include "mesh_processing/tree_processing.h"

namespace mesh_processing {
    TreeProcessing(const std::string& filename,
            const std::string& sphere_filename,
            const std::string& cylinder_filename) : WireframeProcessing(filename, sphere_filename, cylinder_filename) {}

    ~TreeProcessing() {}

    void fill_vertex_wireframe_properties(void) {
        surface_mesh::Vec3 lowest_point = get_lowest_point(mesh_);
        surface_mesh::Vec3 highest_point = get_highest_point(mesh_);
        
        for (Mesh::Vertex v : mesh_.vertices()) {

        }
    }

    void fill_edge_wireframe_properties(void) {

    }
}
