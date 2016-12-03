#include "mesh_processing/tree_processing.h"

namespace mesh_processing {
    TreeProcessing::TreeProcessing(const std::string& filename,
            const std::string& sphere_filename,
            const std::string& cylinder_filename,
            const float sphere_base_diameter,
            const float cylinder_base_diameter) : WireframeProcessing(filename, sphere_filename, cylinder_filename) {
        sphere_base_diameter_ = sphere_base_diameter;
        cylinder_base_diameter_ = cylinder_base_diameter;
    }

    TreeProcessing::~TreeProcessing() {}

    void TreeProcessing::fill_vertex_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale) {
        const float lowest_point = get_lowest_point(mesh_)[1];
        const float highest_point = get_highest_point(mesh_)[1];
        
        for (Mesh::Vertex v : mesh_.vertices()) {
            float relative_height = (mesh_.position(v)[1] - lowest_point) / (highest_point - lowest_point);
            v_inwireframe[v] = true;
            v_scale[v] = (1.0f - relative_height) * sphere_base_diameter_;
        }
    }

    void TreeProcessing::fill_edge_wireframe_properties(Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale) {
        const float lowest_point = get_lowest_point(mesh_)[1];
        const float highest_point = get_highest_point(mesh_)[1];
    
        for (Mesh::Edge e : mesh_.edges()) {
            Mesh::Vertex v0 = mesh_.vertex(e, 0);
            Mesh::Vertex v1 = mesh_.vertex(e, 1);
            Point p0 = mesh_.position(v0);
            Point p1 = mesh_.position(v1);

            float relative_height = (((p0+p1)/2.0f)[1] - lowest_point) / (highest_point - lowest_point);
            e_inwireframe[e] = true;
            e_scale[e] = (1.0f - relative_height) * cylinder_base_diameter_;
        }
    }
}
