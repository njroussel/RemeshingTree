#pragma once

#include "mesh_processing/wireframe_processing.h"

namespace mesh_processing {
    class TreeProcessing : public WireframeProcessing {
        public:
            TreeProcessing(const std::string& filename,
                           const std::string& sphere_filename,
                           const std::string& cylinder_filename,
                           const float sphere_base_diameter,
                           const float cylinder_base_diameter);
            ~TreeProcessing();

        private:
            virtual void fill_vertex_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale);
            virtual void fill_edge_wireframe_properties(Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale);

            float sphere_base_diameter_;
            float cylinder_base_diameter_;
    };
}
