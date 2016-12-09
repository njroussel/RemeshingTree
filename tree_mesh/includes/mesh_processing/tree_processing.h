#pragma once

#include "mesh_processing/wireframe_processing.h"
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <tuple> 
#include <queue>

#define BETTER_ALGO 
#define USE_STACK

namespace mesh_processing {

    typedef struct {
        Mesh::Vertex root;
        Mesh::Vertex last;
    }recursion_data_t;

    class TreeProcessing : public WireframeProcessing {
        public:
            TreeProcessing(const std::string& filename,
                           const std::string& sphere_filename,
                           const std::string& cylinder_filename,
                           const float sphere_base_diameter,
                           const float cylinder_base_diameter);
            ~TreeProcessing();
	    
	    void export_mesh(const std::string& filename);

        private:
            virtual void fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale);
            void build_main_root(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length);
            void build_main_root_inner(Mesh::Vertex root, Mesh::Vertex_property<bool> v_root, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length);
            void inner_fill(std::queue<recursion_data_t> to_process, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length);

            float sphere_base_diameter_;
            float cylinder_base_diameter_;
            float max_root_length;
    };
}
