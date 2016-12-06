#pragma once

#include "mesh_processing/wireframe_processing.h"
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <tuple> 
#include <queue>

#define BETTER_ALGO 
#define USE_STACK
#define SORT_BY_ANGLE

namespace mesh_processing {
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
#ifdef USE_STACK
    void inner_fill(std::queue<Mesh::Vertex> to_process, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length);
#else
		    void inner_fill(Mesh::Vertex root, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length);
#endif

            float sphere_base_diameter_;
            float cylinder_base_diameter_;
    };
}
