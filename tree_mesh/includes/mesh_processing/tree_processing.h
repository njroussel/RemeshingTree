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
    }branch_t;

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
            virtual void fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<std::pair<float, float>> e_scale);

            void inner_fill(std::queue<branch_t> to_process);

            std::vector<Mesh::Vertex> get_neighbors(Mesh::Vertex v, bool not_in_wireframe);

            bool is_root(Mesh::Vertex v);
            Mesh::Vertex get_lowest_root_vertex(void);
            bool split(Mesh::Vertex v);
            std::pair<Mesh::Vertex, Mesh::Vertex> get_best_split_pair(Mesh::Vertex v, std::vector<Mesh::Vertex> neighbors);

            float sphere_base_diameter_;
            float cylinder_base_diameter_;
            float max_length_ = 15.0f;
            std::vector<Mesh::Vertex> roots_;

            /* All the properties, they will be the same as the ones given in
             * fill_wireframe_properties() above. This way we can use them in
             * any helper functions without having to pass them as argument. */
            Mesh::Vertex_property<bool> v_inwireframe_;
            Mesh::Vertex_property<surface_mesh::Vec3> v_scale_;
            Mesh::Edge_property<bool> e_inwireframe_;
            Mesh::Edge_property<std::pair<float, float>> e_scale_;
            Mesh::Vertex_property<float> v_abs_length_;
            Mesh::Vertex_property<float> v_rel_length_;
            Mesh::Vertex_property<bool> v_root_;
    };
}
