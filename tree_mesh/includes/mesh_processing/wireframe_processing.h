#pragma once

#include "mesh_processing/mesh_processing.h"
#include "utils.h"

namespace mesh_processing {
    class WireframeProcessing : public MeshProcessing {
        public:
            WireframeProcessing(const std::string& filename,
                                const std::string& sphere_filename,
                                const std::string& cylinder_filename);
            ~WireframeProcessing(void);

            void create_wire_frame(void);

	    void export_mesh(const std::string& filename);

        protected:
            void replace_vertices();
            void replace_edges();
            void insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot_axis, const float rot_angle, const surface_mesh::Vec3 scale);
            virtual void fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<std::pair<float, float>> e_scale);

            Mesh sphere_;
            Mesh cylinder_;
            Mesh result_; /* Dirty, but temporary. */
    };
}
