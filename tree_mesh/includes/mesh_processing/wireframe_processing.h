#ifndef WIREFRAME_PROCESSING_H
#define WIREFRAME_PROCESSING_H

#include "mesh_processing/mesh_processing.h"
#include "utils.h"

namespace mesh_processing {
    class WireframeProcessing : public MeshProcessing {
        public:
            WireframeProcessing(const std::string& filename,
                                const std::string& sphere_filename,
                                const std::string& cylinder_filename);
            ~WireframeProcessing(void);

            virtual void fill_vertex_wireframe_properties(void);
            virtual void fill_edge_wireframe_properties(void);
            void create_wire_frame(void);

        protected:
            void replace_vertices();
            void replace_edges();
            void insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot_axis, const float rot_angle, const surface_mesh::Vec3 scale);

            Mesh sphere_;
            Mesh cylinder_;
            Mesh result_; /* Dirty, but temporary. */
    };
}

#endif /* WIREFRAME_PROCESSING_H */
