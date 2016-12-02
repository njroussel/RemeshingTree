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

            void create_wire_frame(const float spheres_radius,
                                   const float cylinder_radius);

        private:
            void replace_vertices(const float spheres_radius);
            void replace_edges(const float cylinder_radius);
            void insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot_axis, const float rot_angle, const surface_mesh::Vec3 scale);
            Mesh sphere_;
            Mesh cylinder_;
            Mesh result_; /* Dirty, but temporary. */
    };
}

#endif /* WIREFRAME_PROCESSING_H */
