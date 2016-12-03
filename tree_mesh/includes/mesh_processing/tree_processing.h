#include "mesh_processing/wireframe_processing.h"

namespace mesh_processing {
    class TreeProcessing : public WireframeProcessing {
        public:
            TreeProcessing(const std::string& filename,
                           const std::string& sphere_filename,
                           const std::string& cylinder_filename);
            ~TreeProcessing();

            virtual void fill_vertex_wireframe_properties(void);
            virtual void fill_edge_wireframe_properties(void);
    };
}
