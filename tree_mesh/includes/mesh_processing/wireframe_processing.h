#pragma once

#include "mesh_processing/mesh_processing.h"
#include "utils.h"

namespace mesh_processing {
    /*
     * Class WireframeProcessing.
     * Create a wireframe version of a given mesh using spheres and
     * cylinders.
     * This class can be seen as a baseline for different algorithms
     * that produce an altered version of a wireframe rendering.
     *
     * Subclasses can redefine the fill_wireframe_properties that will be called
     * upon wireframe creation.
     *
     * It is important to note that vertices and edges have now two new
     * properties related to the wireframe :
     *      1. v_inwireframe : type bool : tells if the given vertex will be
     *              replaced by a sphere upon wireframe creation (true) or
     *              left ignored (false).
     *      2. v_scale : type vec3 : The scaling factor of the sphere that will
     *              replace this vertex.
     *
     * for edges :
     *      1. e_inwireframe : Same as v_inwireframe.
     *      2. e_scale : type pair (float, float) differs a bit compared to
     *              v_scale. To sum up : cylinders always span between the
     *              two vertices that it connects, futhermore, the scale of the
     *              diameter is not the same at both ends, so that is the 2
     *              vertices are not of the same 'size', the cylinder will
     *              have a shape similar to a cone. This will be paticulary
     *              useful for our tree algorithm.
     *
     * This class can be used as is and provides a default implementation of
     * a wireframe that contains all vertices and edges, and all vertices are
     * of the same size.
     */
    class WireframeProcessing : public MeshProcessing {
        public:
            /* Constructor
             * @param filename : file name of the mesh to be processed.
             * @param sphere_filename : file containing the mesh that
             * will replace vertices.
             * @param cylinder_filename : file containing the mesh that
             * will replace edges.
             */
            WireframeProcessing(const std::string& filename,
                                const std::string& sphere_filename,
                                const std::string& cylinder_filename);
            
            /* Desctructor */
            ~WireframeProcessing(void);

            /* Create a wireframe after calling the fill_wireframe_properties
             * method.
             */
            void create_wire_frame(void);

            /* Create a 'default' wireframe in which all vertices and edges are in the
             * wireframe, with fixed size.
             * @param sphere_size : The diameter of the sphere.
             * @param cylinder_diameter : The diameter of the cylinders.
             */
            void create_wire_frame(const float sphere_size, const float cylinder_diameter);

        protected:
            /* Helper method for the create_wire_frame method.
             * @param custom : Tell if the fill_wireframe_properties method
             *  needs to be called before the transformation.
             * @param sphere_scale : Default sphere scale.
             * @parma cylinder_scale : Default cylinder scale.
             * Note : The last two args are ignored when custom = true.
             */
            void inner_create_wireframe(const bool custom, const surface_mesh::Vec3 sphere_scale, const std::pair<float, float> cylinder_scale);

            /* Replace vertices with spheres. */
            void replace_vertices();

            /* Replace edges with cylinders. */
            void replace_edges();

            /* Insert a given mesh into the current wireframe.
             * @param to_insert : the mesh that will be inserted.
             * @param pos : the position where to insert the mesh.
             * @param rot_axis : the axis of the rotation of the mesh.
             * @param rot_angle : the angle of the rotation.
             * @param scale : the scaling vector.
             * @param edge : tell wether we replace an edge or not. It is
             * useful as scales are handled differently with edges (see code
             * and report for more details).
             */
            void insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot_axis, float rot_angle, surface_mesh::Vec3 scale, bool edge);

            /* Virtual method that fills the wireframe properties of the
             * vertices and edges. (See report for details on these
             * properties).
             * @params : all the wireframe related properties.
             */
            virtual void fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<std::pair<float, float>> e_scale);

            Mesh sphere_;
            Mesh cylinder_;

            /* Will store the result of the transformation and
             then replace the mesh_.*/
            Mesh result_;
   };
}
