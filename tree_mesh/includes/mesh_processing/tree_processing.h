#pragma once

#include "mesh_processing/wireframe_processing.h"
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <tuple> 
#include <queue>

#define BETTER_ALGO 
#define USE_STACK

/* The limit in the number of trunk is set to 3 by default.
 * The color for each trunk is defined below.
 *
 * This limitation of 3 trunks makes the algo a lot easier.
 */
#define MAX_TRUNK_COUNT 3
#define TRUNK_RED 0
#define TRUNK_GREEN 1
#define TRUNK_BLUE 2


namespace mesh_processing {

    /* Element of step.
     * Contains the current vertex as well as the last one so that we are
     * able to compute the direction of the branch.
     */
    typedef struct {
        Mesh::Vertex root;
        Mesh::Vertex last;
    }branch_t;

    /*
     * class TreeProcessing : The one that implements the 'tree' algorithm.
     *
     * As the resulting tree can be seen as a partial wireframe, it is natural
     * to make this class a subclass of the WireframeProcessing class.
     *
     * This class simply redefine the fill_wireframe_properties method of its
     * super class. The resulting wireframe looks like a tree in which you 
     * can recognize the model.
     */
    class TreeProcessing : public WireframeProcessing {
        public:
            /* Constructor.
             * @param filename : file containing the mesh to be processed.
             * @param sphere_filename : file containing the mesh that
             * will replace vertices.
             * @param cylinder_filename : file containing the mesh that
             * will replace edges.
             * @param sphere_base_diameter : diameter of the spheres.
             * @param cylinder_base_diameter : diameter of the cylinders.
             */
            TreeProcessing(const std::string& filename,
                           const std::string& sphere_filename,
                           const std::string& cylinder_filename);

            /* Destructor. */
            ~TreeProcessing();
	    
            /* Export the mesh to the current directory.
             * @param filename : the name of the target file.
             */
            void export_mesh(const std::string& filename);

            /* Create the tree wireframe with given parameters.
             * @param sphere_base_diameter : the sphere diameter as seen on the lowest point of the trunk.
             * @param cylinder_base_diameter : the cylinder diameter as seen on the lowest point of the trunk. 
             * @param max_length : maximum length of a branch.
             * @param trunk_scale_multiplier : scale emplification of the trunk.
             * @param min_dot_between_branches : minimum dot for neighbor filtering, see report.
             * @param min_rel_len_before_split : minimum relative length needed to split a branch.
             * @param draw_trunk[] : Mask array to indicate which trunk needs to be grown.
             */ 
            void create_tree_wireframe(const float sphere_base_diameter,
                                       const float cylinder_base_diameter,
                                       const float max_length,
                                       const float trunk_scale_multiplier,
                                       const float min_dot_between_branches,
                                       const float min_rel_len_before_split,
                                       const bool draw_trunk[MAX_TRUNK_COUNT]);

        private:
            /* Main method of our project.
             * Virtual method that fills the wireframe properties of the
             * vertices and edges. (See report for details on these
             * properties).
             * @params : all the wireframe related properties.
             */
            virtual void fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<std::pair<float, float>> e_scale);

            /* Helper method to construct the tree.
             * @param to_process : queue containing the vertices to process.
             * Note : This method is not recursive as explained in the report.
             */
            void inner_fill(std::queue<branch_t> to_process);

            /* Returns the neighbors of a vertex in a vector.
             * @param v : the vertex.
             * @param not_in_wireframe : if true only the neighbors that have
             * their v_inwireframe property set to false will be returned.
             */
            std::vector<Mesh::Vertex> get_neighbors(Mesh::Vertex v, bool not_in_wireframe);

            /* Returns true iff the given vertex is contained in the main trunk
             * of the tree.
             */
            bool is_trunk(Mesh::Vertex v);

            /* Returns the trunk index of the given vertex. */
            int get_trunk_index(Mesh::Vertex v);

            /* Return the lowest trunk vertex. 
             * @param trunk_idx : the index of the trunk from which to retrieve
             *  the lowest vertex.
             */
            Mesh::Vertex get_lowest_trunk_vertex(const int trunk_idx);

            /* Randomly decides if a given vertex can split or not. */ 
            bool split(Mesh::Vertex v);
            
            /* Returns the two neighbors that form the best split as explained
             * in the report.
             * @param v : the vertex on which to perform the split.
             * @param neighors : the list of neighbors to choose from.
             */
            std::pair<Mesh::Vertex, Mesh::Vertex> get_best_split_pair(Mesh::Vertex v, std::vector<Mesh::Vertex> neighbors);

            /* Get the scaling factor for a given vertex. */
            float get_scale_factor(Mesh::Vertex);

            /* Some generation paramters. */
            /* Sphere and cylinder width as seen at the bottom of the trunk. */
            float sphere_base_diameter_ = 0.5f;
            float cylinder_base_diameter_ = 0.5f;
            /* Maximum lenth possible for a branch. */
            float max_length_ = 15.0f;
            /* Make the trunk a big bigger. */
            float trunk_scale_multiplier_ = 1.2f;
            /* Minimum dot between 2 branches. */
            float min_dot_between_branches_ = 0.0f;
            /* Min relative length before split. */
            float min_rel_len_before_split_ = 1.0f;
            /* The trunks of the tree. */
            std::vector<Mesh::Vertex> trunks_;

            /* All the wireframe related properties stored as members so that
             * we can easily access them in any helper method. */
            Mesh::Vertex_property<bool> v_inwireframe_;
            Mesh::Vertex_property<surface_mesh::Vec3> v_scale_;
            Mesh::Edge_property<bool> e_inwireframe_;
            Mesh::Edge_property<std::pair<float, float>> e_scale_;

            /* Some new properties. */

            /* Absolute length of a vertex, that is the length from the
             * lowest vertex of the trunk. */
            Mesh::Vertex_property<float> v_abs_length_;

            /* Relative length of a vertex, that is the lenght from the last
             * split. */
            Mesh::Vertex_property<float> v_rel_length_;

            /* true iff the vertex is in the trunk of the tree. */
            Mesh::Vertex_property<bool> v_trunk_;
            /* Indice of the trunk, -1 if not in the trunk. */
            Mesh::Vertex_property<int> v_trunk_index_;

            /* Multiple trunks : */
            const surface_mesh::Color trunk_colors[MAX_TRUNK_COUNT] = {
                surface_mesh::Color(1, 0, 0),
                surface_mesh::Color(0, 1, 0),
                surface_mesh::Color(0, 0, 1)
            };

            /* Keeps track of which trunks exist. */
            bool trunk_existing[MAX_TRUNK_COUNT] = {
                false,
                false,
                false
            };

            /* Which trunk will be grown during the wireframe creation. */
            bool draw_trunk_[MAX_TRUNK_COUNT];
    };
}
