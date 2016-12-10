#include "mesh_processing/tree_processing.h"

#define DEBUG(mesg) \
    std::cout << "DEBUG : " << mesg << std::endl

namespace mesh_processing {
    TreeProcessing::TreeProcessing(const std::string& filename,
            const std::string& sphere_filename,
            const std::string& cylinder_filename,
            const float sphere_base_diameter,
            const float cylinder_base_diameter) : WireframeProcessing(filename, sphere_filename, cylinder_filename) {
        sphere_base_diameter_ = sphere_base_diameter;
        cylinder_base_diameter_ = cylinder_base_diameter;
        //std::srand(std::time(0)); // use current time as seed for random generator
        std::srand(14567); // use current time as seed for random generator
    }

    TreeProcessing::~TreeProcessing() {}

    void TreeProcessing::export_mesh(const std::string& filename){
	    mesh_.write(filename);
    }

    std::vector<Mesh::Vertex> TreeProcessing::get_neighbors(Mesh::Vertex v, bool not_in_wireframe) {
	    Mesh::Vertex_around_vertex_circulator vc, vc_end;
	    vc = mesh_.vertices(v);
	    vc_end = vc;

	    std::vector<Mesh::Vertex> neighbors;
	    do {
		    Mesh::Vertex n = *vc;
		    Point n_pos = mesh_.position(n);
		    if (!not_in_wireframe || !v_inwireframe_[n]) {
			    neighbors.push_back(n);
		    } 
	    } while(++vc!=vc_end);
	    return neighbors;
    }

    bool TreeProcessing::is_root(Mesh::Vertex v) {
        Mesh::Vertex_property<surface_mesh::Color> v_color = mesh_.get_vertex_property<surface_mesh::Color>("v:color");
        surface_mesh::Color c = v_color[v];
        return c[1] <= 0.5f && c[2] <= 0.5f;
    }

    Mesh::Vertex TreeProcessing::get_lowest_root_vertex(void) {
        Mesh::Vertex curr;
        float curr_height = FLT_MAX;
        for (Mesh::Vertex v : mesh_.vertices()) {
            if (v_root_[v]) {
                float h = mesh_.position(v)[1];
                if (h < curr_height) {
                    curr_height = h;
                    curr = v;
                }
            }
        }
        return curr;
    }

    void TreeProcessing::fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale) {
        /* Set the private properties to avoid clutter. */
        v_inwireframe_ = v_inwireframe;
        v_scale_ = v_scale;
        e_inwireframe_ = e_inwireframe;
        e_scale_ = e_scale;
        v_root_ = mesh_.vertex_property<bool>("v:is_root", false);
        v_length_ = mesh_.vertex_property<float>("v:length", 0.0f);

        /* Fill is_root property */
        for (Mesh::Vertex v : mesh_.vertices()) {
            if (is_root(v)) {
                v_root_[v] = true;
            }
        }

        std::queue<branch_t> to_process;
        /* Initially the queue will contain only the lowest vertex of the
         * root. */
        Mesh::Vertex start = get_lowest_root_vertex();
        v_inwireframe_[start] = true;
        to_process.push({start, start});

        /* Start the algo. */
        inner_fill(to_process);
    }

#define keep(v) \
    do { \
    to_keep.push_back(v); \
    neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), v), neighbors.end()); \
    } while(0);

#define ignore(v) \
    do { \
    neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), v), neighbors.end()); \
    } while(0);

    void TreeProcessing::inner_fill(std::queue<branch_t> to_process) {
        const float max_length = 12.0f;
	    while (!to_process.empty()) {
            DEBUG("Queue size = " << to_process.size());
            branch_t current_branch = to_process.front();
            to_process.pop();

            Mesh::Vertex current_vertex = current_branch.root;
            Mesh::Vertex last_vertex = current_branch.last;
            Point current_vertex_pos = mesh_.position(current_branch.root);
            Point last_vertex_pos = mesh_.position(current_branch.last);

            const float current_length = v_length_[current_vertex];
            if (current_length >= max_length) {
                continue;
            }
            const float length_scale_factor = (1.0f - (current_length / max_length)) * (v_root_[current_vertex] ? 1.2f : 1.0f);
            v_scale_[current_vertex] = length_scale_factor * sphere_base_diameter_; // TODO

            std::vector<Mesh::Vertex> neighbors = get_neighbors(current_vertex, true);
            /* To keep will be the result of filtering the set of neighbors. */
            std::vector<Mesh::Vertex> to_keep;

            /* Deal with neighbors belonging to the root. */
            for (Mesh::Vertex n : neighbors) {
                if (v_root_[n]) {
                    if (v_root_[current_vertex]) {
                        /* We are allowed to continue our path on the root,
                         * iff we are on the root from the start. */
                        keep(n);
                    }
                    else {
                        /* Otherwise we are not allowed to process this
                         * neighbor, thus we need to ignore it.*/
                        ignore(n);
                    }
                }
            }

            /* The second condition is that the angle between the two branches
             * 'looks' realistic. */
            for (Mesh::Vertex n : neighbors) {
                Point last_dir = current_vertex_pos - last_vertex_pos;
                Point new_dir = mesh_.position(n) - current_vertex_pos;
                if (dot(last_dir, new_dir) >= 0.0f) {
                    keep(n);
                }
            }

            /* Now that the neighbors are filtered we can process recursively. */
            for (Mesh::Vertex n : to_keep) {
                Mesh::Edge e = mesh_.find_edge(current_vertex, n);
                v_length_[n] = v_length_[current_vertex] + mesh_.edge_length(e);
                e_inwireframe_[e] = true;
                e_scale_[e] = length_scale_factor * cylinder_base_diameter_; // TODO
                v_inwireframe_[n] = true;

                to_process.push({n, current_vertex});
            }
        }
    }
}
