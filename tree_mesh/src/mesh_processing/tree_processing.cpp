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
        const float th = 0.2f;
        return c[1] <= th && c[2] <= th;
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
        v_abs_length_ = mesh_.vertex_property<float>("v:v_abslength", 0.0f);
        v_rel_length_ = mesh_.vertex_property<float>("v:v_rellength", 0.0f);

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

    bool TreeProcessing::split(Mesh::Vertex v) {
        const float p = (float)std::rand() / RAND_MAX;
        return max_length_ * p < v_abs_length_[v];
    }

    std::pair<Mesh::Vertex, Mesh::Vertex> TreeProcessing::get_best_split_pair(Mesh::Vertex v, std::vector<Mesh::Vertex> neighbors) {
        float curr_angle = FLT_MIN;
        Mesh::Vertex curr_1;
        Mesh::Vertex curr_2;
        for (Mesh::Vertex v1 : neighbors) {
            for (Mesh::Vertex v2 : neighbors) {
                Point pos_1 = normalize(this->mesh_.position(v1));
                Point pos_2 = normalize(this->mesh_.position(v2));
                float d = dot(pos_1, pos_2);
                DEBUG("d = " << d);
                float angle = d;
                DEBUG("angle = " << angle);
                if (angle >= curr_angle) {
                    curr_angle = angle;
                    curr_1 = v1;
                    curr_2 = v2;
                }
            }
        }
        return std::pair<Mesh::Vertex, Mesh::Vertex>(curr_1, curr_2);
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
        int total_splits_performed = 0;
        while (!to_process.empty()) {
            branch_t current_branch = to_process.front();
            to_process.pop();

            Mesh::Vertex current_vertex = current_branch.root;
            Mesh::Vertex last_vertex = current_branch.last;
            Point current_vertex_pos = mesh_.position(current_branch.root);
            Point last_vertex_pos = mesh_.position(current_branch.last);

            const float current_length = v_abs_length_[current_vertex];
            if (current_length >= max_length_) {
                continue;
            }
            const float length_scale_factor = (1.0f - (current_length / max_length_)) * (v_root_[current_vertex] ? 1.2f : 1.0f);
            v_scale_[current_vertex] = length_scale_factor * sphere_base_diameter_; // TODO

            std::vector<Mesh::Vertex> neighbors = get_neighbors(current_vertex, true);
            std::vector<Mesh::Vertex> all_neighbors = get_neighbors(current_vertex, false);
            const bool close_to_branch = all_neighbors.size() - neighbors.size() >= 3;

            std::vector<Mesh::Vertex> neighbors_cpy = neighbors;
            /* To keep will be the result of filtering the set of neighbors. */
            std::vector<Mesh::Vertex> to_keep;

            /* Deal with neighbors belonging to the root. */
            for (Mesh::Vertex n : neighbors_cpy) {
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
            neighbors_cpy = neighbors;

            /* The second condition to keep a neighbor is that the angle 
             * between the two branches 'looks' realistic. */
            for (Mesh::Vertex n : neighbors_cpy) {
                Point last_dir = normalize(current_vertex_pos - last_vertex_pos);
                Point new_dir = normalize(mesh_.position(n) - current_vertex_pos);
                if (!(dot(last_dir, new_dir) >= 0.0f)) {
                    ignore(n);
                }
            }
            neighbors_cpy = neighbors; /* Not necessary, but here to avoid forgetting it. */

            if (neighbors.size() == 0) {
                continue;
            }

            if (to_keep.size() < 2) {
                /* Now we decide weather we split or not. */
                const bool split_condition = (v_rel_length_[current_vertex] >= 1.5f) && split(current_vertex);
                int split_count = 1 - to_keep.size(); /* Number of neigbors to take when splitting. 1 means no split. */
                if (split_condition) {
                    split_count ++;
                }

                /* Now we randomly take 'split_count' neighbors. */
                if (to_keep.size() <= split_count && !close_to_branch) {
                    split_count = std::min(split_count, (int)neighbors.size());

                    if (split_count == 1) {
                        /* We try to follow the direction of the current branch as
                         * much as possible, thus we would like to take the next
                         * neighbor that minimizes the change of direction. */
                        std::sort(neighbors.begin(), neighbors.end(), 
                            [this, &current_vertex_pos, &last_vertex_pos](Mesh::Vertex &a, Mesh::Vertex &b) {
                                Point a_pos = this->mesh_.position(a);
                                Point b_pos = this->mesh_.position(b);
                                float a_dot = dot(normalize(a_pos - current_vertex_pos), normalize(current_vertex_pos - last_vertex_pos));
                                float b_dot = dot(normalize(b_pos - current_vertex_pos), normalize(current_vertex_pos - last_vertex_pos));
                                return a_dot > b_dot;
                            }
                        );
                        Mesh::Vertex next = neighbors[0];
                        to_keep.push_back(next);
                    }
                    else {
                        /* Split == 2. */
                        auto best_pair = get_best_split_pair(current_vertex, neighbors);
                        to_keep.push_back(std::get<0>(best_pair));
                        to_keep.push_back(std::get<1>(best_pair));
                    }
                }
            }

            const bool split_occured = (to_keep.size() > 1);
            /* Now that the neighbors are filtered we can process recursively. */
            for (Mesh::Vertex n : to_keep) {
                Mesh::Edge e = mesh_.find_edge(current_vertex, n);
                const float edge_len = mesh_.edge_length(e);
                v_abs_length_[n] = v_abs_length_[current_vertex] + edge_len;
                if (split_occured) {
                    total_splits_performed ++;
                    v_rel_length_[n] = edge_len;
                }
                else {
                    v_rel_length_[n] = v_rel_length_[current_vertex] + edge_len;
                }
                e_inwireframe_[e] = true;
                e_scale_[e] = length_scale_factor * cylinder_base_diameter_;
                v_inwireframe_[n] = true;

                to_process.push({n, current_vertex});
            }
        }
        DEBUG(total_splits_performed << " splits performed.");
    }
}
