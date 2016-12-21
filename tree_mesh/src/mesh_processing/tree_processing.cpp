#include "mesh_processing/tree_processing.h"

#define DEBUG(mesg) \
    std::cout << "DEBUG : " << mesg << std::endl

namespace mesh_processing {
    TreeProcessing::TreeProcessing(const std::string& filename,
            const std::string& sphere_filename,
            const std::string& cylinder_filename) : WireframeProcessing(filename, sphere_filename, cylinder_filename) {
        std::srand(14567);
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

    bool TreeProcessing::is_trunk(Mesh::Vertex v) {
        return get_trunk_index(v) != -1;
    }

    int TreeProcessing::get_trunk_index(Mesh::Vertex v) {
        Mesh::Vertex_property<surface_mesh::Color> v_color = mesh_.get_vertex_property<surface_mesh::Color>("v:color");
        for (int i = 0; i < MAX_TRUNK_COUNT; ++i) {
            if (v_color[v] == trunk_colors[i]) {
                return i;
            }
        }
        return -1;
    }

    Mesh::Vertex TreeProcessing::get_lowest_trunk_vertex(const int trunk_idx) {
        Mesh::Vertex curr;
        float curr_height = FLT_MAX;
        for (Mesh::Vertex v : mesh_.vertices()) {
            if (v_trunk_[v] && v_trunk_index_[v] == trunk_idx) {
                float h = mesh_.position(v)[1];
                if (h < curr_height) {
                    curr_height = h;
                    curr = v;
                }
            }
        }
        return curr;
    }

    void TreeProcessing::create_tree_wireframe(const float sphere_base_diameter,
                                               const float cylinder_base_diameter,
                                               const float max_length,
                                               const float trunk_scale_multiplier,
                                               const float min_dot_between_branches,
                                               const float min_rel_len_before_split) {
        sphere_base_diameter_     = sphere_base_diameter;
        cylinder_base_diameter_   = cylinder_base_diameter;
        max_length_               = max_length;
        trunk_scale_multiplier_    = trunk_scale_multiplier;
        min_dot_between_branches_ = min_dot_between_branches;
        min_rel_len_before_split_ = min_rel_len_before_split;
        /* create_wire_frame will call the fill_wireframe_properties */
        create_wire_frame();
    }

    void TreeProcessing::fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<std::pair<float, float>> e_scale) {
        /* THIS IS WHERE THE MAGIC HAPPENS ! */

        /* Reset properties from previous runs. */
        remove_edge_property_with_name<bool>(mesh_, "v:is_trunk");
        remove_edge_property_with_name<float>(mesh_, "v:v_abslength");
        remove_edge_property_with_name<float>(mesh_, "v:v_rellength");

        /* Set the private properties to avoid clutter. */
        v_inwireframe_ = v_inwireframe;
        v_scale_ = v_scale;
        e_inwireframe_ = e_inwireframe;
        e_scale_ = e_scale;
        v_trunk_ = mesh_.vertex_property<bool>("v:is_trunk", false);
        v_trunk_index_ = mesh_.vertex_property<int>("v:trunk_index", -1);
        v_abs_length_ = mesh_.vertex_property<float>("v:v_abslength", 0.0f);
        v_rel_length_ = mesh_.vertex_property<float>("v:v_rellength", 0.0f);

        /* Fill is_trunk property */
        for (Mesh::Vertex v : mesh_.vertices()) {
            if (is_trunk(v)) {
                v_trunk_[v] = true;
                int idx = get_trunk_index(v);
                v_trunk_index_[v] = idx; // TODO : Not optimal
                if (idx != -1) {
                    trunk_existing[idx] = true;
                    DEBUG("Found trunk with id " << idx << " and color " << trunk_colors[idx]);
                }
                else {
                    throw "Trunk as invalid index.";
                }
            }
        }

        std::queue<branch_t> to_process;
        /* Initially the queue will contain only the lowest vertex of the
         * trunk from there the inner method does all the work. */
        Mesh::Vertex starting_vertices[MAX_TRUNK_COUNT];
        for (int i = 0; i < MAX_TRUNK_COUNT; ++i) {
            if (trunk_existing[i]) {
                starting_vertices[i] = get_lowest_trunk_vertex(i);
                v_inwireframe_[starting_vertices[i]] = true;
                to_process.push({starting_vertices[i], starting_vertices[i]});
            }
        }

        if (to_process.empty()) {
            throw "No trunk found !";
        }

        /* Start the algo. */
        inner_fill(to_process);
    }

    bool TreeProcessing::split(Mesh::Vertex v) {
        /* The idea is to have a lot of splits on top and none at the bottom. */
        const float p = (float)std::rand() / RAND_MAX;
        return max_length_ * p < v_abs_length_[v];
    }

    std::pair<Mesh::Vertex, Mesh::Vertex> TreeProcessing::get_best_split_pair(Mesh::Vertex v, std::vector<Mesh::Vertex> neighbors) {
        /* As explain in the report, we aim to find the 2 neighbors that split
         * the most, ie the angle between the 2 is maximized. */
        float curr_angle = FLT_MIN;
        Mesh::Vertex curr_1;
        Mesh::Vertex curr_2;
        Point pos_v = mesh_.position(v);
        for (Mesh::Vertex v1 : neighbors) {
            for (Mesh::Vertex v2 : neighbors) {
                Point pos_1 = normalize(mesh_.position(v1)-pos_v);
                Point pos_2 = normalize(mesh_.position(v2)-pos_v);
                float d = dot(pos_1, pos_2);
                float angle = std::acos(d);
                if (angle >= curr_angle) {
                    curr_angle = angle;
                    curr_1 = v1;
                    curr_2 = v2;
                }
            }
        }
        return std::pair<Mesh::Vertex, Mesh::Vertex>(curr_1, curr_2);
    }

    float TreeProcessing::get_scale_factor(Mesh::Vertex v) {
        /* Note that the trunk is a little bit bigger so that we easily
         * see the important features of the face. */
        return (1.0f - (v_abs_length_[v] / max_length_)) * (v_trunk_[v] ? trunk_scale_multiplier_ : 1.0f);
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
        /* I lied above, the real magic happens here ;) */
        int total_splits_performed = 0;
        while (!to_process.empty()) {
            branch_t current_branch = to_process.front();
            to_process.pop();

            Mesh::Vertex current_vertex = current_branch.root;
            Mesh::Vertex last_vertex = current_branch.last;
            Point current_vertex_pos = mesh_.position(current_branch.root);
            Point last_vertex_pos = mesh_.position(current_branch.last);

            const float current_length = v_abs_length_[current_vertex];
            const float length_scale_factor = get_scale_factor(current_vertex);

            /* We set the scales for the sphere. The edges come after. */
            v_scale_[current_vertex] = length_scale_factor * sphere_base_diameter_;

            if (current_length >= max_length_) {
                /* Termination of the branch. */
                continue;
            }

            /* We retrieve the neighbors. */
            std::vector<Mesh::Vertex> neighbors = get_neighbors(current_vertex, true);
            std::vector<Mesh::Vertex> all_neighbors = get_neighbors(current_vertex, false);

            /* The current vertex is considered close to a branch is more than
             * 2 of its neighors are already in the tree wireframe. This boolean
             * avoid having branches that follows along others, giving questionabl
             * results. */
            const bool close_to_branch = all_neighbors.size() - neighbors.size() >= 3;

            /* We copy the vector so that we can perform deletion while iterating
             * over the copy. */
            std::vector<Mesh::Vertex> neighbors_cpy = neighbors;

            /* To keep will be the result of filtering the set of neighbors. */
            std::vector<Mesh::Vertex> to_keep;

            /* Deal with neighbors belonging to the trunk. Those ones are
             * always kept but only if the current vertex is also in the 
             * trunk. */
            for (Mesh::Vertex n : neighbors_cpy) {
                if (v_trunk_[n]) {
                    if (v_trunk_[current_vertex] && v_trunk_index_[current_vertex] == v_trunk_index_[n]) {
                        /* We are allowed to continue our path on the trunk,
                         * iff we are on the trunk from the start. */
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

            /* We now renove all neighbors that would yield to non realistic 
             * splits ie. with absolute value angle > PI/2 or dot product < 0. */
            for (Mesh::Vertex n : neighbors_cpy) {
                Point last_dir = normalize(current_vertex_pos - last_vertex_pos);
                Point new_dir = normalize(mesh_.position(n) - current_vertex_pos);
                if (!(dot(last_dir, new_dir) >= min_dot_between_branches_)) {
                    ignore(n);
                }
            }
            neighbors_cpy = neighbors; /* Not necessary, but here to avoid forgetting it. */

            if (neighbors.size() == 0) {
                /* If no neighbor is kept, then we stop, and goes with the next
                 * entry in the queue. */
                continue;
            }

            /* We don't allow split with more than 2 branches. */ 
            if (to_keep.size() < 2) {
                /* If we kept less than 2 neighbors (trunk in this case) we
                 * consider splitting. */

                /* We impse a condition on splitting, the last split must have
                 * occured at a distance of at least a certain threshold. */
                const bool split_condition = (v_rel_length_[current_vertex] >= min_rel_len_before_split_) && split(current_vertex);
                int split_count = 1 - to_keep.size(); /* Number of neigbors to take when splitting. 1 means no split. */
                if (split_condition) {
                    /* A split occurs at the current vertex. */
                    split_count ++;
                }

                if (to_keep.size() <= split_count && !close_to_branch) {
                    split_count = std::min(split_count, (int)neighbors.size());

                    if (split_count == 1) {
                        /* Case 1 : No split occurs. */
                        Mesh::Vertex next;
                        if (to_keep.size() == 0) {
                            /* If we are not following a trunk, we try to follow 
                             * the direction of the current branch as much as 
                             * possible, thus we would like to take the next 
                             * neighbor that minimizes the change of direction.
                             */
                            std::sort(neighbors.begin(), neighbors.end(), 
                                [this, &current_vertex_pos, &last_vertex_pos](Mesh::Vertex &a, Mesh::Vertex &b) {
                                    Point a_pos = this->mesh_.position(a);
                                    Point b_pos = this->mesh_.position(b);
                                    float a_dot = dot(normalize(a_pos - current_vertex_pos), normalize(current_vertex_pos - last_vertex_pos));
                                    float b_dot = dot(normalize(b_pos - current_vertex_pos), normalize(current_vertex_pos - last_vertex_pos));
                                    return a_dot > b_dot;
                                }
                            );
                            next = neighbors[0];
                        }
                        else {
                            /* Here, we are in the situation where :
                             * 1) We were forced to follow a trunk
                             * 2) The trunk does no split (otherwise to_keep
                             *      would be of size 2.
                             * 3) We want to split 'naturally'
                             * The idea is to take the neighbor that goes as far
                             * as possible from the trunk. ie the angle between the
                             * neighbor and the trunk is maximized. */
                            Mesh::Vertex winner;
                            float best_angle = FLT_MIN;
                            for (Mesh::Vertex v : neighbors) {
                                Point pos_v = normalize(mesh_.position(v)-current_vertex_pos);
                                Point pos_trunk = normalize(mesh_.position(to_keep[0])-current_vertex_pos);
                                float d = dot(pos_v, pos_trunk);
                                float angle = std::acos(d);
                                if (angle > best_angle) {
                                    winner = v;
                                    best_angle = angle;
                                }
                            }
                            next = winner;
                        }
                        to_keep.push_back(next);
                    }
                    else {
                        /* Case 2 : A split occurs. In this case we take the
                         * 2 neighbors that 'split the most' ie that make 
                         * branches that have teh maximum angle possible
                         * between them.
                         */
                        auto best_pair = get_best_split_pair(current_vertex, neighbors);
                        to_keep.push_back(std::get<0>(best_pair));
                        to_keep.push_back(std::get<1>(best_pair));
                    }
                }
            }

            const bool split_occured = (to_keep.size() > 1);

            /* Now that the neighbors are filtered and splits performed, we
             * can 'recurse'. */
            for (Mesh::Vertex n : to_keep) {
                Mesh::Edge e = mesh_.find_edge(current_vertex, n);
                const float edge_len = mesh_.edge_length(e);

                /* We update the length of the neighbor. */
                v_abs_length_[n] = v_abs_length_[current_vertex] + edge_len;
                
                if (split_occured) {
                    /* In case of split the relative length of the neighbor
                     * is reseted. */
                    total_splits_performed ++;
                    v_rel_length_[n] = edge_len;
                }
                else {
                    /* Otherwise it is just updated with the edge length. */
                    v_rel_length_[n] = v_rel_length_[current_vertex] + edge_len;
                }

                /* Finally we set the edge's properties. */
                e_inwireframe_[e] = true;

                /* It may happen that the edge is stored in the opposite
                 * direction, in this case we need to invert the e_scale
                 * pair.
                 */
                if (mesh_.vertex(e, 0) != current_vertex) {
                    e_scale_[e] = std::make_pair(length_scale_factor * cylinder_base_diameter_, get_scale_factor(n) * cylinder_base_diameter_);
                }
                else {
                    e_scale_[e] = std::make_pair(get_scale_factor(n) * cylinder_base_diameter_, length_scale_factor * cylinder_base_diameter_);
                }
                v_inwireframe_[n] = true;

                /* We push the neighbor into the queue. */
                to_process.push({n, current_vertex});
            }
        }
        DEBUG(total_splits_performed << " splits performed.");
    }
}
