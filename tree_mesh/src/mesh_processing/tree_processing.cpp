#include "mesh_processing/tree_processing.h"

namespace mesh_processing {
    TreeProcessing::TreeProcessing(const std::string& filename,
            const std::string& sphere_filename,
            const std::string& cylinder_filename,
            const float sphere_base_diameter,
            const float cylinder_base_diameter) : WireframeProcessing(filename, sphere_filename, cylinder_filename) {
        sphere_base_diameter_ = sphere_base_diameter;
        cylinder_base_diameter_ = cylinder_base_diameter;
        std::srand(std::time(0)); // use current time as seed for random generator
    }

    TreeProcessing::~TreeProcessing() {}

    void TreeProcessing::fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale) {
        Mesh::Vertex lowest_vertex = get_lowest_point(mesh_);
        inner_fill(lowest_vertex, v_inwireframe, v_scale, e_inwireframe, e_scale);
    }

    static std::vector<Mesh::Vertex> take_n_first_higher_neighbors(std::vector<Mesh::Vertex> neighbors, int n) {
        if (n > neighbors.size()) {
            return std::vector<Mesh::Vertex> ();
        }
        std::vector<Mesh::Vertex> new_vec(neighbors.begin(), neighbors.begin()+n);
        return new_vec;
    }

    void TreeProcessing::inner_fill(Mesh::Vertex root, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale) {
        v_inwireframe[root] = true;
        Point root_pos = mesh_.position(root);
        v_scale[root] = sphere_base_diameter_;
        Mesh::Vertex_around_vertex_circulator vc, vc_end;
        vc = mesh_.vertices(root);
        vc_end = vc;

        /* We take all neighbors that are higher than 'root'. */
        std::vector<Mesh::Vertex> upper_neighbors;
        do {
            Mesh::Vertex candidate = *vc;
            Point candidate_pos = mesh_.position(candidate);
            if (!v_inwireframe[candidate]) {
               upper_neighbors.push_back(candidate);
            } 
        } while(++vc!=vc_end);

        int n = 0;
        if (upper_neighbors.size() > 0) {
            n = (2 + std::rand()) % upper_neighbors.size();
            if (n == 0) {
                n = upper_neighbors.size();
            }
        }
#ifdef SORT_NEIGHBORS
        sort(upper_neighbors.begin(), upper_neighbors.end(), 
                    [this](const Mesh::Vertex &a, const Mesh::Vertex &b) -> bool
                    { 
                        return this->mesh_.position(a)[1] < this->mesh_.position(b)[1];
                    });
#endif

        std::cout << n << " neighbors, from " << upper_neighbors.size() << " will be chosen." << std::endl;
        std::vector<Mesh::Vertex> next_roots = take_n_first_higher_neighbors(upper_neighbors, n);
        std::cout << next_roots.size() << std::endl;

        for (Mesh::Vertex r : next_roots) {
            v_inwireframe[r] = true;
        }

        for (Mesh::Vertex r : next_roots) {
            /* TODO : Make it converge faster !!!!! */
            Mesh::Edge e = mesh_.find_edge(root, r);
            e_inwireframe[e] = true;
            float low = mesh_.position(get_lowest_point(mesh_))[1];
            float high = mesh_.position(get_highest_point(mesh_))[1];
            float mean = ((mesh_.position(r) + mesh_.position(root))/2.0f)[1];
            float tmp = cylinder_base_diameter_ * (1.0f - (mean - low) / (high - low));
            e_scale[e] = surface_mesh::Vec3(tmp, 1, tmp);
            inner_fill(r, v_inwireframe, v_scale, e_inwireframe, e_scale);
        }
    }
}
