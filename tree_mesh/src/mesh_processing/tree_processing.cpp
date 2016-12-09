#include "mesh_processing/tree_processing.h"

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

    std::vector<Mesh::Vertex> get_neighbors(Mesh mesh, Mesh::Vertex v, Mesh::Vertex_property<bool> v_inwireframe) {
	    Mesh::Vertex_around_vertex_circulator vc, vc_end;
	    vc = mesh.vertices(v);
	    vc_end = vc;

	    std::vector<Mesh::Vertex> neighbors;
	    do {
		    Mesh::Vertex n = *vc;
		    Point n_pos = mesh.position(n);
		    //if (n_pos[1] > mesh.position(v)[1] && !v_inwireframe[n]) {
		    if (!v_inwireframe[n]) {
			    neighbors.push_back(n);
		    } 
	    } while(++vc!=vc_end);
	    return neighbors;
    }

    static std::vector<Mesh::Vertex> get_n_lowest_vertices(Mesh &mesh, int n) {
	    std::vector<Mesh::Vertex> vertices;
	    for (Mesh::Vertex v : mesh.vertices()) {
		    vertices.push_back(v);
	    }
	    sort(vertices.begin(), vertices.end(), 
			    [&mesh](const Mesh::Vertex &a, const Mesh::Vertex &b) -> bool
			    { 
			    return mesh.position(a)[1] < mesh.position(b)[1];
			    });
	    if (!(0 <= n && n < vertices.size())) {
		    throw std::string("NOPE.");
	    }
	    else {
		    return std::vector<Mesh::Vertex>(vertices.begin(), vertices.begin()+n);
	    }
    }


    void TreeProcessing::fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale) {
	    std::queue<recursion_data_t> to_process;
        Mesh::Vertex_property<int> length = mesh_.vertex_property<int>("v:branch_length", 0); 

        // We trace the root
        build_main_root(v_inwireframe, v_scale, e_inwireframe, e_scale, length);

        int i = 0;
	    for (Mesh::Vertex root : roots_) {
            length[root] = 1;
            if (++i%50 == 0)
                to_process.push({root, root});
	    }
	    inner_fill(to_process, v_inwireframe, v_scale, e_inwireframe, e_scale, length);
    }

    static bool split(float relative_height) {
        float lambda = 2.0f;
        const float upper_limit_threshold = 1.0f;
        if (relative_height > upper_limit_threshold) {
            return (float)std::rand() / RAND_MAX < 1.0f - relative_height;
        }
        else {
            return (float)std::rand() / RAND_MAX < std::pow(relative_height, lambda);
        }
    }


    void TreeProcessing::build_main_root(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length) {
        Mesh::Vertex_property<Color> v_color = mesh_.get_vertex_property<Color>("v:color");
        Mesh::Vertex_property<bool> v_root = mesh_.vertex_property<bool>("v:root");

        for (Mesh::Vertex v : mesh_.vertices()) {
            if (v_color[v][1] < 0.5 && v_color[v][2] < 0.5) {
                roots_.push_back(v);
                v_root[v] = true;
            }
        } 

        Mesh::Vertex start;
        float low = FLT_MAX;
        for (Mesh::Vertex v : roots_) {
            if (low > mesh_.position(v)[1]) {
                low = mesh_.position(v)[1];
                start = v;
            } 
        }

        v_length[start] = 0;
        build_main_root_inner(start, v_root, v_inwireframe, v_scale, e_inwireframe, e_scale, v_length);
        std::cout << "Max len = " << max_root_length << std::endl;
    }

    void TreeProcessing::build_main_root_inner(Mesh::Vertex root, Mesh::Vertex_property<bool> v_root, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length) {
        v_inwireframe[root] = true;
        const int max_root_len = 117;
        int len = v_length[root];
        float scale_factor = 1.0f - ((float)len / max_root_len);
        scale_factor = std::pow(scale_factor, 2);
        v_scale[root] = scale_factor * sphere_base_diameter_;

        std::vector<Mesh::Vertex> neighbors = get_neighbors(mesh_, root, v_inwireframe);
        std::vector<Mesh::Vertex> root_nei(neighbors.size());
        
        auto it = std::copy_if (neighbors.begin(), neighbors.end(), root_nei.begin(),
                [this, &v_root](Mesh::Vertex &v) {
                    return v_root[v];
                });
        root_nei.resize(std::distance(root_nei.begin(),it));

        if (root_nei.size()) {
            if (v_length[root] > max_root_length) {
                max_root_length = v_length[root];
            }
        }

        for (Mesh::Vertex n : root_nei) {
            Mesh::Edge e = mesh_.find_edge(root, n);
            e_inwireframe[e] = true;
            e_scale[e] = scale_factor * cylinder_base_diameter_;
            v_length[n] = v_length[root] + 1;
            build_main_root_inner(n, v_root, v_inwireframe, v_scale, e_inwireframe, e_scale, v_length);
        }
    }

    void TreeProcessing::inner_fill(std::queue<recursion_data_t> to_process, Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<surface_mesh::Vec3> e_scale, Mesh::Vertex_property<int> v_length) {
        Mesh::Vertex_property<bool> v_root = mesh_.vertex_property<bool>("v:root");
	    while (!to_process.empty()) {
            recursion_data_t first = to_process.front();
		    Mesh::Vertex root = first.root;
            Mesh::Vertex last = first.last;
		    to_process.pop();
            float low = mesh_.position(get_lowest_point(mesh_))[1];
            float high = mesh_.position(get_highest_point(mesh_))[1];
            float relative = (mesh_.position(root)[1] - low) / (high - low);
            float len; // = v_length[root];
            if (v_root[root]) {
                len = 1.0f;
            }
            else {
                len = v_length[root];
            }
            const int max_branch_length = 16;

            float scale_factor = 1.0f - ((float)len / max_branch_length);
            scale_factor = std::pow(scale_factor, 2);

            float root_scale = scale_factor * 0.1f * sphere_base_diameter_;

            v_scale[root] = surface_mesh::Vec3(root_scale, root_scale, root_scale);
            Point root_pos = mesh_.position(root);


            std::vector<Mesh::Vertex> neighbors = get_neighbors(mesh_, root, v_inwireframe);
            std::vector<Mesh::Vertex> new_nei(neighbors.size());
            
            auto it = std::copy_if (neighbors.begin(), neighbors.end(), new_nei.begin(),
                    [this, root, last](Mesh::Vertex &v) {
                        Point proot = this->mesh_.position(root);
                        Point pv = this->mesh_.position(v);
                        Point plast = this->mesh_.position(last);
                        if (last == root) {
                            return true;
                        }
                        else {
                            return dot(pv-proot, proot-plast) >= 0.0f;
                        }
                    });
            new_nei.resize(std::distance(new_nei.begin(),it));
            neighbors = new_nei;

            if (neighbors.size() == 0) {
                continue;
            }

            int n = 1;
            if (split(relative)) {
                n = std::min(2, (int)neighbors.size());
            } 
            if (len < max_branch_length) {
                std::vector<Mesh::Vertex> next(neighbors.begin(), neighbors.begin()+n);
                for (Mesh::Vertex v : next) {
                    v_inwireframe[v] = true;
                }
                for (Mesh::Vertex v : next) {
                    Point v_pos = mesh_.position(v);
                    v_inwireframe[v] = true;
                    v_length[v] = len + 1;
                    Mesh::Edge e = mesh_.find_edge(root, v);
                    e_inwireframe[e] = true;
                    float mean_edge_height = ((root_pos + v_pos) / 2.0f)[1];
                    mean_edge_height = (mean_edge_height - low) / (high - low);
                    mean_edge_height = gaussian(mean_edge_height);
                    float edge_scale = scale_factor * 0.1f * cylinder_base_diameter_;
                    edge_scale = std::max(edge_scale, 0.02f);
                    e_scale[e] = surface_mesh::Vec3(edge_scale, 1.0f, edge_scale);
                    to_process.push({v, root});
                }
            }
        } // for while loop
    }
}
