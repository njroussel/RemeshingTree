//=============================================================================
//
//   Code framework for the lecture
//
//   "Digital 3D Geometry Processing"
//
//   Gaspard Zoss
//
//   Copyright (C) 2016 by Computer Graphics and Geometry Laboratory,
//         EPF Lausanne
//
//-----------------------------------------------------------------------------
#include "mesh_processing/mesh_processing.h"
#include <set>
#include <cfloat>

namespace mesh_processing {

    using namespace surface_mesh;

    MeshProcessing::MeshProcessing(const string &filename) {
        load_mesh(filename);
    }

    MeshProcessing::~MeshProcessing() {
        // TODO
    }

    const surface_mesh::Point MeshProcessing::get_mesh_center() {
        return mesh_center_;
    }

    const float MeshProcessing::get_dist_max() {
        return dist_max_;
    }

    const Eigen::MatrixXf* MeshProcessing::get_points() {
        return &points_;
    }

    const MatrixXu* MeshProcessing::get_indices() {
        return &indices_;
    }

    const Eigen::MatrixXf* MeshProcessing::get_normals() {
        return &normals_;
    }

    const Eigen::MatrixXf* MeshProcessing::get_colors_valence() {
        return &color_valence_;
    }

    const Eigen::MatrixXf* MeshProcessing::get_colors_unicurvature() {
        return &color_unicurvature_;
    }

    const Eigen::MatrixXf* MeshProcessing::get_colors_gaussian_curv() {
        return &color_gaussian_curv_;
    }

    const Eigen::MatrixXf* MeshProcessing::get_color_curvature() {
        return &color_curvature_;
    }

    const unsigned int MeshProcessing::get_number_of_face() {
        return mesh_.n_faces();
    }

    const Mesh& MeshProcessing::mesh() {
        return mesh_;
    }

    void MeshProcessing::swap(Mesh& mesh) {
        mesh_ = mesh;
    }

    Vec3 MeshProcessing::laplacian_operator(Surface_mesh::Vertex x) {
        Surface_mesh::Vertex_around_vertex_circulator vc, vc_end;
        vc = mesh_.vertices(x);
        vc_end = vc;
        Vec3 laplacian = Vec3(0, 0, 0);
        int N = 0;

        Point x_position = mesh_.position(x);

        do {
            N++;
            Surface_mesh::Vertex v = *vc;
            laplacian += (mesh_.position(v) - x_position);
        } while (++vc != vc_end);
        return laplacian / N;
    }

    Vec3 MeshProcessing::laplace_beltrami_operator(Surface_mesh::Vertex x, bool normalize) {
        Mesh::Edge_property <Scalar> e_weight = mesh_.edge_property<Scalar>("e:weight", 0);
        Mesh::Vertex_property <Scalar> v_weight = mesh_.vertex_property<Scalar>("v:weight", 0);

        Surface_mesh::Vertex_around_vertex_circulator vc, vc_end;
        vc = mesh_.vertices(x);
        vc_end = vc;

        Vec3 laplace_beltrami = Vec3(0, 0, 0);
        float weight_sum = 0.0f;

        do {
            Surface_mesh::Edge edge = mesh_.find_edge(*vc, x);
            if (edge == Surface_mesh::Edge()) {
                throw std::string("Should not happen !");
            }

            float weight = e_weight[edge];
            weight_sum += weight;
            laplace_beltrami += weight * (mesh_.position(*vc) - mesh_.position(x));
        } while (++vc != vc_end);

        if (normalize) {
            return laplace_beltrami / weight_sum;
        } else {
            return laplace_beltrami * v_weight[x];
        }
    }


    void MeshProcessing::calc_uniform_mean_curvature() {
        Mesh::Vertex_property <Scalar> v_unicurvature =
            mesh_.vertex_property<Scalar>("v:unicurvature", 0.0f);
        for (Mesh::Vertex x : mesh_.vertices()) {
            if (!mesh_.is_boundary(x)) {
                Vec3 laplacian = laplacian_operator(x);
                /* The curvature is approximated as the half length of the laplacian operator for x. */
                v_unicurvature[x] = norm(laplacian) * 0.5f;
            }
        }
    }

    void MeshProcessing::calc_mean_curvature() {
        Mesh::Vertex_property <Scalar> v_curvature =
            mesh_.vertex_property<Scalar>("v:curvature", 0.0f);
        for (Mesh::Vertex x : mesh_.vertices()) {
            if (!mesh_.is_boundary(x)) {
                Vec3 laplace_beltrami = laplace_beltrami_operator(x, false);
                /* The curvature is approximated as the half length of the laplace beltrami operator for x. */
                v_curvature[x] = norm(laplace_beltrami) * 0.5f;
            }
        }
    }

    void MeshProcessing::calc_gauss_curvature() {
        Mesh::Vertex_property <Scalar> v_gauss_curvature =
            mesh_.vertex_property<Scalar>("v:gauss_curvature", 0.0f);
        Mesh::Vertex_property <Scalar> v_weight =
            mesh_.vertex_property<Scalar>("v:weight", 0.0f);

        /* For each vertex we compute the sum of the angles. */
        for (Mesh::Vertex x : mesh_.vertices()) {
            if (!mesh_.is_boundary(x)) {
                Mesh::Face_around_vertex_circulator fc, fc_end;
                fc = mesh_.faces(x);
                fc_end = fc;

                double angle_sum = 0.0;
                do {
                    Mesh::Vertex_around_face_circulator vc, vc_end;

                    vc = mesh_.vertices(*fc);
                    vc_end = vc;

                    std::vector<Mesh::Vertex> T;
                    do {
                        T.push_back(*vc);
                    } while (++vc != vc_end);
                    assert(T.size() == 3);

                    Point xi = mesh_.position(T[0]);
                    Point xj = mesh_.position(T[1]);
                    Point xk = mesh_.position(T[2]);

                    float dot_product;
                    Point x_pos = mesh_.position(x);
                    if (x_pos == xi)
                        dot_product = dot(normalize(xi - xj), normalize(xi - xk));
                    else if (x_pos == xj)
                        dot_product = dot(normalize(xj - xi), normalize(xj - xk));
                    else if (x_pos == xk)
                        dot_product = dot(normalize(xk - xi), normalize(xk - xj));

                    if (dot_product >= 1.0) {
                        dot_product = 1.0f;
                    } else if (dot_product <= -1.0) {
                        dot_product = -1.0f;
                    }

                    float angle = acos(dot_product);

                    angle_sum += angle;
                } while (++fc != fc_end);

                /* Approximate the curvature using the angle sum. */
                v_gauss_curvature[x] = (2 * M_PI - angle_sum) * 2.0f * v_weight[x];
            }
        }
    }

    void MeshProcessing::remesh(const REMESHING_TYPE &remeshing_type,
            const int &num_iterations) {
        calc_weights();
        calc_mean_curvature();
        calc_uniform_mean_curvature();
        calc_gauss_curvature();
        calc_target_length(remeshing_type);

        // main remeshing loop
        for (int i = 0; i < num_iterations; ++i) {
            std::cout << 100.0f * (float) i / num_iterations << " % completed." << std::endl;
            split_long_edges();
            cout << "split" << endl;
            collapse_short_edges();
            cout << "collpase" << endl;
            equalize_valences();
            cout << "flip" << endl;
            tangential_relaxation();
            cout << "relax" << endl;
        }
    }

    float MeshProcessing::average_edge_length() {
        float mean_length = 0;
        for (Mesh::Edge edge : mesh_.edges()) {
            mean_length += mesh_.edge_length(edge);
        }
        return mean_length / mesh_.edges_size();;
    }

    void MeshProcessing::calc_target_length(const REMESHING_TYPE &remeshing_type) {
        Mesh::Vertex_iterator v_it, v_end(mesh_.vertices_end());
        Mesh::Vertex_around_vertex_circulator vv_c, vv_end;
        Scalar length;
        Scalar mean_length;
        Scalar H;
        Scalar K;

        Mesh::Vertex_property <Scalar> curvature = mesh_.vertex_property<Scalar>("v:curvature", 0);
        Mesh::Vertex_property <Scalar> gauss_curvature = mesh_.vertex_property<Scalar>("v:gauss_curvature", 0);
        Mesh::Vertex_property <Scalar> target_length = mesh_.vertex_property<Scalar>("v:length", 0);
        Mesh::Vertex_property <Scalar> target_new_length = mesh_.vertex_property<Scalar>("v:newlength", 0);

        mean_length = average_edge_length();

        if (remeshing_type == AVERAGE) {
            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                target_length[*v_it] = mean_length * 10;
            }
        } else if (remeshing_type == CURV) {
            // calculate desired length
            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                length = 1.0;
                if (!mesh_.is_boundary(*v_it)) {
                    length = 1 / curvature[*v_it];
                }
                target_length[*v_it] = length;
            }

            // smooth desired length
            for (int i = 0; i < 5; i++) {
                for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                    vv_c = mesh_.vertices(*v_it);
                    vv_end = vv_c;
                    int vertex_count = 0;
                    float smooth_target_length = 0.0f;
                    float current_target_length = target_length[*v_it];
                    do {
                        vertex_count++;
                        Surface_mesh::Vertex neighbour = *vv_c;
                        smooth_target_length += target_length[neighbour] - current_target_length;
                    } while (++vv_c != vv_end);
                    smooth_target_length /= vertex_count;

                    target_new_length[*v_it] = current_target_length + 0.5f * smooth_target_length;
                    gauss_curvature[*v_it] = target_new_length[*v_it];
                }
            }

            // rescale desired length:
            float max_length = FLT_MIN;
            float min_length = FLT_MAX;
            float max_l = 20;
            float min_l = 0.6;

            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                float target = target_new_length[*v_it];
                if (target > max_length) {
                    max_length = target;
                }
                if (target < min_length) {
                    min_length = target;
                }
            }

            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                float target = target_new_length[*v_it];
                float coef = min_l + (max_l - min_l) * (target - min_length) / (max_length - min_length);
                target_length[*v_it] = mean_length * coef;
            }
        } else if (remeshing_type == HEIGHT) {
            // Number of times the mean of the new length max can be.
            float max_l = 6;
            float min_l = 0.3;

            mean_length = 0;
            for (Mesh::Edge edge : mesh_.edges()) {
                mean_length += mesh_.edge_length(edge);
            }
            mean_length /= mesh_.edges_size();

            float max_height = FLT_MIN;
            float min_height = FLT_MAX;
            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                float y = Point(mesh_.position(*v_it))[1];
                if (y > max_height) {
                    max_height = y;
                }
                if (y < min_height) {
                    min_height = y;
                }
            }

            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                float y = Point(mesh_.position(*v_it))[1];
                float coef = min_l + (max_l - min_l) * (y - min_height) / (max_height - min_height);
                target_length[*v_it] = mean_length * coef;
            }
        }
    }

    void MeshProcessing::split_long_edges() {
        Mesh::Edge_iterator e_it, e_end(mesh_.edges_end());
        Mesh::Vertex v0, v1, v;
        bool finished;
        int i;

        Mesh::Vertex_property <Point> normals = mesh_.vertex_property<Point>("v:normal");
        Mesh::Vertex_property <Scalar> target_length = mesh_.vertex_property<Scalar>("v:length", 0);

        for (finished = false, i = 0; !finished && i < 100; ++i) {
            finished = true;
            for (e_it = mesh_.edges_begin(); e_it != e_end; ++e_it) {
                //Compute current edge length and its target length
                Mesh::Edge edge = *e_it;
                double edge_length = mesh_.edge_length(edge);
                v0 = mesh_.vertex(edge, 0);
                v1 = mesh_.vertex(edge, 1);
                double edge_target_length =
                    target_length[v0] + (target_length[v1] - target_length[v0]) / 2.0;

                //Check if edge should be split
                if (edge_target_length * 4 / 3.0 < edge_length) {
                    finished = false;

                    //Add new  vertex to mesh, split the edge and compute the normal of the new vertex
                    v = mesh_.add_vertex(
                            mesh_.position(v0) + (mesh_.position(v1) - mesh_.position(v0)) / 2.0);
                    mesh_.split(edge, v);
                    normals[v] = mesh_.compute_vertex_normal(v);

                    //Traverse all neighbouring vertices, and compute the mean of their target lengths.
                    Surface_mesh::Vertex_around_vertex_circulator vc, vc_end;
                    vc = mesh_.vertices(v);
                    vc_end = vc;

                    float new_target_length_sum = 0;
                    int vertex_count = 0;
                    do {
                        vertex_count++;
                        Surface_mesh::Vertex neighbour = *vc;
                        new_target_length_sum += target_length[neighbour];
                    } while (++vc != vc_end);

                    target_length[v] = new_target_length_sum / vertex_count;
                }
            }
        }
    }

    void MeshProcessing::collapse_short_edges() {
        Mesh::Edge_iterator e_it, e_end(mesh_.edges_end());
        Mesh::Vertex v0, v1;
        Mesh::Halfedge h01, h10;
        bool finished, b0, b1;
        int i;
        bool hcol01, hcol10;

        Mesh::Vertex_property <Scalar> target_length = mesh_.vertex_property<Scalar>("v:length", 0);

        for (finished = false, i = 0; !finished && i < 100; ++i) {
            finished = true;

            for (e_it = mesh_.edges_begin(); e_it != e_end; ++e_it) {
                if (!mesh_.is_deleted(*e_it)) // might already be deleted
                {
                    //Compute current edge length and its target length
                    Mesh::Edge edge = *e_it;
                    double edge_length = mesh_.edge_length(edge);
                    v0 = mesh_.vertex(edge, 0);
                    v1 = mesh_.vertex(edge, 1);
                    double edge_target_length =
                        target_length[v0] + (target_length[v1] - target_length[v0]) / 2.0;

                    //Check if edge should potentially be collapsed (edge length < targe length * 4/5)
                    if (edge_target_length * 4 / 5.0 > edge_length) {
                        h01 = mesh_.halfedge(edge, 0);
                        h10 = mesh_.halfedge(edge, 1);

                        //Check if halfedges go from a boundary to a non-boundary vertex
                        b0 = !(mesh_.is_boundary(mesh_.from_vertex(h01)) && !mesh_.is_boundary(mesh_.to_vertex(h01)));
                        b1 = !(mesh_.is_boundary(mesh_.from_vertex(h10)) && !mesh_.is_boundary(mesh_.to_vertex(h10)));

                        if (b0 && b1) {
                            //CASE : both halfedges respect the boundary condition
                            //Check if they are collapsible
                            hcol01 = mesh_.is_collapse_ok(h01);
                            hcol10 = mesh_.is_collapse_ok(h10);

                            if (hcol01 && hcol10) {
                                //CASE: both halfedges are collapsible
                                //Compute vertex valences and collapse the smaller one into the larger one
                                finished = false;
                                int valence_from = mesh_.valence(mesh_.from_vertex(h01));
                                int valence_to = mesh_.valence(mesh_.to_vertex(h01));

                                if (valence_from <= valence_to) {
                                    mesh_.collapse(h01);
                                } else {
                                    mesh_.collapse(h10);
                                }
                            } else if (hcol01 && !hcol10) {
                                //Only one of the halfedges is collapsible
                                //Collapse it
                                finished = false;
                                mesh_.collapse(h01);
                            } else if (!hcol01 && hcol10) {
                                //Only one of the halfedges is collapsible
                                //Collapse it
                                finished = false;
                                mesh_.collapse(h10);
                            }
                        } else if (b0 && !b1) {
                            //CASE : halfedge h01 respects the boundary condition
                            //Check if it is collapsible, if so collapse it.
                            hcol01 = mesh_.is_collapse_ok(h01);
                            if (hcol01) {
                                finished = false;
                                mesh_.collapse(h01);
                            }
                        } else if (!b0 && b1) {
                            //CASE : halfedge h10 respects the boundary condition
                            //Check if it is collapsible, if so collapse it.
                            hcol10 = mesh_.is_collapse_ok(h10);
                            if (hcol10) {
                                finished = false;
                                mesh_.collapse(h10);
                            }
                        }
                    }
                }
            }
        }

        mesh_.garbage_collection();

        if (i == 100) std::cerr << "collapse break\n";
    }

    void MeshProcessing::equalize_valences() {
        Mesh::Edge_iterator e_it, e_end(mesh_.edges_end());
        Mesh::Vertex v0, v1, v2, v3;
        int val0, val1, val2, val3;
        int val_opt0, val_opt1, val_opt2, val_opt3;
        int ve0, ve1, ve2, ve3, ve_before, ve_after;
        bool finished;
        int i;


        // flip all edges
        for (finished = false, i = 0; !finished && i < 100; ++i) {
            finished = true;
            for (e_it = mesh_.edges_begin(); e_it != e_end; ++e_it) {
                if (!mesh_.is_boundary(*e_it)) {
                    //Check if edge is flippable
                    if (mesh_.is_flip_ok(*e_it)) {
                        //Get halfedges which will allow to access the appropriate four vertices needed.
                        Mesh::Halfedge halfedge_0 = mesh_.halfedge(*e_it, 0);
                        Mesh::Halfedge halfedge_1 = mesh_.halfedge(*e_it, 1);
                        Mesh::Halfedge halfedge_2 = mesh_.next_halfedge(halfedge_0);
                        Mesh::Halfedge halfedge_3 = mesh_.next_halfedge(halfedge_1);

                        //Get the four needed vertices
                        v0 = mesh_.to_vertex(halfedge_0);
                        v1 = mesh_.to_vertex(halfedge_1);
                        v2 = mesh_.to_vertex(halfedge_2);
                        v3 = mesh_.to_vertex(halfedge_3);

                        //For each vertex, get its optimal valence (4 if boundary, 6 otherwise)
                        val_opt0 = mesh_.is_boundary(v0) ? 4 : 6;
                        val_opt1 = mesh_.is_boundary(v1) ? 4 : 6;
                        val_opt2 = mesh_.is_boundary(v2) ? 4 : 6;
                        val_opt3 = mesh_.is_boundary(v3) ? 4 : 6;

                        //For each vertex, get its current valence
                        val0 = mesh_.valence(v0);
                        val1 = mesh_.valence(v1);
                        val2 = mesh_.valence(v2);
                        val3 = mesh_.valence(v3);

                        //For each vertex valence, compute its squared distance to its optimal valence
                        ve0 = (val0 - val_opt0) * (val0 - val_opt0);
                        ve1 = (val1 - val_opt1) * (val1 - val_opt1);
                        ve2 = (val2 - val_opt2) * (val2 - val_opt2);
                        ve3 = (val3 - val_opt3) * (val3 - val_opt3);

                        //Compute the sum of the squared distances
                        ve_before = ve0 + ve1 + ve2 + ve3;

                        //Compute the "flipped" valences
                        val0 = val0 - 1;
                        val1 = val1 - 1;
                        val2 = val2 + 1;
                        val3 = val3 + 1;

                        //Compute the "flipped" squared distances
                        ve0 = (val0 - val_opt0) * (val0 - val_opt0);
                        ve1 = (val1 - val_opt1) * (val1 - val_opt1);
                        ve2 = (val2 - val_opt2) * (val2 - val_opt2);
                        ve3 = (val3 - val_opt3) * (val3 - val_opt3);

                        //Sum the "flipped" squared distances
                        ve_after = ve0 + ve1 + ve2 + ve3;

                        //Compare the sum of the squared distances before and after the flip
                        //Flip if the sum if smaller after the flip
                        if (ve_before > ve_after) {
                            mesh_.flip(*e_it);
                            finished = false;
                        }
                    }
                }
            }
        }

        if (i == 100) std::cerr << "flip break\n";
    }

    void MeshProcessing::tangential_relaxation() {
        Mesh::Vertex_iterator v_it, v_end(mesh_.vertices_end());
        Point n;
        Point laplace;

        Mesh::Vertex_property <Point> normals = mesh_.vertex_property<Point>("v:normal");
        Mesh::Vertex_property <Point> update = mesh_.vertex_property<Point>("v:update");


        for (int iters = 0; iters < 10; ++iters) {
            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it) {
                if (!mesh_.is_boundary(*v_it)) {
                    Mesh::Vertex p = *v_it;
                    //Compute the uniform laplacian
                    laplace = Point(laplacian_operator(p));

                    //Project the laplace onto the normal, this is the parallel component of the laplace vector
                    //The perpendicular (tangent) component is simply the normal minus the parallel component.
                    n = normals[p];
                    Vec3 parallel = n * dot(laplace, n);
                    Vec3 perpendicular = laplace - parallel;

                    update[p] = perpendicular;
                }
            }

            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it)
                if (!mesh_.is_boundary(*v_it))
                    mesh_.position(*v_it) += update[*v_it];
        }
    }

    void MeshProcessing::calc_weights() {
        calc_edges_weights();
        calc_vertices_weights();
    }

    void MeshProcessing::calc_edges_weights() {
        auto e_weight = mesh_.edge_property<Scalar>("e:weight", 0.0f);
        auto points = mesh_.vertex_property<Point>("v:point");

        Mesh::Halfedge h0, h1, h2;
        Point p0, p1, p2, d0, d1;

        for (auto e: mesh_.edges()) {
            e_weight[e] = 0.0;

            h0 = mesh_.halfedge(e, 0);
            p0 = points[mesh_.to_vertex(h0)];

            h1 = mesh_.halfedge(e, 1);
            p1 = points[mesh_.to_vertex(h1)];

            if (!mesh_.is_boundary(h0)) {
                h2 = mesh_.next_halfedge(h0);
                p2 = points[mesh_.to_vertex(h2)];
                d0 = p0 - p2;
                d1 = p1 - p2;
                e_weight[e] += dot(d0, d1) / norm(cross(d0, d1));
            }

            if (!mesh_.is_boundary(h1)) {
                h2 = mesh_.next_halfedge(h1);
                p2 = points[mesh_.to_vertex(h2)];
                d0 = p0 - p2;
                d1 = p1 - p2;
                e_weight[e] += dot(d0, d1) / norm(cross(d0, d1));
            }
        }
    }

    void MeshProcessing::calc_vertices_weights() {
        Mesh::Face_around_vertex_circulator vf_c, vf_end;
        Mesh::Vertex_around_face_circulator fv_c;
        Scalar area;
        auto v_weight = mesh_.vertex_property<Scalar>("v:weight", 0.0f);

        for (auto v: mesh_.vertices()) {
            area = 0.0;
            vf_c = mesh_.faces(v);

            if (!vf_c) {
                continue;
            }

            vf_end = vf_c;

            do {
                fv_c = mesh_.vertices(*vf_c);

                const Point &P = mesh_.position(*fv_c);
                ++fv_c;
                const Point &Q = mesh_.position(*fv_c);
                ++fv_c;
                const Point &R = mesh_.position(*fv_c);

                area += norm(cross(Q - P, R - P)) * 0.5f * 0.3333f;

            } while (++vf_c != vf_end);

            v_weight[v] = 0.5f / area;
        }
    }

    void MeshProcessing::load_mesh(const string &filename) {
        if (!mesh_.read(filename)) {
            std::cerr << "Mesh " << filename << "not found, exiting." << std::endl;
            exit(-1);
        }

        cout << "Mesh " << filename << " loaded." << endl;
        cout << "# of vertices : " << mesh_.n_vertices() << endl;
        cout << "# of faces : " << mesh_.n_faces() << endl;
        cout << "# of edges : " << mesh_.n_edges() << endl;

        // Compute the center of the mesh
        mesh_center_ = Point(0.0f, 0.0f, 0.0f);
        for (auto v: mesh_.vertices()) {
            mesh_center_ += mesh_.position(v);
        }
        mesh_center_ /= mesh_.n_vertices();

        // Compute the maximum distance from all points in the mesh and the center
        dist_max_ = 0.0f;
        for (auto v: mesh_.vertices()) {
            if (distance(mesh_center_, mesh_.position(v)) > dist_max_) {
                dist_max_ = distance(mesh_center_, mesh_.position(v));
            }
        }

        compute_mesh_properties();

        // Store the original mesh, this might be useful for some computations
        mesh_init_ = mesh_;
        std::cout << "Average edge length = " << average_edge_length() << std::endl;
    }

    void MeshProcessing::compute_mesh_properties() {
        Mesh::Vertex_property <Point> vertex_normal =
            mesh_.vertex_property<Point>("v:normal");
        mesh_.update_face_normals();
        mesh_.update_vertex_normals();
        Mesh::Vertex_property <Color> v_color_valence =
            mesh_.vertex_property<Color>("v:color_valence",
                    Color(1.0f, 1.0f, 1.0f));
        Mesh::Vertex_property <Color> v_color_unicurvature =
            mesh_.vertex_property<Color>("v:color_unicurvature",
                    Color(1.0f, 1.0f, 1.0f));
        Mesh::Vertex_property <Color> v_color_curvature =
            mesh_.vertex_property<Color>("v:color_curvature",
                    Color(1.0f, 1.0f, 1.0f));
        Mesh::Vertex_property <Color> v_color_gaussian_curv =
            mesh_.vertex_property<Color>("v:color_gaussian_curv",
                    Color(1.0f, 1.0f, 1.0f));

        Mesh::Vertex_property <Scalar> vertex_valence =
            mesh_.vertex_property<Scalar>("v:valence", 0.0f);
        for (auto v: mesh_.vertices()) {
            vertex_valence[v] = mesh_.valence(v);
        }

        Mesh::Vertex_property <Scalar> v_unicurvature =
            mesh_.vertex_property<Scalar>("v:unicurvature", 0.0f);
        Mesh::Vertex_property <Scalar> v_curvature =
            mesh_.vertex_property<Scalar>("v:curvature", 0.0f);
        Mesh::Vertex_property <Scalar> v_gauss_curvature =
            mesh_.vertex_property<Scalar>("v:gauss_curvature", 0.0f);

        calc_weights();
        calc_uniform_mean_curvature();
        calc_mean_curvature();
        calc_gauss_curvature();
        color_coding(vertex_valence, &mesh_, v_color_valence, 3 /* min */,
                8 /* max */);
        color_coding(v_unicurvature, &mesh_, v_color_unicurvature);
        color_coding(v_curvature, &mesh_, v_color_curvature);
        color_coding(v_gauss_curvature, &mesh_, v_color_gaussian_curv);

        // get the mesh attributes and upload them to the GPU
        int j = 0;
        unsigned int n_vertices(mesh_.n_vertices());

        // Create big matrices to send the data to the GPU with the required
        // format
        color_valence_ = Eigen::MatrixXf(3, n_vertices);
        color_unicurvature_ = Eigen::MatrixXf(3, n_vertices);
        color_curvature_ = Eigen::MatrixXf(3, n_vertices);
        color_gaussian_curv_ = Eigen::MatrixXf(3, n_vertices);
        normals_ = Eigen::MatrixXf(3, n_vertices);
        points_ = Eigen::MatrixXf(3, n_vertices);
        indices_ = MatrixXu(3, mesh_.n_faces());

        for (auto f: mesh_.faces()) {
            std::vector<float> vv(3);
            int k = 0;
            for (auto v: mesh_.vertices(f)) {
                vv[k] = v.idx();
                ++k;
            }
            indices_.col(j) << vv[0], vv[1], vv[2];
            ++j;
        }

        j = 0;
        for (auto v: mesh_.vertices()) {
            points_.col(j) << mesh_.position(v).x,
                mesh_.position(v).y,
                mesh_.position(v).z;

            normals_.col(j) << vertex_normal[v].x,
                vertex_normal[v].y,
                vertex_normal[v].z;

            color_valence_.col(j) << v_color_valence[v].x,
                v_color_valence[v].y,
                v_color_valence[v].z;

            color_unicurvature_.col(j) << v_color_unicurvature[v].x,
                v_color_unicurvature[v].y,
                v_color_unicurvature[v].z;

            color_curvature_.col(j) << v_color_curvature[v].x,
                v_color_curvature[v].y,
                v_color_curvature[v].z;

            color_gaussian_curv_.col(j) << v_color_gaussian_curv[v].x,
                v_color_gaussian_curv[v].y,
                v_color_gaussian_curv[v].z;
            ++j;
        }
    }

    void MeshProcessing::color_coding(Mesh::Vertex_property <Scalar> prop, Mesh *mesh,
            Mesh::Vertex_property <Color> color_prop, Scalar min_value,
            Scalar max_value, int bound) {
        // Get the value array
        std::vector<Scalar> values = prop.vector();

        if (min_value == 0.0 && max_value == 0.0) {
            // discard upper and lower bound
            unsigned int n = values.size() - 1;
            unsigned int i = n / bound;
            std::sort(values.begin(), values.end());
            min_value = values[i];
            max_value = values[n - 1 - i];
        }

        // map values to colors
        for (auto v: mesh->vertices()) {
            set_color(v, value_to_color(prop[v], min_value, max_value), color_prop);
        }
    }

    void MeshProcessing::set_color(Mesh::Vertex v, const Color &col,
            Mesh::Vertex_property <Color> color_prop) {
        color_prop[v] = col;
    }

    Color MeshProcessing::value_to_color(Scalar value, Scalar min_value, Scalar max_value) {
        Scalar v0, v1, v2, v3, v4;
        v0 = min_value + 0.0f / 4.0f * (max_value - min_value);
        v1 = min_value + 1.0f / 4.0f * (max_value - min_value);
        v2 = min_value + 2.0f / 4.0f * (max_value - min_value);
        v3 = min_value + 3.0f / 4.0f * (max_value - min_value);
        v4 = min_value + 4.0f / 4.0f * (max_value - min_value);

        Color col(1.0f, 1.0f, 1.0f);

        if (value < v0) {
            col = Color(0, 0, 1);
        } else if (value > v4) {
            col = Color(1, 0, 0);
        } else if (value <= v2) {
            if (value <= v1) { // [v0, v1]
                Scalar u = (value - v0) / (v1 - v0);
                col = Color(0, u, 1);
            } else { // ]v1, v2]
                Scalar u = (value - v1) / (v2 - v1);
                col = Color(0, 1, 1 - u);
            }
        } else {
            if (value <= v3) { // ]v2, v3]
                Scalar u = (value - v2) / (v3 - v2);
                col = Color(u, 1, 0);
            } else { // ]v3, v4]
                Scalar u = (value - v3) / (v4 - v3);
                col = Color(1, 1 - u, 0);
            }
        }
        return col;
    }


}


