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
#ifndef MESH_PROCESSING_H
#define MESH_PROCESSING_H

#include <surface_mesh/Surface_mesh.h>
#include <surface_mesh/types.h>
#include <Eigen/Sparse>

using std::string;
using surface_mesh::Point;
using surface_mesh::Scalar;
using surface_mesh::Color;
using std::min;
using std::max;
using std::cout;
using std::endl;

typedef surface_mesh::Surface_mesh Mesh;
typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;

namespace mesh_processing {
    enum REMESHING_TYPE : int { AVERAGE = 0, CURV = 1, HEIGHT = 2 };

    class MeshProcessing {
        public:
            MeshProcessing(const string& filename);
            ~MeshProcessing();

            const surface_mesh::Point get_mesh_center();
            const float get_dist_max();
            const Eigen::MatrixXf* get_points();
            const MatrixXu* get_indices();
            const Eigen::MatrixXf* get_normals();
            const Eigen::MatrixXf* get_colors_valence();
            const Eigen::MatrixXf* get_colors_unicurvature();
            const Eigen::MatrixXf* get_colors_gaussian_curv();
            const Eigen::MatrixXf* get_color_curvature();
            const unsigned int get_number_of_face();


            void remesh(const REMESHING_TYPE &remeshing_type, const int &num_iterations);
            void calc_target_length(const REMESHING_TYPE &remeshing_type);
            void split_long_edges();
            void collapse_short_edges();
            void equalize_valences();
            void tangential_relaxation();

            void load_mesh(const string& filename);
            void compute_mesh_properties();

            void calc_mean_curvature();
            void calc_uniform_mean_curvature();
            void calc_gauss_curvature();

            /* Some operations on the mesh. */
            surface_mesh::Vec3 laplacian_operator(Mesh::Vertex x);
            surface_mesh::Vec3 laplace_beltrami_operator(Mesh::Vertex x, bool normalize);

            const Mesh& mesh() { return mesh_;}

        private:
            void calc_weights();
            void calc_edges_weights();
            void calc_vertices_weights();

    protected:
            Mesh mesh_;
            Mesh mesh_init_;
            surface_mesh::Point mesh_center_ = surface_mesh::Point(0.0f, 0.0f, 0.0f);
            float dist_max_ = 0.0f;

            Eigen::MatrixXf points_;
            MatrixXu indices_;
            Eigen::MatrixXf normals_;
            Eigen::MatrixXf color_valence_;
            Eigen::MatrixXf color_unicurvature_;
            Eigen::MatrixXf color_gaussian_curv_;
            Eigen::MatrixXf color_curvature_;

            void color_coding(Mesh::Vertex_property<surface_mesh::Scalar> prop,
                    Mesh *mesh,
                    Mesh::Vertex_property<surface_mesh::Color> color_prop,
                    surface_mesh::Scalar min_value = 0.0,
                    surface_mesh::Scalar max_value = 0.0, int bound = 20);
            void set_color(Mesh::Vertex v, const surface_mesh::Color& col,
                    Mesh::Vertex_property<surface_mesh::Color> color_prop);
            surface_mesh::Color value_to_color(surface_mesh::Scalar value,
                    surface_mesh::Scalar min_value,
                    surface_mesh::Scalar max_value);
    };

}

#endif // MESH_PROCESSING_H
