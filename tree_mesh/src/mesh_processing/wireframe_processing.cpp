#include "mesh_processing/wireframe_processing.h"
#include "mesh_processing/mesh_processing.h"
#include <cmath>

namespace mesh_processing {
    WireframeProcessing::WireframeProcessing(const std::string& filename, const std::string& sphere_filename, const std::string& cylinder_filename) : MeshProcessing(filename) {
        sphere_.read(sphere_filename);
        cylinder_.read(cylinder_filename);
    }

    WireframeProcessing::~WireframeProcessing(void) {

    }

    void WireframeProcessing::create_wire_frame(const float spheres_radius, const float cylinder_radius) {
        replace_vertices(spheres_radius);
        replace_edges(cylinder_radius);
        swap(result_);
    }

    void WireframeProcessing::replace_vertices(const float spheres_radius) {
        Mesh::Vertex_iterator vc, vc_end;
        vc = mesh_.vertices_begin();
        vc_end = mesh_.vertices_end();
        do {
            Mesh::Vertex v = *vc;
            Point p = mesh_.position(v);
            insert_mesh(sphere_, p, surface_mesh::Vec3(0, 0, 0), surface_mesh::Vec3(spheres_radius, spheres_radius, spheres_radius));
        } while (++vc != vc_end);
    }

    static float dot(surface_mesh::Vec3 v1, surface_mesh::Vec3 v2) {
        surface_mesh::Vec3 tmp = v1 * v2;
        return tmp[0] + tmp[1] + tmp[2];
    }

    void WireframeProcessing::replace_edges(const float cylinder_radius) {
        Mesh::Edge_iterator ec, ec_end;
        ec = mesh_.edges_begin();
        ec_end = mesh_.edges_end();
        int skip = 0;
        do {
            if (skip > 0 && --skip == 0)
                continue;
            Mesh::Edge e = *ec;
            Mesh::Vertex v0 = mesh_.vertex(e, 0);
            Mesh::Vertex v1 = mesh_.vertex(e, 1);
            Point p0 = mesh_.position(v0);
            Point p1 = mesh_.position(v1);
            surface_mesh::Vec3 scale = surface_mesh::Vec3(cylinder_radius, mesh_.edge_length(e), cylinder_radius);
            Point pos = (p0 + p1) / 2.0f;

            /* Rotations : shit just got real. */
            std::cout << " Edge = ( " << p0 << " ; " << p1 << " )" << std::endl;

            surface_mesh::Vec3 edge_dir = (p0 - p1).normalize();
            std::cout << "edge_dir = " << edge_dir << std::endl;

            float angle_axis_x = std::acos(dot(edge_dir, surface_mesh::Vec3(1, 0, 0)));
            float angle_axis_y = std::acos(dot(edge_dir, surface_mesh::Vec3(0, 1, 0)));
            float angle_axis_z = std::acos(dot(edge_dir, surface_mesh::Vec3(0, 0, 1)));

            std::cout << "angles = ( " << angle_axis_x << " ; " << angle_axis_y << " ; " << angle_axis_z << " )" << std::endl;

            float pi02 = 3.14 / 2.0;
            surface_mesh::Vec3 rot(angle_axis_x, angle_axis_y + pi02, angle_axis_z);
            /* *** */

#define ZERO surface_mesh::Vec3(0, 0, 0)
            insert_mesh(cylinder_, pos, rot, scale);
            //break;
        }while (++ec != ec_end);
    }

    void WireframeProcessing::insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot, const surface_mesh::Vec3 scale) {
        std::cout << "rot  =  " << rot << std::endl;
        Mesh::Face_iterator fc, fc_end;
        fc = to_insert.faces_begin();
        fc_end = to_insert.faces_end();
        do {
            Mesh::Face f = *fc;

            Mesh::Vertex_around_face_circulator vc, vc_end;
            vc = to_insert.vertices(f);
            vc_end = vc;

            std::vector<Mesh::Vertex> vertices;

            do {
                Mesh::Vertex v = *vc;
                Point p = to_insert.position(v) * scale;
                /* TODO : Avoid creating vertices multiple times. May not be trivial... */

                /* Rotations */
                Point after_rot;
                float phi = rot[0];
                after_rot[0] = p[0];
                after_rot[1] = std::cos(phi) * p[1] - std::sin(phi) * p[2];
                after_rot[2] = std::sin(phi) * p[1] + std::cos(phi) * p[2];
                p = after_rot;

                phi = rot[1];
                after_rot[0] = std::cos(phi) * p[0] + std::sin(phi) * p[2];
                after_rot[1] = p[1];
                after_rot[2] = -std::sin(phi) * p[0] + std::cos(phi) * p[2];
                p = after_rot;

                phi = rot[2];
                after_rot[0] = std::cos(phi) * p[0] - std::sin(phi) * p[1];
                after_rot[1] = std::sin(phi) * p[0] + std::cos(phi) * p[1];
                after_rot[2] = p[2];
                p = after_rot;


                p = p + pos;

                Mesh::Vertex to_add = result_.add_vertex(p);
                vertices.push_back(to_add);
            }while (++vc != vc_end);

            if (vertices.size() != 3){
                throw std::string (" HELL NO ");
            }

            Mesh::Face new_face = result_.add_face(vertices);
            /* TODO : Compute normals ( doesn't work as is ).
             * But may not be important for printing. */
            result_.compute_face_normal(new_face);
        } while (++fc != fc_end);
    }
}
