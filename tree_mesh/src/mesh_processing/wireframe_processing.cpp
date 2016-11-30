#include "mesh_processing/wireframe_processing.h"
#include "mesh_processing/mesh_processing.h"

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

    void WireframeProcessing::replace_edges(const float cylinder_radius) {

    }

    void WireframeProcessing::insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot, const surface_mesh::Vec3 scale) {
        Mesh::Face_iterator fc, fc_end;
        /* TODO : Maybe change MeshProcessing to Mesh, but then cannot read from file ? :/ */
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
                Point p = to_insert.position(v) * scale + pos;
                /* TODO : Avoid creating vertices multiple times. May not be trivial... */
                Mesh::Vertex to_add = mesh_.add_vertex(p);
                vertices.push_back(to_add);
            }while (++vc != vc_end);

            if (vertices.size() != 3){
                throw std::string (" HELL NO ");
            }

            Mesh::Face new_face = mesh_.add_face(vertices);
            /* TODO : Compute normals ( doesn't work as is ). */
            mesh_.compute_face_normal(new_face);
            /* TODO : Rotations. */
        } while (++fc != fc_end);
    }
}
