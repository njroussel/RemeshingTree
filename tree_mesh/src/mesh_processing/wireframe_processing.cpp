#include "mesh_processing/wireframe_processing.h"
#include "mesh_processing/mesh_processing.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>
#define GLM_SWIZZLE
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <map>

#define IDENTITY glm::mat4(1.0f)
#define ZERO_VEC surface_mesh::Vec3(0.0f, 0.,0f, 0.0f)

#define USE_PROPERTY false

/* Some conversion macros. */
#define glm_to_Vec3(v) \
    surface_mesh::Vec3(v.x, v.y, v.z)

#define Vec3_to_glm(v) \
    glm::vec3(v[0], v[1], v[2])

namespace mesh_processing {
    WireframeProcessing::WireframeProcessing(const std::string& filename, const std::string& sphere_filename, const std::string& cylinder_filename) : MeshProcessing(filename) {
        sphere_.read(sphere_filename);
        cylinder_.read(cylinder_filename);
    }

    WireframeProcessing::~WireframeProcessing(void) {

    }

    void WireframeProcessing::fill_vertex_wireframe_properties(void) {
        /* Default behavior, all vertices are in the wireframe and their scale
         * is determined by the DEFAULT_SPHERE_SIZE macro. */
        Mesh::Vertex_property<bool> v_wireframe = mesh_.vertex_property<bool>("v:in_wireframe", false); 
        Mesh::Vertex_property<surface_mesh::Vec3> v_scale = mesh_.vertex_property<surface_mesh::Vec3>("v:wireframe_sphere_scale", ZERO_VEC); 
        for (Mesh::Vertex v : mesh_.vertices()) {
            v_wireframe[v] = true;
            v_scale[v] = surface_mesh::Vec3(spheres_radius, spheres_radius, spheres_radius);
        }
    }

    void WireframeProcessing::fill_edge_wireframe_properties(void) {
        Mesh::Edge_property<bool> e_wireframe = mesh_.edge_property<bool>("e:in_wireframe", false); 
        Mesh::Edge_property<surface_mesh::Vec3> e_scale = mesh_.edge_property<surface_mesh::Vec3>("e:wireframe_cylinder_scale", ZERO_VEC); 
        for (Mesh::Edge e : mesh_.edges()) {
            e_wireframe[e] = true;
            e_scale[e] = 
        }
    }

    void WireframeProcessing::create_wire_frame() {
        replace_vertices(spheres_radius);
        replace_edges(cylinder_radius);
        if (result_.points().size() > 0) {
            swap(result_);
        }
        else {
            std::cout << "WARNING : No wireframe created." << std::endl;
        }
        std::cout << result_.points().size() << " vertices inserted." << std::endl;
    }

    void WireframeProcessing::replace_vertices() {
        Mesh::Vertex_property<bool> v_wireframe = mesh_.vertex_property<bool>("v:in_wireframe", false); 
        Mesh::Vertex_property<surface_mesh::Vec3> v_scale = mesh_.vertex_property<surface_mesh::Vec3>("v:wireframe_sphere_scale", ZERO_VEC); 
        Mesh::Vertex_iterator vc, vc_end;
        vc = mesh_.vertices_begin();
        vc_end = mesh_.vertices_end();
        do {
            Mesh::Vertex v = *vc;
            if (!USE_PROPERTY || v_wireframe[v]) {
                Point p = mesh_.position(v);
                insert_mesh(sphere_, p, surface_mesh::Vec3(0, 1, 0), 0.0f, v_scale[v]);
            }
        } while (++vc != vc_end);
    }

    void WireframeProcessing::replace_edges() {
        Mesh::Edge_property<bool> e_wireframe = mesh_.edge_property<bool>("e:in_wireframe", false); 
        Mesh::Edge_property<surface_mesh::Vec3> e_scale = mesh_.edge_property<surface_mesh::Vec3>("e:wireframe_cylinder_scale", ZERO_VEC); 
        Mesh::Edge_iterator ec, ec_end;
        ec = mesh_.edges_begin();
        ec_end = mesh_.edges_end();
        do {
            Mesh::Edge e = *ec;
            if (!USE_PROPERTY || e_wireframe[e]) {
                Mesh::Vertex v0 = mesh_.vertex(e, 0);
                Mesh::Vertex v1 = mesh_.vertex(e, 1);
                Point p0 = mesh_.position(v0);
                Point p1 = mesh_.position(v1);
                surface_mesh::Vec3 scale = surface_mesh::Vec3(e_scale[0], mesh_.edge_length(e), e_scale[2]);
                Point pos = (p0 + p1) / 2.0f;

                glm::vec3 edge_dir = Vec3_to_glm((p0 - p1).normalize());
                glm::vec3 cylinder_axis = glm::vec3(0.0f, 1.0f, 0.0f);
                surface_mesh::Vec3 rot_axis = glm_to_Vec3(glm::cross(cylinder_axis, edge_dir));

                float rot_angle = glm::acos(glm::dot(edge_dir, cylinder_axis));

                insert_mesh(cylinder_, pos, rot_axis, rot_angle, scale);
            }
        }while (++ec != ec_end);
    }

    void WireframeProcessing::insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot_axis, const float rot_angle, const surface_mesh::Vec3 scale) {
        glm::vec3 axis = Vec3_to_glm(rot_axis);
        glm::mat4 rot_matrix = glm::rotate(IDENTITY, rot_angle, axis);
        Mesh::Face_iterator fc, fc_end;
        fc = to_insert.faces_begin();
        fc_end = to_insert.faces_end();
        std::map<Mesh::Vertex, Mesh::Vertex> vertex_mapping;
        do {
            Mesh::Face f = *fc;

            Mesh::Vertex_around_face_circulator vc, vc_end;
            vc = to_insert.vertices(f);
            vc_end = vc;

            std::vector<Mesh::Vertex> vertices;

            do {
                Mesh::Vertex v = *vc;
                if (vertex_mapping.find(v) == vertex_mapping.end()) {
                    Point p = to_insert.position(v) * scale;

                    /* Rotations */
                    glm::vec3 pos_glm = Vec3_to_glm(p);
                    glm::vec3 rotated_pos;
                    if (axis != glm::vec3(0.0f))
                        rotated_pos = glm::vec3(rot_matrix * glm::vec4(pos_glm, 1.0f));
                    else
                        rotated_pos = pos_glm;
                    p = glm_to_Vec3(rotated_pos);

                    p = p + pos;

                    
                    Mesh::Vertex to_add = result_.add_vertex(p);
                    vertices.push_back(to_add);
                    vertex_mapping[v] = to_add;
                }
                else {
                    vertices.push_back(vertex_mapping[v]);
                }
            }while (++vc != vc_end);

            if (vertices.size() != 3){
                throw std::string (" HELL NO ");
            }

            Mesh::Face new_face = result_.add_face(vertices);
            result_.compute_face_normal(new_face);
        } while (++fc != fc_end);
    }
}
