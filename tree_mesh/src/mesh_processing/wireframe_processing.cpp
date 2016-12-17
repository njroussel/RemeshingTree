#include "mesh_processing/wireframe_processing.h"
#include "mesh_processing/mesh_processing.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>
#define GLM_SWIZZLE
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <map>

#define IDENTITY glm::mat4(1.0f)
#define ZERO_VEC surface_mesh::Vec3(0.0f, 0.0f, 0.0f)
#define ZERO_PAIR std::pair<float, float>(0.0f, 0.0f)
#define DUMMY 1.0f

#define USE_PROPERTY true
#define DEFAULT_SPHERE_DIAMETER 0.02f
#define DEFAULT_CYLINDER_DIAMETER 0.02f

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
	
    
    void WireframeProcessing::export_mesh(const std::string& filename){
	    mesh_.write(filename);
    }
   
    void WireframeProcessing::create_wire_frame() {
        Mesh::Vertex_property<bool> v_inwireframe = mesh_.vertex_property<bool>("v:in_wireframe", false); 
        Mesh::Vertex_property<surface_mesh::Vec3> v_scale = mesh_.vertex_property<surface_mesh::Vec3>("v:wireframe_sphere_scale", ZERO_VEC); 
        Mesh::Edge_property<bool> e_inwireframe = mesh_.edge_property<bool>("e:in_wireframe", false); 
        Mesh::Edge_property<std::pair<float, float>> e_scale = mesh_.edge_property<std::pair<float, float>>("e:wireframe_cylinder_scale", ZERO_PAIR); 
        fill_wireframe_properties(v_inwireframe, v_scale, e_inwireframe, e_scale);

        replace_vertices();
        replace_edges();

        /* Once we are done we can swap the current mesh with the result of
         * the wireframe. */
        if (result_.points().size() > 0) {
            swap(result_);
        }
        else {
            std::cout << "WARNING : No wireframe created." << std::endl;
        }
        std::cout << result_.points().size() << " vertices inserted." << std::endl;
    }

    void WireframeProcessing::replace_vertices() {
        Mesh::Vertex_property<bool> v_wireframe = mesh_.get_vertex_property<bool>("v:in_wireframe"); 
        Mesh::Vertex_property<surface_mesh::Vec3> v_scale = mesh_.get_vertex_property<surface_mesh::Vec3>("v:wireframe_sphere_scale"); 

        /* Iterate over all vertices of the mesh and replace them with spheres. */
        Mesh::Vertex_iterator vc, vc_end;
        vc = mesh_.vertices_begin();
        vc_end = mesh_.vertices_end();
        do {
            Mesh::Vertex v = *vc;
            // TODO : REmoVe USE_PROPERTY
            if (!USE_PROPERTY || v_wireframe[v]) {
                Point p = mesh_.position(v);
                /* We insert the sphere at the position of the current vertex. */
                insert_mesh(sphere_, p, surface_mesh::Vec3(0, 1, 0), 0.0f, v_scale[v], false);
            }
        } while (++vc != vc_end);
    }

    void WireframeProcessing::replace_edges() {
        Mesh::Edge_property<bool> e_wireframe = mesh_.get_edge_property<bool>("e:in_wireframe"); 
        Mesh::Edge_property<std::pair<float, float>> e_scale = mesh_.get_edge_property<std::pair<float, float>>("e:wireframe_cylinder_scale"); 

        /* Iterate over all edges of the mesh and replace them with cylinders. */
        Mesh::Edge_iterator ec, ec_end;
        ec = mesh_.edges_begin();
        ec_end = mesh_.edges_end();

        do {
            Mesh::Edge e = *ec;
            // TODO : remove macro
            if (!USE_PROPERTY || e_wireframe[e]) {
                Mesh::Vertex v0 = mesh_.vertex(e, 0);
                Mesh::Vertex v1 = mesh_.vertex(e, 1);
                Point p0 = mesh_.position(v0);
                Point p1 = mesh_.position(v1);

                surface_mesh::Vec3 scale = surface_mesh::Vec3(std::get<0>(e_scale[e]), mesh_.edge_length(e), std::get<1>(e_scale[e]));

                Point pos = (p0 + p1) / 2.0f;

                /* Compute the rotation of the edge. */
                glm::vec3 edge_dir = Vec3_to_glm((p0 - p1).normalize());
                glm::vec3 cylinder_axis = glm::vec3(0.0f, 1.0f, 0.0f);
                surface_mesh::Vec3 rot_axis = glm_to_Vec3(glm::cross(cylinder_axis, edge_dir));

                float rot_angle = glm::acos(glm::dot(edge_dir, cylinder_axis));

                insert_mesh(cylinder_, pos, rot_axis, rot_angle, scale, true);
            }
        }while (++ec != ec_end);
    }

    void WireframeProcessing::insert_mesh(Mesh& to_insert, const surface_mesh::Point pos, const surface_mesh::Vec3 rot_axis, float rot_angle, surface_mesh::Vec3 scale, bool edge) {
        const float low = to_insert.position(get_lowest_point(to_insert))[1];
        const float high = to_insert.position(get_highest_point(to_insert))[1];
        glm::vec3 axis = Vec3_to_glm(rot_axis);
        glm::mat4 rot_matrix = glm::rotate(IDENTITY, rot_angle, axis);

        /* Surface_mesh does not allow us to insert edges but only vertices and
         * faces. So the idea is to iterate over all faces of the mesh to_insert 
         * and add them to the current mesh (by adding the vertices first and 
         * then creating the face).
         */
        Mesh::Face_iterator fc, fc_end;
        fc = to_insert.faces_begin();
        fc_end = to_insert.faces_end();

        /* The vertex mapping will keep track of all vertices that are already
         * in the current mesh. */
        std::map<Mesh::Vertex, Mesh::Vertex> vertex_mapping;
        do {
            Mesh::Face f = *fc;

            Mesh::Vertex_around_face_circulator vc, vc_end;
            vc = to_insert.vertices(f);
            vc_end = vc;

            /* Will contain the vertices that are inserted into the current mesh. */
            std::vector<Mesh::Vertex> vertices;

            do {
                Mesh::Vertex v = *vc;
                /* If the vertex v is not already inserted we compute its
                 * position and add it to the mesh. */
                if (vertex_mapping.find(v) == vertex_mapping.end()) {
                    surface_mesh::Vec3 tmp_scale;
                    if (edge) {
                        /* Incase of an edge we apply a non linear scaling over
                         * the cylinder. */
                        float height = to_insert.position(v)[1];
                        float relative_height = (height - low) / (high - low);
                        float scale_along_axis = scale[0] + relative_height * (scale[2] - scale[0]);
                        tmp_scale[0] = scale_along_axis;
                        tmp_scale[1] = scale[1];
                        tmp_scale[2] = scale_along_axis;
                    }
                    else {
                        tmp_scale = scale;
                    }
                    Point p = to_insert.position(v) * tmp_scale;

                    /* Rotations */
                    glm::vec3 pos_glm = Vec3_to_glm(p);
                    glm::vec3 rotated_pos;
                    if (axis != glm::vec3(0.0f))
                        rotated_pos = glm::vec3(rot_matrix * glm::vec4(pos_glm, 1.0f));
                    else
                        rotated_pos = pos_glm;
                    p = glm_to_Vec3(rotated_pos);

                    /* Translation. */
                    p = p + pos;

                    
                    Mesh::Vertex to_add = result_.add_vertex(p);
                    vertices.push_back(to_add);
                    /* We keep the corresponding vertex in the mesh so that we
                     * avoid inseting it multiple times. */
                    vertex_mapping[v] = to_add;
                }
                else {
                    /* v was alredy inserted, a simple look up suffices to
                     * retrive the corresponding vertex. */
                    vertices.push_back(vertex_mapping[v]);
                }
            }while (++vc != vc_end);

            assert (vertices.size() == 3);

            /* We can add the new face to the mesh. */
            Mesh::Face new_face = result_.add_face(vertices);
            /* We also compute the normal for the shading. */
            result_.compute_face_normal(new_face);
        } while (++fc != fc_end);
    }

    void WireframeProcessing::fill_wireframe_properties(Mesh::Vertex_property<bool> v_inwireframe, Mesh::Vertex_property<surface_mesh::Vec3> v_scale, Mesh::Edge_property<bool> e_inwireframe, Mesh::Edge_property<std::pair<float, float>> e_scale) {
        /* Default behavior of the wireframe_processing class. */
        for (Mesh::Vertex v : mesh_.vertices()) {
            v_inwireframe[v] = true;
            v_scale[v] = surface_mesh::Vec3(DEFAULT_SPHERE_DIAMETER, DEFAULT_SPHERE_DIAMETER, DEFAULT_SPHERE_DIAMETER);
        }
        for (Mesh::Edge e : mesh_.edges()) {
            e_inwireframe[e] = true;
            e_scale[e] = std::make_pair(DEFAULT_CYLINDER_DIAMETER, DEFAULT_CYLINDER_DIAMETER);
        }
    }
}
