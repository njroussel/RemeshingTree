#include "utils.h"

surface_mesh::Vec3 get_highest_point(const Mesh &mesh) {
    Mesh::Vertex_iterator vc, vc_end;
    vc = mesh.vertices_begin();
    vc_end = mesh.vertices_end();
    surface_mesh::Vec3 highest = surface_mesh::Vec3(FLT_MIN,FLT_MIN,FLT_MIN);
    do {
        Mesh::Vertex v = *vc;
        Point p = mesh.position(v);
        if (p[1] > highest[1]) {
            highest = p;
        }
    } while (++vc!=vc_end); 
    return highest;
}

surface_mesh::Vec3 get_lowest_point(const Mesh &mesh) {
    Mesh::Vertex_iterator vc, vc_end;
    vc = mesh.vertices_begin();
    vc_end = mesh.vertices_end();
    surface_mesh::Vec3 lowest = surface_mesh::Vec3(FLT_MAX,FLT_MAX,FLT_MAX);
    do {
        Mesh::Vertex v = *vc;
        Point p = mesh.position(v);
        if (p[1] < lowest[1]) {
            lowest = p;
        }
    } while (++vc!=vc_end); 
    return lowest;
}
