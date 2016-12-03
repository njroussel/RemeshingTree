#include "utils.h"

Mesh::Vertex get_highest_point(const Mesh &mesh) {
    Mesh::Vertex_iterator vc, vc_end;
    vc = mesh.vertices_begin();
    vc_end = mesh.vertices_end();
    float highest = FLT_MIN; 
    Mesh::Vertex highest_v;
    do {
        Mesh::Vertex v = *vc;
        Point p = mesh.position(v);
        if (p[1] > highest) {
            highest = p[1];
            highest_v = v;
        }
    } while (++vc!=vc_end); 
    return highest_v;
}

Mesh::Vertex get_lowest_point(const Mesh &mesh) {
    Mesh::Vertex_iterator vc, vc_end;
    vc = mesh.vertices_begin();
    vc_end = mesh.vertices_end();
    float lowest = FLT_MAX; 
    Mesh::Vertex lowest_v;
    do {
        Mesh::Vertex v = *vc;
        Point p = mesh.position(v);
        if (p[1] < lowest) {
            lowest = p[1];
            lowest_v = v;
        }
    } while (++vc!=vc_end); 
    return lowest_v;
}
