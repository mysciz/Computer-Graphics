#include "halfedge.h"

#include <set>
#include <map>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <spdlog/spdlog.h>

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::optional;
using std::set;
using std::size_t;
using std::string;
using std::unordered_map;
using std::vector;

HalfedgeMesh::EdgeRecord::EdgeRecord(unordered_map<Vertex*, Matrix4f>& vertex_quadrics, Edge* e) :
    edge(e)
{
    (void)vertex_quadrics;
    optimal_pos = Vector3f(0.0f, 0.0f, 0.0f);
    cost        = 0.0f;
}

bool operator<(const HalfedgeMesh::EdgeRecord& a, const HalfedgeMesh::EdgeRecord& b)
{
    if (a.cost == b.cost) {
        // Sort by edge id if cost are the same
        return a.edge->id < b.edge->id;
    }
    return a.cost < b.cost;
}

optional<Edge*> HalfedgeMesh::flip_edge(Edge* e)
{
    if(e->on_boundary())
    return std::nullopt;
     //要用到的半边
    Halfedge*h =e->halfedge;
    Halfedge*h_inv =h->inv;
    Halfedge*h_2_3 =h->next;
    Halfedge*h_3_1 =h_2_3->next;
    Halfedge*h_1_4 =h_inv->next;
    Halfedge*h_4_2 =h_1_4->next;
    //要用到的顶点
    //v1 andv2 are verticesalongtheedge
    Vertex*v1 =h->from;
    Vertex*v2 =h_inv->from;
    //v3 andv4 are verticesopposite the edge
    Vertex*v3 =h_3_1->from;
    Vertex*v4 =h_4_2->from;
    //要用到的面片
    Face*f1 =h->face;
    Face*f2 =h_inv->face;
    //重新连接各基本元素
    h->next = h_3_1;
    h->prev = h_1_4;
    h->from = v4;

 //其余部分请自己完成
    h->face = f1;

    h_inv->next = h_4_2;
    h_inv->prev = h_2_3;
    h_inv->from = v3;
    h_inv->face = f2;

    // 更新 h_2_3 和 h_4_2 的连接关系
    h_2_3->next = h_inv;
    h_2_3->prev = h_4_2;
    h_2_3->from = v2;
    h_2_3->face = f2;

    h_4_2->next = h_2_3;
    h_4_2->prev = h_inv;
    h_4_2->from = v4;
    h_4_2->face = f2;

    // 更新 h_3_1 和 h_1_4 的连接关系
    h_3_1->next = h_1_4;
    h_3_1->prev = h;
    h_3_1->from = v3;
    h_3_1->face = f1;

    h_1_4->next = h;
    h_1_4->prev = h_3_1;
    h_1_4->from = v1;
    h_1_4->face = f1;

    // 更新顶点的半边
    v1->halfedge = h_1_4;
    v2->halfedge = h_2_3;
    v3->halfedge = h_3_1;
    v4->halfedge = h_4_2;
    //更新面片的半边
    f1->halfedge = h_1_4;
    f2->halfedge = h_2_3;
    // 更新边的半边
    e->halfedge = h;
    logger->trace("---startflippingedge {}---",e->id);
    //假如将e的两个端点赋值给v1和v2，将两个相对位置的点赋值给v3和v4
    logger->trace("(v1, v2)({},{})",v1->id,v2->id);
    logger->trace("(v3, v4)({},{})",v3->id,v4->id);
    //修改各种指针
    logger->trace("face 123:{}->{}->{}",f1->halfedge->from->id,
    f1->halfedge->next->from->id,
    f1->halfedge->next->next->from->id);
    logger->trace("face 214:{}->{}->{}",f2->halfedge->from->id,
    f2->halfedge->next->from->id,
    f2->halfedge->next->next->from->id);
    logger->trace("---end---");
    return e;
}

optional<Vertex*> HalfedgeMesh::split_edge(Edge* e)
{

    // 创建新顶点
    Vertex* v_new = new_vertex();
    if (!v_new) return std::nullopt;

    // 要用到的半边
    Halfedge* h_1_n = e->halfedge;
    Halfedge* h_2_n = h_1_n->inv;
    Halfedge* h_2_3 = h_1_n->next;
    Halfedge* h_3_1 = h_2_3->next;
    Halfedge* h_1_4 = h_2_n->next;
    Halfedge* h_4_2 = h_1_4->next;
    // 要用到的顶点
    Vertex* v1 = h_1_n->from;
    Vertex* v2 = h_2_n->from;
    Vertex* v3 = h_3_1->from;
    Vertex* v4 = h_4_2->from;
    // 要用到的面片
    Face* f1_3 = h_1_n->face;
    Face* f2_4 = h_2_n->face;
    // 重新连接各基本元素
    Halfedge* h_n_3 = new_halfedge();
    Halfedge* h_n_4 = new_halfedge();
    Halfedge* h_3_n = new_halfedge();
    Halfedge* h_4_n = new_halfedge();
    Halfedge* h_n_1 = new_halfedge();
    Halfedge* h_n_2 = new_halfedge();
    Face* f2_3 = new_face();
    Face* f1_4 = new_face();
    Edge* e2_n = new_edge();
    Edge* e3_n = new_edge();
    Edge* e4_n = new_edge();
    v_new->pos=(v1->pos+v2->pos)/2.0f;
    v_new->halfedge = h_n_1;
    //设置新建元素
    h_1_n->set_neighbors(h_n_3,h_3_1,h_n_1,v1,e,f1_3);
    h_3_1->prev = h_n_3;
    h_n_3->set_neighbors(h_3_1,h_1_n,h_3_n,v_new,e3_n,f1_3);
    e3_n->halfedge=h_n_3;
    h_n_2->set_neighbors(h_2_3,h_3_n,h_2_n,v_new,e2_n,f2_3);
    f1_3->halfedge = h_3_1;
    e2_n->halfedge=h_n_2;
    h_2_3->next = h_3_n;
    h_2_3->prev = h_n_2;;
    h_2_3->face = f2_3;
    h_3_n->set_neighbors(h_n_2,h_2_3,h_n_3,v3,e3_n,f2_3);
    h_n_1->set_neighbors(h_1_4,h_4_n,h_1_n,v_new,e,f1_4);
    f1_4->halfedge = h_1_4;
    e4_n->halfedge=h_n_4;
    h_4_2->next = h_2_n;
    h_4_2->prev = h_n_4;
    h_4_2->face = f2_4;
    h_n_4->set_neighbors(h_4_2,h_2_n,h_4_n,v_new,e4_n,f2_4);
    h_4_n->set_neighbors(h_n_1,h_1_4,h_n_4,v4,e4_n,f1_4);
    //重新建立各基本元素
    h_2_n->set_neighbors(h_n_4,h_4_2,h_n_2,v2,e2_n,f2_4);
    h_3_1->next = h_1_n;
    h_3_1->face = f1_3;
    h_1_4->next = h_4_n;
    h_1_4->prev = h_n_1;
    h_1_4->face = f1_4;
    //更新面片的半边
    f2_4->halfedge = h_4_2;
    f2_3->halfedge = h_2_3;
   
    return v_new;
}

optional<Vertex*> HalfedgeMesh::collapse_edge(Edge* e)
{
   if (!e || !e->halfedge) return std::nullopt;

    // 创建新顶点
    Vertex* v_new = new_vertex();
    if (!v_new) return std::nullopt;
    // 要用到的半边
    Halfedge* h=e->halfedge;
    Halfedge* h_inv=h->inv;
    Halfedge* h_inv1=h->next->inv;
    Halfedge* h_inv2=h_inv->next->inv;
    Halfedge* h1=h->prev->inv;
    Halfedge* h2=h_inv->prev->inv;
    h_inv2->from->halfedge=h_inv2;
    h_inv1->from->halfedge=h_inv1;
    h2->edge->halfedge=h2;
    h1->edge->halfedge=h1;
    h_inv2->edge=h2->edge;
    h_inv1->edge=h1->edge;
    v_new->pos=(h->from->pos+h_inv->from->pos)/2.0f;
    v_new->halfedge=h2;
    h2->inv=h_inv2;
    h_inv2->inv=h2;
    Halfedge* next_h = h->next;
    while(next_h!=h){
        next_h->from=v_new;
        next_h=next_h->inv->next;
    }

    if(e->on_boundary()==false){
    h1->inv=h_inv1;

    h_inv1->inv=h1;
    
    erase(h->from);
    erase(h_inv->from);
    erase(h->next->edge);
    erase(h->next);
    erase(h_inv->next->edge);
    erase(h_inv->next);
    erase(h->prev);
    erase(h_inv->prev);
    erase(h->edge);
    erase(h->face);
    erase(h_inv->face);
    erase(h);
    erase(h_inv);
    }
    else{
    erase(h->from);
    erase(h_inv->from);
    erase(h_inv->next->edge);
    erase(h_inv->next);
    erase(h_inv->prev);
    erase(h_inv->edge);
    erase(h_inv->face);
    erase(h_inv);
    }

    return v_new;
}

void HalfedgeMesh::loop_subdivide()
{
    optional<HalfedgeMeshFailure> check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    logger->info(
        "subdivide object {} (ID: {}) with Loop Subdivision strategy", object.name, object.id
    );
    logger->info("original mesh: {} vertices, {} faces in total", vertices.size, faces.size);
    // Each vertex and edge of the original mesh can be associated with a vertex
    // in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions using the connectivity of the original
    // (coarse) mesh. Navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.
    // We will then assign vertex positions in the new mesh based on the values
    // we computed for the original mesh.

    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    for (Vertex* v = vertices.head; v != nullptr; v = v->next_node) {
        v->is_new = false;
        bool is_boundary_vertex = false;
        // Check if the vertex is on the boundary by checking its incident halfedges
        Halfedge* he = v->halfedge;
        do {
            if (he->is_boundary()) {
                is_boundary_vertex = true;
                break;
            }
            he = he->next;
        } while (he != v->halfedge);

        // Loop subdivision rule for vertices
        if (is_boundary_vertex) {
            // For boundary vertices
            v->new_pos = (1 - 3.0 / 8) * v->pos + 3.0 / 8 * (v->halfedge->from->pos + v->halfedge->inv->from->pos) / 2;
        } else {
            // For interior vertices
            int n = v->degree();
            v->new_pos = (n - 3.0) / n * v->pos + 3.0 / (2 * n) * (v->halfedge->from->pos + v->halfedge->inv->from->pos);
            he = v->halfedge;
            do {
                v->new_pos += 1.0 / (2 * n) * he->inv->next->from->pos;
                he = he->next;
            } while (he != v->halfedge);
        }
    }
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.

    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new.
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    // I use a vector to store iterators of original because there are three kinds of
    // edges: original edges, edges split from original edges and newly added edges.
    // The newly added edges are marked with Edge::is_new property, so there is not
    // any other property to mark the edges I just split.

    // Now flip any new edge that connects an old and new vertex.

    // Finally, copy new vertex positions into the Vertex::pos.

    // Once we have successfully subdivided the mesh, set global_inconsistent
    // to true to trigger synchronization with GL::Mesh.
    global_inconsistent = true;
    logger->info("subdivided mesh: {} vertices, {} faces in total", vertices.size, faces.size);
    logger->info("Loop Subdivision done");
    logger->info("");
    validate();
}

void HalfedgeMesh::simplify()
{
    optional<HalfedgeMeshFailure> check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    logger->info("simplify object {} (ID: {})", object.name, object.id);
    logger->info("original mesh: {} vertices, {} faces", vertices.size, faces.size);
    unordered_map<Vertex*, Matrix4f> vertex_quadrics;
    unordered_map<Face*, Matrix4f>   face_quadrics;
    unordered_map<Edge*, EdgeRecord> edge_records;
    set<EdgeRecord>                  edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics

    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics

    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.

    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    logger->info("simplified mesh: {} vertices, {} faces", vertices.size, faces.size);
    logger->info("simplification done\n");
    global_inconsistent = true;
    validate();
}

void HalfedgeMesh::isotropic_remesh()
{
    optional<HalfedgeMeshFailure> check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    logger->info(
        "remesh the object {} (ID: {}) with strategy Isotropic Remeshing", object.name, object.id
    );
    logger->info("original mesh: {} vertices, {} faces", vertices.size, faces.size);
    // Compute the mean edge length.

    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions
    static const size_t iteration_limit = 5;
    set<Edge*>          selected_edges;
    for (size_t i = 0; i != iteration_limit; ++i) {
        // Split long edges.

        // Collapse short edges.

        // Flip edges.

        // Vertex averaging.
    }
    logger->info("remeshed mesh: {} vertices, {} faces\n", vertices.size, faces.size);
    global_inconsistent = true;
    validate();
}
