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
    /*logger->trace("---startflippingedge {}---",e->id);
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
    logger->trace("---end---");*/
    /*optional<HalfedgeMeshFailure> check_result = validate();
    if (check_result.has_value()) {
    return std::nullopt;
    }*/
    return e;
}

optional<Vertex*> HalfedgeMesh::split_edge(Edge* e)
{

    // 创建新顶点
    Vertex* v_new = new_vertex();
    if (!v_new) return std::nullopt;
    // 要用到的半边
    Halfedge* h_1_n = e->halfedge;
    
    if(!h_1_n->is_boundary()&&h_1_n->inv->is_boundary()){
        logger->info("---not is boundary---",h_1_n->id);
        h_1_n = h_1_n->inv;
    }
    Halfedge* h_2_n = h_1_n->inv;
    Halfedge* h_2_3 = h_1_n->next;
    Halfedge* h_3_1 = h_2_3->next;
    Halfedge* h_1_4 = h_2_n->next;
    Halfedge* h_4_2 = h_1_4->next;
    //logger->trace("---h_1_n->id:{},h_2_n->id:{},h_3_1->id:{},h_4_2->id:{}", h_1_n->id,h_2_n->id,h_3_1->id,h_4_2->id);    
    // 要用到的顶点
    Vertex* v1 = h_1_n->from;
    Vertex* v2 = h_2_n->from;
    Vertex* v3 = h_3_1->from;
    Vertex* v4 = h_4_2->from;
    // 要用到的面片
    Face* f1_3 = h_1_n->face;
    Face* f2_4 = h_2_n->face;
    // 重新连接各基本元素
   
    Halfedge* h_n_4 = new_halfedge();
    
    Halfedge* h_4_n = new_halfedge();
    Halfedge* h_n_1 = new_halfedge();
    Halfedge* h_n_2 = new_halfedge();
    logger->trace("---h_n_1->id:{},h_n_2->id:{},h_n_4->id:{}", h_n_1->id,h_n_2->id,h_n_4->id);
    
    Face* f1_4 = new_face();
    Edge* e2_n = new_edge();
    
    Edge* e4_n = new_edge();
    //logger->trace("---e3_n->id:{},e4_n->id:{}", e3_n->id,e4_n->id);
    v_new->pos=(v1->pos+v2->pos)/2.0f;
    v_new->halfedge = h_n_1;
    //设置新建元素
    if(e->on_boundary()){
    if(h_1_n->is_boundary())
    logger->trace("h->from->id:{},h->next->inv->id:{},h->inv->from->id:{},v_new->id:{}",h_1_n->from->id,h_1_n->next->inv->from->id,h_1_n->inv->from->id,v_new->id);
    h_n_1->set_neighbors(h_1_4,h_4_n,h_1_n,v_new,e,f1_4);
    h_4_n->set_neighbors(h_n_1,h_1_4,h_n_4,v4,e4_n,f1_4);
    h_n_4->set_neighbors(h_4_2,h_2_n,h_4_n,v_new,e4_n,f2_4);
    h_2_n->set_neighbors(h_n_4,h_4_2,h_n_2,v2,e2_n,f2_4);
    f2_4->halfedge = h_4_2;
    f1_4->halfedge = h_1_4;
    h_1_4->next = h_4_n;
    h_1_4->prev = h_n_1;
    h_1_4->face = f1_4;
    h_4_2->next = h_2_n;
    h_4_2->prev = h_n_4;
    h_4_2->face = f2_4;
    e4_n->halfedge=h_n_4;
    e2_n->halfedge=h_n_2;
    h_1_n->next=h_n_2;
    h_1_n->inv=h_n_1;
    h_2_3->prev=h_n_2;
    h_n_2->set_neighbors(h_2_3,h_1_n,h_2_n,v_new,e2_n,f1_3);
    f1_3->halfedge=h_3_1;
    e->halfedge=h_1_n;
    logger->trace("h_1_n->id:{},h_n_1->id:{},h_n_2->id:{},h_2_n->id:{},h_n_4->id:{},h_4_2->id:{},h_4_n->id:{},h_1_4->id:{},h_2_3->id:{},h_3_1->id:{}", h_1_n->id,h_n_1->id,h_n_2->id,h_2_n->id,h_n_4->id,h_4_2->id,h_4_n->id,h_1_4->id,h_2_3->id,h_3_1->id);
    logger->trace("e1_n->id:{},e2_n->id:{},e4_n->id:{},e2_3->id:{}", e->id,e2_n->id,e4_n->id,h_2_3->edge->id);

    return v_new;
    }
    Edge* e3_n = new_edge();
    Halfedge* h_3_n = new_halfedge();
    Halfedge* h_n_3 = new_halfedge();
    Face* f2_3 = new_face();
    e->halfedge=h_1_n;
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
    if(!h->is_boundary())//
        h = h->inv;
    Halfedge* h_inv=h->inv;
    Halfedge* h_inv1=h->next->inv;
    Halfedge* h_inv2=h_inv->next->inv;
    Halfedge* h1=h->prev->inv;
    Halfedge* h2=h_inv->prev->inv;
    logger->trace("---h->id:{},h_inv->id:{},h_inv1->id:{},h_inv2->id:{},h1->id:{},h2->id:{}", h->id,h_inv->id,h_inv1->id,h_inv2->id,h1->id,h2->id);
    logger->trace("h->prev->id:{},h->next->id:{},h_inv->prev->id:{},h_inv->next->id:{}", h->prev->id,h->next->id,h_inv->prev->id,h_inv->next->id);
    logger->trace("h->edge->id:{},h1->edge->id:{},h2->edge->id:{},h_inv1->edge->id:{},h_inv2->edge->id:{}", h->edge->id,h1->edge->id,h2->edge->id,h_inv1->edge->id,h_inv2->edge->id);
    logger->trace("h->from->id:{},h->next->inv->from->id:{},h->prev->from->id:{}",h->from->id,h->next->inv->from->id,h->prev->from->id);
    h_inv2->from->halfedge=h_inv2;
    h_inv1->from->halfedge=h_inv1;
    h2->edge->halfedge=h2;
    h1->edge->halfedge=h1;
    h_inv2->edge=h2->edge;
    
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
    h_inv1->edge=h1->edge;
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
    h->prev->next=h->next;
    h->next->prev=h->prev;//虚面调整
    erase(h->from);
    erase(h_inv->from);
    erase(h_inv->next->edge);
    erase(h_inv->next);
    erase(h_inv->prev);
    erase(h->edge);
    erase(h_inv->face);
    erase(h);
    erase(h_inv);
    }
    optional<HalfedgeMeshFailure> check_result = validate();
    if (check_result.has_value()) {
    return std::nullopt;
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
    logger->info("original mesh: {} vertices, {} faces in total,{} edges", vertices.size, faces.size,edges.size);
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
    // Step 1: Compute new positions for all the original vertices in the input mesh using the Loop subdivision rule
    logger->info("---begin computing new positions for original vertices---");
    for (Vertex* v = vertices.head; v != nullptr; v = v->next_node) {
        // Formula 2.2: Compute new position for vertex v
        bool is_boundary = false;//记录是否为边界顶点
        Halfedge* h = v->halfedge;
        Halfedge* next_h = h->inv->next;
        v->new_pos = (3.0f/4.0f)*v->pos;
        while(next_h!= h){
            if(h->edge->on_boundary()){
                v->new_pos += (1.0f/8.0f)*next_h->inv->from->pos;
                is_boundary=true;
            }
            next_h=next_h->inv->next;
        }
        if(!is_boundary){//不在边上使用公式2.2
        float u;
        if (v->degree() == 3) {
            u = 3.0f / 16.0f;
        } else {
            u = 3.0f / (8.0f * v->degree());
        }
        v->new_pos = (1 - v->degree() * u) * v->pos + u *v->degree() * v->neighborhood_center();
        v->is_new = false; // Mark as old vertex
        }
    }
    check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    logger->info("---end computing new positions for original vertices---");
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.
    // Step 2: Compute the subdivided vertex positions associated with edges, and store them in Edge::new_pos
    logger->info("---begin computing new positions for edges---");
    for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
        Eigen::Vector3f v1 = e->halfedge->from->pos;
        Eigen::Vector3f v2 = e->halfedge->inv->from->pos;
        if (!e->is_new) { // Only process original edges
            if(e->on_boundary()){
            e->new_pos=(1.0f/2.0f)*(v1+v2);//在边上使用公式2.3
            }
            else{
            Eigen::Vector3f v3 = e->halfedge->face->center();
            Eigen::Vector3f v4 = e->halfedge->inv->face->center();
            Eigen::Vector3f new_pos = (1.0f / 8.0f) * (v1 + v2) + (3.0f / 8.0f) * (v3 + v4);//不在边上使用公式2.1 但是这里的v3 v4使用三角形的中心 修改了公式2.1
            e->new_pos = new_pos;
            }
        }
    }
    check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    logger->info("---end computing new positions for edges---");
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
    logger->info("---begin splitting edges---");
    vector<Edge*> original_edges;
    //int count=0;
    for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
            original_edges.push_back(e);
            //logger->trace("push edge {}", e->id);
    }
    for (Edge* e : original_edges) {
        if (!e->is_new) { // Only process original edges
            //logger->trace("split edge {}", e->id);
            std::optional<Vertex*> new_vertex = split_edge(e);
            //count++;
            //logger->trace("count:{}",count);
            /*check_result = validate();
            if (check_result.has_value()) {
            logger->error("split edge {} failed", e->id);
                return;
            }*/
            Vertex* v=new_vertex.value();
            v->is_new=true;//标记新边和新顶点
            if(e->on_boundary()){
                e->halfedge->inv->prev->edge->is_new=true;
            }
            else{
            e->halfedge->inv->prev->edge->is_new=true;
            e->halfedge->next->edge->is_new=true;
            //logger->trace("e3->id:{},e4->id:{}", e->halfedge->next->edge->id, e->halfedge->inv->prev->edge->id);
            }
            v->pos=e->new_pos;//顺手修改新顶点的坐标
        }
    }
    logger->info("---end splitting edges---");
    check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    // Now flip any new edge that connects an old and new vertex.
    logger->info("---begin flipping edges---");
    for (Edge* e = edges.head; e != nullptr; e = e->next_node){
         if (e->is_new) { // Only process new edges
            //logger->trace("check edge {}", e->id);
            Vertex* v1 = e->halfedge->from;
            Vertex* v2 = e->halfedge->inv->from;
            if ((!v1->is_new && v2->is_new) || (v1->is_new && !v2->is_new)) {
                optional<Edge*>flipped_edge =flip_edge(e);      
            }
        }
        if(e->next_node==nullptr){
            logger->trace("check edge end");
        }
    }
    logger->info("---end flipping edges---");
    check_result = validate();
    if (check_result.has_value()) {
        return;
    }
    // Finally, copy new vertex positions into the Vertex::pos.
    logger->info("---begin copying new vertex positions into old vertices---");
    for(Vertex* v = vertices.head; v != nullptr; v = v->next_node){
        if(!v->is_new)
        v->pos=v->new_pos;//修改旧顶点的坐标
    }
    logger->info("---end copying new vertex positions into old vertices---");
    //将所有的属性重置
    for(Edge* e = edges.head; e != nullptr; e = e->next_node){
        e->is_new=false;
    }
    for(Vertex* v = vertices.head; v != nullptr; v = v->next_node){
        v->is_new=false;
    }
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
