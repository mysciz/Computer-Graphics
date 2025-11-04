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
    /*(void)vertex_quadrics;
    optimal_pos = Vector3f(0.0f, 0.0f, 0.0f);
    cost        = 0.0f;*/
    // 获取两个端点及其二次误差矩阵
    Vertex* v1 = e->halfedge->from;
    Vertex* v2 = e->halfedge->inv->from;
    const Matrix4f& Q1 = vertex_quadrics[v1];
    const Matrix4f& Q2 = vertex_quadrics[v2];

    // 计算合并后的二次误差矩阵 Q = Q1 + Q2
    Matrix4f Q = Q1 + Q2;

    // 尝试计算最优坍缩位置 v'
    Matrix3f A = Q.block<3, 3>(0, 0); // A 是 Q 的左上角 3x3 矩阵
    Vector3f b = Q.block<3, 1>(0, 3); // b 是 Q 的右上角 3x1 向量
    // d = Q(3, 3);
    // 检查 A 是否可逆
    // Eigen 库中，Matrix::lu().isInvertible() 可以检查可逆性
    if (A.determinant() != 0) { // 简单地使用行列式判断，实际应用中可用更稳定的方法
        // 可逆：最优位置 x_opt = - A^(-1) * b
        Vector3f x_opt = A.inverse() * (-b);
        optimal_pos = x_opt;
        // 计算代价 cost = v'^T * Q * v'
        Vector4f v_opt_homo;
        v_opt_homo << x_opt, 1.0f;
        cost = v_opt_homo.transpose() * Q * v_opt_homo;

    } else {
        // 不可逆（奇异）：最优位置取 v1, v2 和中点 (v1+v2)/2 中误差最小的一个
        // 候选点
        Vector3f pos_v1 = v1->pos;
        Vector3f pos_v2 = v2->pos;
        Vector3f pos_mid = (pos_v1 + pos_v2) / 2.0f;
        
        // 齐次坐标
        Vector4f v1_homo, v2_homo, mid_homo;
        v1_homo << pos_v1, 1.0f;
        v2_homo << pos_v2, 1.0f;
        mid_homo << pos_mid, 1.0f;

        // 计算误差
        float cost_v1 = v1_homo.transpose() * Q * v1_homo;
        float cost_v2 = v2_homo.transpose() * Q * v2_homo;
        float cost_mid = mid_homo.transpose() * Q * mid_homo;

        // 选取误差最小的作为最优位置和代价
        if (cost_v1 <= cost_v2 && cost_v1 <= cost_mid) {
            optimal_pos = pos_v1;
            cost = cost_v1;
        } else if (cost_v2 <= cost_v1 && cost_v2 <= cost_mid) {
            optimal_pos = pos_v2;
            cost = cost_v2;
        } else {
            optimal_pos = pos_mid;
            cost = cost_mid;
        }
    }
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

    // 要用到的半边
    Halfedge* h=e->halfedge;
    //安全性检查
    Vertex* v1 = h->from;
    Vertex* v2 = h->inv->from;
    size_t neighbors = 0;
    // 找出 v1 的所有邻居
    std::set<Vertex*> v1_neighbors;
    Halfedge* curr_h = v1->halfedge;
    do {
        v1_neighbors.insert(curr_h->inv->from);
        curr_h = curr_h->inv->next;
    } while (curr_h != v1->halfedge);
    curr_h = v2->halfedge;
    do {
        if (v1_neighbors.count(curr_h->inv->from)) {
            neighbors++;
        }
        curr_h = curr_h->inv->next;
    } while (curr_h != v2->halfedge);
    if(neighbors!=2){
        return std::nullopt;
    }
    // 创建新顶点
    Vertex* v_new = new_vertex();
    if (!v_new) return std::nullopt;
    
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
    /* 
    optional<HalfedgeMeshFailure> check_result = validate();
    if (check_result.has_value()) {
    return std::nullopt;
    }*/
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
        if(h->edge->on_boundary())
        v->new_pos += (1.0f/8.0f)*h->inv->from->pos;
        while(next_h!= h){
            if(next_h->edge->on_boundary()){
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
                flip_edge(e);      
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
    const size_t target_face_count = faces.size / 4;
    // Step 1: 计算每个面片的二次误差矩阵 (K_f)
    logger->info("---begin computing initial quadrics for faces---");
    for (Face* f = faces.head; f != nullptr; f = f->next_node) {
        if (f->is_boundary) continue; // 跳过边界虚拟面片 
        
        // 计算面片平面方程 a*x + b*y + c*z + d = 0
        Eigen::Vector3f n = f->normal();
        Eigen::Vector3f p = f->halfedge->from->pos;
        float d = -n.dot(p);

        // 齐次坐标向量 v = (n_x, n_y, n_z, d)^T
        Eigen::Vector4f v;
        v << n, d;

        // 面片的二次误差矩阵 K_f = v * v^T (公式 3.2)
        face_quadrics[f] = v * v.transpose();
    }
    logger->info("---end computing initial quadrics for faces---");
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // Step 2: 计算每个顶点的二次误差矩阵 (K_v)
    logger->info("---begin computing initial quadrics for vertices---");
    for (Vertex* v = vertices.head; v != nullptr; v = v->next_node) {
        Matrix4f Q_v = Matrix4f::Zero();
        Halfedge* h = v->halfedge;
        Halfedge* curr_h = h;
        // 遍历所有邻接面片，累加 K_f
        do {
            Face* f = curr_h->face;
            if (!f->is_boundary) {
                Q_v += face_quadrics[f];
            }
            curr_h = curr_h->inv->next;
        } while (curr_h != h);

        vertex_quadrics[v] = Q_v;
    }
    logger->info("---end computing initial quadrics for vertices---");
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // Step 3: 构建优先队列
    logger->info("---begin building priority queue---");
    for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
        // EdgeRecord 构造函数计算 cost 和 optimal_pos
        EdgeRecord record(vertex_quadrics, e);
        edge_records[e] = record;
        edge_queue.insert(record);
    }
    logger->info("---end building priority queue---");
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.
    // Step 4: 循环坍缩
    logger->info("---begin simplification---");
    while (!edge_queue.empty() && faces.size > target_face_count) {
        // 4a: 取出 cost 最小的边
        EdgeRecord best_record = *edge_queue.begin();
        Edge* e = best_record.edge;
        edge_queue.erase(edge_queue.begin());

        // 获取要坍缩的两个顶点 v1, v2
        Vertex* v1 = e->halfedge->from;
        Vertex* v2 = e->halfedge->inv->from;

        // 收集受影响的边，并从队列中移除
        std::set<Edge*> affected_edges;
        
        // v1 邻接的边
        Halfedge* curr_h = v1->halfedge;
        do {
            affected_edges.insert(curr_h->edge);
            curr_h = curr_h->inv->next;
        } while (curr_h != v1->halfedge);
        
        // v2 邻接的边
        curr_h = v2->halfedge;
        do {
            affected_edges.insert(curr_h->edge);
            curr_h = curr_h->inv->next;
        } while (curr_h != v2->halfedge);

        // 从优先队列和记录中移除受影响的边
        for (Edge* affected_e : affected_edges) {
            auto it = edge_records.find(affected_e);
            if (it != edge_records.end()) {
                edge_queue.erase(it->second);
                edge_records.erase(it);
            }
        }

        //  坍缩边，得到新顶点 v_new
        std::optional<Vertex*> new_vertex_opt = collapse_edge(e);

        if (!new_vertex_opt.has_value()) {
            logger->warn("Edge collapse failed for edge {}", e->id);
            // 失败的边已经从队列中移除，忽略即可
            continue; 
        }

        Vertex* v_new = new_vertex_opt.value();
        
        // 确保新顶点位置是 QEM 最优位置
        v_new->pos = best_record.optimal_pos; 
        
        // 4c: 更新新顶点的二次误差矩阵 K_new = K1 + K2
        vertex_quadrics[v_new] = vertex_quadrics[v1] + vertex_quadrics[v2];
        
        // 移除被删除的顶点记录
        vertex_quadrics.erase(v1);
        vertex_quadrics.erase(v2);

        // 重新计算并插入新顶点邻接的边
        curr_h = v_new->halfedge;
        do {
            Edge* adjacent_e = curr_h->edge;
            
            // 计算新的 EdgeRecord
            EdgeRecord new_record(vertex_quadrics, adjacent_e);

            // 更新记录并插入队列
            edge_records[adjacent_e] = new_record;
            edge_queue.insert(new_record);

            curr_h = curr_h->inv->next;
        } while (curr_h != v_new->halfedge);
    }
    logger->info("---end simplification---");
    // 清理已删除元素的内存
    clear_erasure_records();
    
    logger->info("simplified mesh: {} vertices, {} faces", vertices.size, faces.size);
    logger->info("simplification done\n");
    global_inconsistent = true;
    validate();
    logger->info("Corrected mesh");
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
    float total_length = 0.0f;
    for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
    total_length += e->length();
    }
    const size_t num_edges = edges.size;
    const float L = total_length / (float)num_edges;
    logger->info("Mean edge length L = {:.4f}", L);
    // 设置分裂和坍缩阈值
    const float SPLIT_THRESHOLD = 4.0f / 3.0f * L;  // 分裂长度 > 4/3 L
    const float COLLAPSE_THRESHOLD = 4.0f / 5.0f * L; // 坍缩长度 < 4/5 L
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
        logger->info("Isotropic Remeshing Iteration {}/{}", i + 1, iteration_limit);
        logger->info("---begin Split---");
        // Split long edges.
        for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
            selected_edges.insert(e);
        }
        for (Edge* e : selected_edges) {
            // 检查边是否仍然存在，且长度大于分裂阈值
            if (e->length() > SPLIT_THRESHOLD) {
                // split_edge 会自动检查是否是三角形面片以及边界情况
                split_edge(e); 
            }
        }
        logger->info("---begin Collapse---");
        // Collapse short edges.
        selected_edges.clear();
        for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
            selected_edges.insert(e);
        }
        auto it =selected_edges.begin();
        while(it!= selected_edges.end()) {
            Edge* e=*it;
            // 检查是否满足坍缩长度条件
            if (e->length() < COLLAPSE_THRESHOLD) {
                logger->info("---begin Collapse Check---");
                // --- 坍缩后的防冲突检查：邻边长度不得大于 4/3 L ---
                Vertex* v1 = e->halfedge->from;
                Vertex* v2 = e->halfedge->inv->from;
                
                // 坍缩位置
                Eigen::Vector3f p_new = (v1->pos + v2->pos) / 2.0f; 
                
                // 找到 v1 和 v2 邻域中所有非共用的顶点 (坍缩后会成为新邻边)
                std::set<Vertex*> N_new;
                
                // 辅助函数：将一个顶点的邻居（非 v_exclude）加入集合
                auto gather_neighbors = [&](Vertex* v_center, Vertex* v_exclude) {
                    Halfedge* h_start = v_center->halfedge;
                    logger->trace("h_start->id:{}",h_start->id);
                    Halfedge* h_curr = h_start->inv->next;
                    while (h_curr != h_start) {
                        logger->trace("h_curr = {}", h_curr->id);
                        logger->trace("h_start->id:{}",h_start->id);
                        Vertex* neighbor = h_curr->inv->from;
                        if (neighbor != v_exclude && neighbor != v_center) {
                            N_new.insert(neighbor);
                        }
                        h_curr = h_curr->inv->next;
                    }
                    logger->trace("gather_neighbors done");
                };
                gather_neighbors(v1, v2);
                gather_neighbors(v2, v1);

                bool will_cause_long_edge = false;
                
                for (Vertex* n : N_new) {
                    float new_len = (p_new - n->pos).norm();
                    if (new_len > SPLIT_THRESHOLD) {
                        will_cause_long_edge = true;
                        break;
                    }
                }

                if (will_cause_long_edge) {
                    logger->trace("Edge collapse will cause long edge, skip.");
                    ++it;
                    continue; // 放弃坍缩这条边
                }
                // --- 坍缩后的防冲突检查结束 ---

                // 如果通过检查，执行坍缩
                logger->trace("Successfully collapsed edge {}.", e->id);
                
                selected_edges.erase(e->halfedge->next->edge);
                selected_edges.erase(e->halfedge->inv->next->edge);
                it=selected_edges.erase(it);
                collapse_edge(e);
            }
            else{++it;}

        }
        logger->info("---begin Flip---");
        // Flip edges.
        selected_edges.clear();
        for (Edge* e = edges.head; e != nullptr; e = e->next_node) {
            selected_edges.insert(e);
        }

        for (Edge* e : selected_edges) {
            if (e->halfedge == nullptr || e->on_boundary()) continue;

            Halfedge* h = e->halfedge;
            Halfedge* inv_h = h->inv;
            
            // 确保是三角形面 (否则翻转的拓扑结构不确定)
            if (h->next->next->next != h || inv_h->next->next->next != inv_h) continue;

            Vertex* v1 = h->from;
            Vertex* v2 = inv_h->from;
            Vertex* v3 = h->next->next->from;  // 对向 h 的顶点
            Vertex* v4 = inv_h->next->next->from; // 对向 inv_h 的顶点

            // 计算度数差指标 d = sum(|degree(vi) - 6|)
            auto degree_diff = [](size_t deg) -> float {
                return static_cast<float>(std::abs(static_cast<int>(deg) - 6));
            };
            
            float d_before = degree_diff(v1->degree()) + degree_diff(v2->degree()) +
                             degree_diff(v3->degree()) + degree_diff(v4->degree());

            // 翻转后，v1, v2 的度数减 1；v3, v4 的度数加 1
            float d_after = degree_diff(v1->degree() - 1) + degree_diff(v2->degree() - 1) +
                            degree_diff(v3->degree() + 1) + degree_diff(v4->degree() + 1);
            
            // 如果翻转可以减小总的度数偏差，则执行翻转。
            if (d_after < d_before) {
                flip_edge(e);
            }
        }
        logger->info("---begin Smoothing---");
        // Vertex averaging.
        const float w = 1.0f / 5.0f; // 缩放因子 w = 1/5
        for (Vertex* v=vertices.head; v!= nullptr; v = v->next_node) {
            // 获取邻域中心 C (拉普拉斯平滑的中心)
            Eigen::Vector3f c = v->neighborhood_center();
            Eigen::Vector3f p = v->pos;
            
            // 拉普拉斯向量：v_vec = c - p
            Eigen::Vector3f v_vec = c - p;
            
            // 顶点法向量 N
            Eigen::Vector3f normal = v->normal();
            normal.normalize(); 
            
            // 沿切线方向的移动向量：v_tan_move = v_vec - (v_vec . N) N
            Eigen::Vector3f v_tan_move = v_vec - (v_vec.dot(normal)) * normal;
            
            // 最终的新位置：p + w * v_tan_move
            v->new_pos = p + w * v_tan_move;
        }

        // 步骤 2: 统一更新位置
        for (Vertex* v=vertices.head; v!= nullptr; v = v->next_node) {
            v->pos = v->new_pos;
        }
    }
    logger->info("remeshed mesh: {} vertices, {} faces\n", vertices.size, faces.size);
    global_inconsistent = true;
    validate();
}
