#include "object.h"

#include <array>
#include <optional>

#ifdef _WIN32
    #include <Windows.h>
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fmt/format.h>

#include "../utils/math.hpp"
#include "../utils/ray.h"
#include "../simulation/solver.h"
#include "../utils/logger.h"

using Eigen::Matrix4f;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using std::array;
using std::make_unique;
using std::optional;
using std::string;
using std::vector;

bool   Object::BVH_for_collision = false;
size_t Object::next_available_id = 0;
std::function<KineticState(const KineticState&, const KineticState&)> Object::step =
    forward_euler_step;

Object::Object(const string& object_name) :
    name(object_name), center(0.0f, 0.0f, 0.0f), scaling(1.0f, 1.0f, 1.0f),
    rotation(1.0f, 0.0f, 0.0f, 0.0f), velocity(0.0f, 0.0f, 0.0f), force(0.0f, 0.0f, 0.0f),
    mass(1.0f), BVH_boxes("BVH", GL::Mesh::highlight_wireframe_color)
{
    visible  = true;
    modified = false;
    id       = next_available_id;
    ++next_available_id;
    bvh                      = make_unique<BVH>(mesh);
    const string logger_name = fmt::format("{} (Object ID: {})", name, id);
    logger                   = get_logger(logger_name);
}

Matrix4f Object::model()
{
    const Quaternionf& r = rotation;
    auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());
    // Construct the rotation matrix with euler angles.
    x_angle = radians(x_angle);
    y_angle = radians(y_angle);
    z_angle = radians(z_angle);
    Matrix4f x_rotation_matrix = Matrix4f::Identity();
    x_rotation_matrix(1, 1) = std::cos(x_angle);
    x_rotation_matrix(1, 2) = -std::sin(x_angle);
    x_rotation_matrix(2, 1) = std::sin(x_angle);
    x_rotation_matrix(2, 2) = std::cos(x_angle);
    Matrix4f y_rotation_matrix = Matrix4f::Identity();
    y_rotation_matrix(0, 0) = std::cos(y_angle);
    y_rotation_matrix(0, 2) = std::sin(y_angle);
    y_rotation_matrix(2, 0) = -std::sin(y_angle);
    y_rotation_matrix(2, 2) = std::cos(y_angle);
    Matrix4f z_rotation_matrix = Matrix4f::Identity();
    z_rotation_matrix(0, 0) = std::cos(z_angle);
    z_rotation_matrix(0, 1) = -std::sin(z_angle);
    z_rotation_matrix(1, 0) = std::sin(z_angle);
    z_rotation_matrix(1, 1) = std::cos(z_angle);
    Matrix4f rotation_matrix = x_rotation_matrix * y_rotation_matrix * z_rotation_matrix;
     // 构建平移矩阵
    Eigen::Matrix4f translation_matrix = Eigen::Matrix4f::Identity();
    translation_matrix(0, 3) = center.x();
    translation_matrix(1, 3) = center.y();
    translation_matrix(2, 3) = center.z();
     // 构建缩放矩阵
    Eigen::Matrix4f scaling_matrix = Eigen::Matrix4f::Identity();
    scaling_matrix(0, 0) = scaling.x();
    scaling_matrix(1, 1) = scaling.y();
    scaling_matrix(2, 2) = scaling.z(); 
    // 构建模型矩阵
    Matrix4f model_matrix =  translation_matrix*rotation_matrix*scaling_matrix ;
    return model_matrix;
    

}

void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    (void)next_state;
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。
    for (auto object: all_objects) {
        (void)object;

        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            if (BVH_for_collision) {
            } else {
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
}

void Object::render(const Shader& shader, WorkingMode mode, bool selected)
{
    if (modified) {
        mesh.VAO.bind();
        mesh.vertices.to_gpu();
        mesh.normals.to_gpu();
        mesh.edges.to_gpu();
        mesh.edges.release();
        mesh.faces.to_gpu();
        mesh.faces.release();
        mesh.VAO.release();
    }
    modified = false;
    // Render faces anyway.
    unsigned int element_flags = GL::Mesh::faces_flag;
    if (mode == WorkingMode::MODEL) {
        // For *Model* mode, only the selected object is rendered at the center in the world.
        // So the model transform is the identity matrix.
        shader.set_uniform("model", I4f);
        shader.set_uniform("normal_transform", I4f);
        element_flags |= GL::Mesh::vertices_flag;
        element_flags |= GL::Mesh::edges_flag;
    } else {
        Matrix4f model = this->model();
        shader.set_uniform("model", model);
        shader.set_uniform("normal_transform", (Matrix4f)(model.inverse().transpose()));
    }
    // Render edges of the selected object for modes with picking enabled.
    if (check_picking_enabled(mode) && selected) {
        element_flags |= GL::Mesh::edges_flag;
    }
    mesh.render(shader, element_flags);
}

void Object::rebuild_BVH()
{
    bvh->recursively_delete(bvh->root);
    bvh->build();
    BVH_boxes.clear();
    refresh_BVH_boxes(bvh->root);
    BVH_boxes.to_gpu();
}

void Object::refresh_BVH_boxes(BVHNode* node)
{
    if (node == nullptr) {
        return;
    }
    BVH_boxes.add_AABB(node->aabb.p_min, node->aabb.p_max);
    refresh_BVH_boxes(node->left);
    refresh_BVH_boxes(node->right);
}
