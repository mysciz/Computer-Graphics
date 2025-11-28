#include <array>
#include <limits>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "rasterizer.h"
#include "triangle.h"
#include "../utils/math.hpp"

using Eigen::Matrix4f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Vector4f;
using std::fill;
using std::tuple;

void Rasterizer::worker_thread()
{
    while (!Context::rasterizer_finish) {
        VertexShaderPayload payload;
        Triangle            triangle;
        {
            if (Context::vertex_finish && Context::vertex_shader_output_queue.empty()) {
                Context::rasterizer_finish = true;
                return;
            }
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            std::unique_lock<std::mutex> lock(Context::vertex_queue_mutex);
            if (Context::vertex_shader_output_queue.size() < 3) {
                continue;
            }
            for (size_t vertex_count = 0; vertex_count < 3; vertex_count++) {
                payload = Context::vertex_shader_output_queue.front();
                Context::vertex_shader_output_queue.pop();
                if (vertex_count == 0) {
                    triangle.world_pos[0]    = payload.world_position;
                    triangle.viewport_pos[0] = payload.viewport_position;
                    triangle.normal[0]       = payload.normal;
                } else if (vertex_count == 1) {
                    triangle.world_pos[1]    = payload.world_position;
                    triangle.viewport_pos[1] = payload.viewport_position;
                    triangle.normal[1]       = payload.normal;
                } else {
                    triangle.world_pos[2]    = payload.world_position;
                    triangle.viewport_pos[2] = payload.viewport_position;
                    triangle.normal[2]       = payload.normal;
                }
            }
        }
        rasterize_triangle(triangle);
    }
}

float sign(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3)
{
    return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，判断(x,y)是否在三角形的内部
bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{
    // 将像素中心坐标转换为浮点数
    float xf = x + 0.0f;
    float yf = y + 0.0f;
    
    // 获取三角形的三个顶点（转换为2D坐标）
    Eigen::Vector2f p(xf, yf);
    Eigen::Vector2f a(vertices[0].x(), vertices[0].y());
    Eigen::Vector2f b(vertices[1].x(), vertices[1].y());
    Eigen::Vector2f c(vertices[2].x(), vertices[2].y());
    
    // 计算三个子三角形的面积（有向面积）
    float d1 = sign(p, a, b);
    float d2 = sign(p, b, c);
    float d3 = sign(p, c, a);
    
    // 判断三个有向面积是否同号（都在三角形同一侧）
    bool has_neg = (d1 <= 0) && (d2 <= 0) && (d3 <= 0);
    bool has_pos = (d1 > 0) && (d2 > 0) && (d3 > 0);
    
    // 如果既有正又有负，说明点在三角形外部
    return (has_neg || has_pos);
}

// 给定坐标(x,y)以及三角形的三个顶点坐标，计算(x,y)对应的重心坐标[alpha, beta, gamma]
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* v)
{
  // 将顶点坐标转换为2D
    Eigen::Vector3f A(v[0].x(), v[0].y(), 1.0f);
    Eigen::Vector3f B(v[1].x(), v[1].y(), 1.0f);
    Eigen::Vector3f C(v[2].x(), v[2].y(), 1.0f);
    Eigen::Vector3f P(x, y, 1.0f);
    
    // 计算重心坐标
    Eigen::Vector3f v0 = B - A;
    Eigen::Vector3f v1 = C - A;
    Eigen::Vector3f v2 = P - A;
    
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);
    float denom = d00 * d11 - d01 * d01;
    
    float beta = (d11 * d20 - d01 * d21) / denom;
    float gamma = (d00 * d21 - d01 * d20) / denom;
    float alpha = 1.0f - beta - gamma;
    
    return std::make_tuple(alpha, beta, gamma);
}

// 对顶点的某一属性插值
Vector3f Rasterizer::interpolate(
    float alpha, float beta, float gamma, const Eigen::Vector3f& vert1,
    const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, const Eigen::Vector3f& weight,
    const float& Z
)
{
    Vector3f interpolated_res;
    for (int i = 0; i < 3; i++) {
        interpolated_res[i] = alpha * vert1[i] / weight[0] + beta * vert2[i] / weight[1]
                            + gamma * vert3[i] / weight[2];
    }
    interpolated_res *= Z;
    return interpolated_res;
}

// 对当前三角形进行光栅化
void Rasterizer::rasterize_triangle(Triangle& t)
{
   // 获取三角形的边界框
    float min_x = std::min({t.viewport_pos[0].x(), t.viewport_pos[1].x(), t.viewport_pos[2].x()});
    float max_x = std::max({t.viewport_pos[0].x(), t.viewport_pos[1].x(), t.viewport_pos[2].x()});
    float min_y = std::min({t.viewport_pos[0].y(), t.viewport_pos[1].y(), t.viewport_pos[2].y()});
    float max_y = std::max({t.viewport_pos[0].y(), t.viewport_pos[1].y(), t.viewport_pos[2].y()});
    
    // 将边界框限制在屏幕范围内
    int start_x = std::max(0, static_cast<int>(std::floor(min_x)));
    int end_x = std::min(Uniforms::width - 1, static_cast<int>(std::ceil(max_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(min_y)));
    int end_y = std::min(Uniforms::height - 1, static_cast<int>(std::ceil(max_y)));
    
    // 遍历边界框内的所有像素
    for (int y = start_y; y <= end_y; y++) {
        for (int x = start_x; x <= end_x; x++) {
            if (inside_triangle(x, y, t.viewport_pos)) {
                // 计算重心坐标
                auto [alpha, beta, gamma] = compute_barycentric_2d(
                    x + 0.0f, y + 0.0f, t.viewport_pos);
                
                // 透视矫正插值所需的权重
                Vector3f weight = {
                    t.viewport_pos[0].w(),
                    t.viewport_pos[1].w(), 
                    t.viewport_pos[2].w()
                };
                
                // 插值深度值（需要透视矫正）
                float depth = alpha * t.viewport_pos[0].z() / weight[0] 
                            + beta * t.viewport_pos[1].z() / weight[1]
                            + gamma * t.viewport_pos[2].z() / weight[2];
                float Z = 1.0f / (alpha / weight[0] + beta / weight[1] + gamma / weight[2]);
                depth *= Z;
                
                // 插值世界坐标（透视矫正）
                Vector3f world_pos = interpolate(
                    alpha, beta, gamma,
                    t.world_pos[0].head<3>(), t.world_pos[1].head<3>(), t.world_pos[2].head<3>(),
                    weight, Z
                );
                
                // 插值法线向量（透视矫正）
                Vector3f world_normal = interpolate(
                    alpha, beta, gamma,
                    t.normal[0], t.normal[1], t.normal[2],
                    weight, Z
                );
                
                // 创建片元数据
                FragmentShaderPayload payload;
                payload.world_pos = world_pos;
                payload.world_normal = world_normal.normalized();
                payload.x = x;
                payload.y = y;
                payload.depth = depth;
                payload.color = Vector3f(0, 0, 0); // 初始颜色，将在片元着色器中计算
                
                // 将片元加入队列
                std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
                Context::rasterizer_output_queue.push(payload);
            }
        }
    }
    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. push primitive into fragment queue
    //std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
    //Context::rasterizer_output_queue.push(payload);
}
