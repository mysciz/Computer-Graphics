#include "rasterizer_renderer.h"
#include "../utils/math.hpp"
#include <cstdio>

#ifdef _WIN32
    #undef min
    #undef max
#endif

using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex shader
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;
    Eigen::Matrix4f M = (Uniforms::inv_trans_M .transpose()).inverse();
    output_payload.world_position =  M* payload.world_position;
    // 应用MVP变换将顶点变换到裁剪空间
    Vector4f clip_position = Uniforms::MVP * payload.world_position;
    
    // 透视除法（齐次坐标归一化）
    Vector4f ndc_position = clip_position / clip_position.w();
    
    // 视口变换：将NDC坐标变换到屏幕坐标
    output_payload.viewport_position.x() = (ndc_position.x() + 1.0f) * 0.5f * Uniforms::width;
    output_payload.viewport_position.y() = (ndc_position.y() + 1.0f) * 0.5f * Uniforms::height;
    output_payload.viewport_position.z() = ndc_position.z();
    output_payload.viewport_position.w() = 1.0f / clip_position.w(); // 存储1/w用于透视矫正
    
    // 将法线变换到世界坐标系
    Vector4f normal_homogeneous = Uniforms::inv_trans_M * Vector4f(payload.normal.x(), payload.normal.y(), payload.normal.z(), 0.0f);
    output_payload.normal = normal_homogeneous.head<3>().normalized();
    return output_payload;
}

Vector3f phong_fragment_shader(
    const FragmentShaderPayload& payload, const GL::Material& material,
    const std::list<Light>& lights, const Camera& camera
)
{
      Vector3f result = {0, 0, 0};
    
    // 假设有一个全局环境光强度
    float global_ambient_intensity = 0.1f;  // 或者从场景中获取
    Vector3f ambient = material.ambient * global_ambient_intensity;
    
    Vector3f diffuse_sum = Vector3f::Zero();
    Vector3f specular_sum = Vector3f::Zero();
    
    for (const auto& light : lights) {
        // Light Direction
        Vector3f light_dir = (light.position - payload.world_pos).normalized();
        
        // View Direction  
        Vector3f view_dir = (camera.position - payload.world_pos).normalized();
        
        // Half Vector
        Vector3f half_vec = (light_dir + view_dir).normalized();
        
        // Light Attenuation
        float distance = (light.position - payload.world_pos).norm();
        float attenuation = 1.0f / (distance * distance);
        Vector3f light_color = light.intensity * attenuation * Vector3f(1, 1, 1);
        
        // Diffuse
        float NdotL = std::max(payload.world_normal.dot(light_dir), 0.0f);
        Vector3f diffuse = material.diffuse.cwiseProduct(light_color) * NdotL;
        diffuse_sum += diffuse;
        
        // Specular
        float NdotH = std::max(payload.world_normal.dot(half_vec), 0.0f);
        Vector3f specular = material.specular.cwiseProduct(light_color) * std::pow(NdotH, material.shininess);
        specular_sum += specular;
    }
    
    result = ambient + diffuse_sum + specular_sum;
    result = result.cwiseMax(0.0f).cwiseMin(1.0f);
    return result * 255.0f;
}