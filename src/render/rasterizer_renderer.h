#ifndef DANDELION_RENDER_RASTERIZER_RENDERER_H
#define DANDELION_RENDER_RASTERIZER_RENDERER_H

#include <list>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../platform/gl.hpp"
#include "../scene/light.h"
#include "../scene/camera.h"
#include "graphics_interface.h"

/*!
 * \file render/rasterizer_renderer.h
 * \ingroup rendering
 * \~chinese
 * \brief 光栅化渲染器中顶点处理、片元处理两个阶段的实现。
 */
class VertexProcessor
{
public:
    /*!
     * \~chinese
     * \brief 输入顶点数据到顶点处理队列中
     *
     * \param positions 顶点位置坐标
     * \param normals 顶点法线向量
     */
    void input_vertices(const Eigen::Vector4f& positions, const Eigen::Vector3f& normals, 
                       int triangle_id, int vertex_in_triangle);
    
    /*!
     * \~chinese
     * \brief 顶点着色器函数指针，指向用于处理顶点数据的着色器函数
     * \param payload 待着色的顶点数据
     * 
     * \return 着色后的顶点数据
     */
    VertexShaderPayload (*vertex_shader_ptr)(const VertexShaderPayload& payload);
    
    /*!
     * \~chinese
     * \brief 负责执行顶点着色器的工作线程
     * 
     * 不断读取顶点队列中的顶点数据，执行顶点着色器，并将结果存储到顶点着色输出队列中
     * 
     */
    void worker_thread();

private:
    /*! \~chinese 存储待着色顶点数据的队列，包含三角形ID和顶点位置信息 */
    std::queue<std::tuple<VertexShaderPayload, int, int>> vertex_queue;
    /*! \~chinese 保护顶点队列的互斥锁 */
    std::mutex queue_mutex;
    /*! \~chinese 用于按三角形分组存储处理后的顶点 */
    std::map<int, std::array<VertexShaderPayload, 3>> processed_triangles;
    /*! \~chinese 保护已处理三角形映射的互斥锁 */
    std::mutex triangles_mutex;
};

/*!
 * \ingroup rendering
 * \~chinese
 * \brief 负责执行片元着色器的工作线程。
 */
class FragmentProcessor
{
public:
    /*!
     * \~chinese
     * \brief 负责执行片元着色器的工作线程
     * 
     * 不断读取光栅化输出队列中的顶点数据，执行片元着色器，计算并设置像素颜色
     * 
     */
    void worker_thread();

    /*!
     * \~chinese
     * \brief 片元着色器函数指针，指向用于计算片元颜色的着色器函数
     * \param payload 片元数据
     * \param material 材质属性
     * \param lights 场景中的光源列表
     * \param camera 场景相机
     * 
     * \return 计算得到的片元RGB颜色
     */
    Eigen::Vector3f (*fragment_shader_ptr)(
        const FragmentShaderPayload& payload, const GL::Material& material,
        const std::list<Light>& lights, const Camera& camera
    );

private:
};

#endif // DANDELION_RENDER_RASTERIZER_RENDERER_H
