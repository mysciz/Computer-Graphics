#include <cstddef>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "render_engine.h"
#include "../scene/light.h"
#include "../utils/logger.h"

using std::chrono::steady_clock;
using std::size_t;
using duration   = std::chrono::duration<float>;
using time_point = std::chrono::time_point<steady_clock, duration>;
using Eigen::Vector3f;
using Eigen::Vector4f;

// vertex processor & rasterizer & fragement processor can visit
// all the static variables below from Uniforms structure
Eigen::Matrix4f Uniforms::MVP;
Eigen::Matrix4f Uniforms::inv_trans_M;
int             Uniforms::width  = 0;
int             Uniforms::height = 0;

GL::Material     ini_material = GL::Material();
std::list<Light> ini_lights   = {};
Camera           ini_camera = Camera(Vector3f::Ones(), Vector3f::Ones(), 0.1f, 10.0f, 45.0f, 1.33f);

GL::Material&     Uniforms::material = ini_material;
std::list<Light>& Uniforms::lights   = ini_lights;
Camera&           Uniforms::camera   = ini_camera;

std::mutex                        Context::vertex_queue_mutex;
std::mutex                        Context::rasterizer_queue_mutex;
std::queue<VertexShaderPayload>   Context::vertex_shader_output_queue;
std::queue<FragmentShaderPayload> Context::rasterizer_output_queue;

volatile bool Context::vertex_finish     = false;
volatile bool Context::rasterizer_finish = false;
volatile bool Context::fragment_finish   = false;

FrameBuffer Context::frame_buffer(Uniforms::width, Uniforms::height);

FrameBuffer::FrameBuffer(int width, int height) :
    width(width), height(height), color_buffer(width * height, Eigen::Vector3f(0, 0, 0)),
    depth_buffer(width * height, std::numeric_limits<float>::infinity()), spin_locks(width * height)
{
    for (auto& lock: spin_locks) {
        lock.unlock();
    }
}

// 光栅化渲染器的构造函数
RasterizerRenderer::RasterizerRenderer(
    RenderEngine& engine, int num_vertex_threads, int num_rasterizer_threads,
    int num_fragment_threads
) :
    width(engine.width), height(engine.height), n_vertex_threads(num_vertex_threads),
    n_rasterizer_threads(num_rasterizer_threads), n_fragment_threads(num_fragment_threads),
    vertex_processor(), rasterizer(), fragment_processor(), rendering_res(engine.rendering_res)
{
    logger = get_logger("Rasterizer Renderer");
}
void RasterizerRenderer::setnum(int num)
{   num--;
    if(num<6){
        n_vertex_threads=num/3;
        n_fragment_threads=num/3;
        n_rasterizer_threads=num-2*num/3;
    }
    else{
        n_vertex_threads=4;
        n_fragment_threads=1;
        n_rasterizer_threads=1;
    }
    return;
}
// 光栅化渲染器的渲染调用接口
void RasterizerRenderer::render(const Scene& scene)
{
    Uniforms::width       = static_cast<int>(width);
    Uniforms::height      = static_cast<int>(height);
    Context::frame_buffer = FrameBuffer(Uniforms::width, Uniforms::height);
    // clear Color Buffer & Depth Buffer & rendering_res
    Context::frame_buffer.clear(BufferType::Color | BufferType::Depth);
    this->rendering_res.clear();
    // run time statistics
    time_point begin_time                  = steady_clock::now();
    Camera     cam                         = scene.camera;
    vertex_processor.vertex_shader_ptr     = vertex_shader;
    fragment_processor.fragment_shader_ptr = phong_fragment_shader;
    logger->info("n_vertex_threads: {}, n_rasterizer_threads: {}, n_fragment_threads: {}",n_vertex_threads,n_rasterizer_threads,n_fragment_threads);
     for (const auto& group: scene.groups) {
        for (const auto& object: group->objects) {
            Context::vertex_finish     = false;
            Context::rasterizer_finish = false;
            Context::fragment_finish   = false;

            std::vector<std::thread> workers;
            for (int i = 0; i < n_vertex_threads; ++i) {
                workers.emplace_back(&VertexProcessor::worker_thread, &vertex_processor);
            }
            for (int i = 0; i < n_rasterizer_threads; ++i) {
                workers.emplace_back(&Rasterizer::worker_thread, &rasterizer);
            }
            for (int i = 0; i < n_fragment_threads; ++i) {
                workers.emplace_back(&FragmentProcessor::worker_thread, &fragment_processor);
            }

            // set Uniforms for vertex shader
            Uniforms::MVP         = cam.projection() * cam.view() * object->model();
            Uniforms::inv_trans_M = object->model().inverse().transpose();
            Uniforms::width       = static_cast<int>(this->width);
            Uniforms::height      = static_cast<int>(this->height);
            Uniforms::material    = object->mesh.material;
            Uniforms::lights      = scene.lights;
            Uniforms::camera      = scene.camera;

            // input object->mesh's vertices & faces & normals data
            const std::vector<float>&        vertices  = object->mesh.vertices.data;
            const std::vector<unsigned int>& faces     = object->mesh.faces.data;
            const std::vector<float>&        normals   = object->mesh.normals.data;
            size_t                           num_faces = faces.size();

            // process vertices - 按三角形输入，保持顺序
            int triangle_id = 0;
            for (size_t i = 0; i < num_faces; i += 3) {
                for (int j = 0; j < 3; j++) {  // 改为 int
                    size_t idx = faces[i + j];
                    vertex_processor.input_vertices(
                        Vector4f(
                            vertices[3 * idx], vertices[3 * idx + 1], vertices[3 * idx + 2], 1.0f
                        ),
                        Vector3f(normals[3 * idx], normals[3 * idx + 1], normals[3 * idx + 2]),
                        triangle_id,  // 三角形ID
                        j             // 顶点在三角形中的位置(0,1,2)
                    );
                }
                triangle_id++;
            }
            
            // 发送结束信号给所有顶点处理线程
            for (int i = 0; i < n_vertex_threads; ++i) {
                vertex_processor.input_vertices(
                    Eigen::Vector4f(0, 0, 0, -1.0f), Eigen::Vector3f::Zero(), -1, -1
                );
            }
            
            for (auto& worker: workers) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
        }
    }

    time_point end_time           = steady_clock::now();
    duration   rendering_duration = end_time - begin_time;

    this->logger->info("rendering takes {:.6f} seconds", rendering_duration.count());

    for (long unsigned int i = 0; i < Context::frame_buffer.depth_buffer.size(); i++) {
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].x())
        );
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].y())
        );
        rendering_res.push_back(
            static_cast<unsigned char>(Context::frame_buffer.color_buffer[i].z())
        );
    }
}

void VertexProcessor::input_vertices(const Eigen::Vector4f& positions, const Eigen::Vector3f& normals, 
                                   int triangle_id, int vertex_in_triangle)
{
    std::unique_lock<std::mutex> lock(queue_mutex);
    VertexShaderPayload payload;
    payload.world_position = positions;
    payload.normal = normals;
    vertex_queue.push(std::make_tuple(payload, triangle_id, vertex_in_triangle));
}


void VertexProcessor::worker_thread()
{
    while (!Context::vertex_finish) {
        std::tuple<VertexShaderPayload, int, int> payload_with_info;
        {
            if (vertex_queue.empty()) {
                std::this_thread::yield();
                continue;
            }
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (vertex_queue.empty()) {
                continue;
            }
            payload_with_info = vertex_queue.front();
            vertex_queue.pop();
        }
        
        auto& payload = std::get<0>(payload_with_info);
        int triangle_id = std::get<1>(payload_with_info);
        int vertex_in_triangle = std::get<2>(payload_with_info);
        
        // 检查结束信号
        if (payload.world_position.w() == -1.0f) {
            Context::vertex_finish = true;
            return;
        }
        
        // 执行顶点着色器
        VertexShaderPayload output_payload = vertex_shader_ptr(payload);
        
        // 按三角形分组存储处理后的顶点
        {
            std::unique_lock<std::mutex> lock(triangles_mutex);
            processed_triangles[triangle_id][vertex_in_triangle] = output_payload;
            
            // 检查是否收集齐一个三角形的三个顶点
            auto& triangle_vertices = processed_triangles[triangle_id];
            bool triangle_complete = true;
            for (int i = 0; i < 3; i++) {
                if (triangle_vertices[i].world_position.w() == 0) {
                    triangle_complete = false;
                    break;
                }
            }
            
            if (triangle_complete) {
                // 按顺序将三个顶点推送到输出队列
                std::unique_lock<std::mutex> output_lock(Context::vertex_queue_mutex);
                for (int i = 0; i < 3; i++) {
                    Context::vertex_shader_output_queue.push(triangle_vertices[i]);
                }
                // 移除已处理的三角形
                processed_triangles.erase(triangle_id);
            }
        }
    }
}

void FragmentProcessor::worker_thread()
{
    while (!Context::fragment_finish) {
        FragmentShaderPayload fragment;
        {
            if (Context::rasterizer_finish && Context::rasterizer_output_queue.empty()) {
                Context::fragment_finish = true;
                return;
            }
            if (Context::rasterizer_output_queue.empty()) {
                continue;
            }
            std::unique_lock<std::mutex> lock(Context::rasterizer_queue_mutex);
            if (Context::rasterizer_output_queue.empty()) {
                continue;
            }
            fragment = Context::rasterizer_output_queue.front();
            Context::rasterizer_output_queue.pop();
        }
        int index = (Uniforms::height - 1 - fragment.y) * Uniforms::width + fragment.x;
        if (fragment.depth > Context::frame_buffer.depth_buffer[index]) {
            continue;
        }
        fragment.color =
            fragment_shader_ptr(fragment, Uniforms::material, Uniforms::lights, Uniforms::camera);
        Context::frame_buffer.set_pixel(index, fragment.depth, fragment.color);
    }
}
