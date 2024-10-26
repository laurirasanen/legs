#include <memory>

#include <legs/entry.hpp>

#include <legs/entity/mesh_entity.hpp>
#include <legs/entity/sky.hpp>
#include <legs/geometry/icosphere.hpp>
#include <legs/geometry/plane.hpp>
#include <legs/isystem.hpp>
#include <legs/log.hpp>
#include <legs/time.hpp>

using namespace legs;

class MySystem : public ISystem
{
  public:
    MySystem()
    {
        int width;
        int height;
        g_engine->GetWindow()->GetFramebufferSize(&width, &height);
        m_camera = std::make_shared<NoclipCamera>(width, height);
        m_camera->SetPosition({0.0f, -10.0f, 5.0f});
        g_engine->SetCamera(m_camera);

        auto renderer = g_engine->GetRenderer();
        auto world    = g_engine->GetWorld();

        // Create a test plane
        std::shared_ptr<Buffer> planeVertexBuffer;
        std::shared_ptr<Buffer> planeIndexBuffer;

        auto testPlane     = SPlane({0.0f, 0.0f, 0.0f}, 20.0f);
        auto planeVertices = std::array<Vertex_P_C, 4>();
        for (unsigned int i = 0; i < 4; i++)
        {
            auto color = glm::vec3 {
                i == 0 ? 1.0f : 0.0f,
                i == 1 ? 1.0f : 0.0f,
                i == 2 ? 1.0f : 0.0f,
            };
            planeVertices[i] = {testPlane.vertices[i], color};
        }
        renderer->CreateBuffer(
            planeVertexBuffer,
            VertexBuffer,
            planeVertices.data(),
            sizeof(Vertex_P_C),
            static_cast<uint32_t>(planeVertices.size())
        );
        renderer->CreateBuffer(
            planeIndexBuffer,
            IndexBuffer,
            testPlane.indices.data(),
            sizeof(Index),
            static_cast<uint32_t>(testPlane.indices.size())
        );

        auto plane = std::make_shared<MeshEntity>();
        plane->SetBuffers(planeVertexBuffer, planeIndexBuffer);
        plane->SetPipeline(RenderPipeline::GEO_P_C);
        world->AddEntity(plane);
    }

    ~MySystem()
    {
    }

    void OnFrame() override
    {
        m_camera->HandleInput(g_engine->GetFrameInput());
    }

    void OnTick() override
    {
    }

  private:
    std::shared_ptr<NoclipCamera> m_camera;
};

int main(int argc, char** argv)
{
    Log::SetLogLevel(LogLevel::Debug);

    auto code = LEGS_Init(argc, argv);
    if (code < 0)
    {
        return code;
    }

    g_engine->GetWindow()->SetTitle("03_systems");

    g_engine->AddSystem(std::make_shared<MySystem>());

    return LEGS_Run();
}
