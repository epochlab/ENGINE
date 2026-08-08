#pragma once

#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

namespace engine::gfx {

struct Vertex {
    glm::vec3 position;
    glm::vec2 uv;
};

// Owns one VAO + VBO + EBO triple (indexed draw), move-only: GL object
// names are handles that can't be safely duplicated.
class Mesh {
public:
    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    // Unit quad (1x1, centered at the origin, z=0), CCW winding, XY plane.
    // Deliberately smaller than the [-1,1] clip-space range: this stage's
    // quad is drawn with no MVP transform, so its authored extents are
    // its NDC footprint. Sized to leave the framebuffer's cleared
    // background visible around it, so the render checkpoint verifies
    // bounded polygon geometry rather than an indistinguishable
    // full-screen fill.
    static Mesh createQuad();

    void draw() const;

    [[nodiscard]] int triangleCount() const { return indexCount_ / 3; }

private:
    unsigned int vao_ = 0;
    unsigned int vbo_ = 0;
    unsigned int ebo_ = 0;
    int indexCount_ = 0;
    std::size_t byteSize_ = 0;  // reported to engine::debug's GPU memory tracker
};

}  // namespace engine::gfx
