#include "engine/gfx/mesh.h"

#include <cstddef>
#include <utility>

#include <GL/glew.h>

#include "engine/debug/memory_tracker.h"
#include "engine/gfx/gl_debug.h"

namespace engine::gfx {

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices)
    : indexCount_(static_cast<int>(indices.size())),
      byteSize_(vertices.size() * sizeof(Vertex) + indices.size() * sizeof(unsigned int)) {
    for (const Vertex& v : vertices) {
        boundsMin_ = glm::min(boundsMin_, v.position);
        boundsMax_ = glm::max(boundsMax_, v.position);
    }

    engine::debug::trackGpuAlloc(byteSize_);
    GL_CALL(glGenVertexArrays(1, &vao_));
    GL_CALL(glGenBuffers(1, &vbo_));
    GL_CALL(glGenBuffers(1, &ebo_));

    GL_CALL(glBindVertexArray(vao_));

    GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, vbo_));
    GL_CALL(glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                         vertices.data(), GL_STATIC_DRAW));

    GL_CALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_));
    GL_CALL(glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                         indices.data(), GL_STATIC_DRAW));

    GL_CALL(glEnableVertexAttribArray(0));
    GL_CALL(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                   reinterpret_cast<const void*>(offsetof(Vertex, position))));
    GL_CALL(glEnableVertexAttribArray(1));
    GL_CALL(glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                   reinterpret_cast<const void*>(offsetof(Vertex, uv))));
    GL_CALL(glEnableVertexAttribArray(2));
    GL_CALL(glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                   reinterpret_cast<const void*>(offsetof(Vertex, normal))));
    GL_CALL(glEnableVertexAttribArray(3));
    GL_CALL(glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                   reinterpret_cast<const void*>(offsetof(Vertex, tangent))));

    // GL_ELEMENT_ARRAY_BUFFER's binding is VAO state, so it must survive
    // the VAO unbind below; GL_ARRAY_BUFFER's binding isn't VAO state but
    // is unbound too for symmetry.
    GL_CALL(glBindVertexArray(0));
    GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
}

Mesh::~Mesh() {
    engine::debug::trackGpuFree(byteSize_);
    if (ebo_ != 0) {
        glDeleteBuffers(1, &ebo_);
    }
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
    }
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
    }
}

Mesh::Mesh(Mesh&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vbo_(std::exchange(other.vbo_, 0)),
      ebo_(std::exchange(other.ebo_, 0)),
      indexCount_(std::exchange(other.indexCount_, 0)),
      byteSize_(std::exchange(other.byteSize_, 0)),
      boundsMin_(other.boundsMin_),
      boundsMax_(other.boundsMax_) {}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        engine::debug::trackGpuFree(byteSize_);
        if (ebo_ != 0) {
            glDeleteBuffers(1, &ebo_);
        }
        if (vbo_ != 0) {
            glDeleteBuffers(1, &vbo_);
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
        }
        vao_ = std::exchange(other.vao_, 0);
        vbo_ = std::exchange(other.vbo_, 0);
        ebo_ = std::exchange(other.ebo_, 0);
        indexCount_ = std::exchange(other.indexCount_, 0);
        byteSize_ = std::exchange(other.byteSize_, 0);
        boundsMin_ = other.boundsMin_;
        boundsMax_ = other.boundsMax_;
    }
    return *this;
}

Mesh Mesh::createQuad() {
    constexpr glm::vec3 kFlatNormal{0.0F, 0.0F, 1.0F};
    constexpr glm::vec4 kFlatTangent{1.0F, 0.0F, 0.0F, 1.0F};
    const std::vector<Vertex> vertices = {
        {{-0.5F, -0.5F, 0.0F}, {0.0F, 0.0F}, kFlatNormal, kFlatTangent},  // bottom-left
        {{0.5F, -0.5F, 0.0F}, {1.0F, 0.0F}, kFlatNormal, kFlatTangent},   // bottom-right
        {{0.5F, 0.5F, 0.0F}, {1.0F, 1.0F}, kFlatNormal, kFlatTangent},    // top-right
        {{-0.5F, 0.5F, 0.0F}, {0.0F, 1.0F}, kFlatNormal, kFlatTangent},   // top-left
    };
    const std::vector<unsigned int> indices = {0, 1, 2, 0, 2, 3};  // CCW
    return Mesh(vertices, indices);
}

// Not wrapped in GL_CALL: runs every frame (see gl_debug.h's rationale).
void Mesh::draw() const {
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

}  // namespace engine::gfx
