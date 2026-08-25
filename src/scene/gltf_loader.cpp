#include "engine/scene/gltf_loader.h"

#include <cgltf.h>

#include <charconv>
#include <cstddef>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "engine/gfx/hdr_image.h"

namespace engine::scene {

namespace {

// Raw glTF-read vertex, one per accessor entry -- an intermediate the world-triangle/shading-triangle builders below consume; not retained past loadPrimitive.
struct Vertex {
    glm::vec3 position;
    glm::vec2 uv;
    glm::vec3 normal;
    glm::vec4 tangent;  // .w = bitangent handedness (glTF convention)
};

std::string dirOf(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return pos == std::string::npos ? "." : path.substr(0, pos);
}

// This project's gltf material `extras` are hand-authored to look like {"roughnessTexture":{"index":2}, ...} (see the gltf fix-up this loader depends on) -- not a general JSON parser, just enough to pull an integer index back out of that exact, self-controlled shape.
std::optional<int> extrasTextureIndex(const char* extrasJson, const std::string& key) {
    if (extrasJson == nullptr) {
        return std::nullopt;
    }
    const std::string text(extrasJson);
    const std::size_t keyPos = text.find("\"" + key + "\"");
    if (keyPos == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t indexPos = text.find("\"index\"", keyPos);
    if (indexPos == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t colonPos = text.find(':', indexPos);
    if (colonPos == std::string::npos) {
        return std::nullopt;
    }
    std::size_t numPos = colonPos + 1;
    while (numPos < text.size() && (text[numPos] == ' ' || text[numPos] == '\t')) {
        ++numPos;
    }
    // from_chars, not atoi: atoi can't distinguish "parsed 0" from "failed to parse", and 0 is a valid texture index.
    int value = 0;
    const std::from_chars_result result =
        std::from_chars(text.c_str() + numPos, text.c_str() + text.size(), value);
    if (result.ec != std::errc{}) {
        return std::nullopt;
    }
    return value;
}

std::optional<engine::gfx::HdrImage> loadTexture(const cgltf_texture* texture,
                                                  const std::string& dir) {
    if (texture == nullptr || texture->image == nullptr || texture->image->uri == nullptr) {
        return std::nullopt;
    }
    return engine::gfx::loadExr(dir + "/" + texture->image->uri);
}

std::optional<engine::gfx::HdrImage> loadTextureByIndex(const cgltf_data* data,
                                                         std::optional<int> index,
                                                         const std::string& dir) {
    if (!index.has_value() || *index < 0 ||
        static_cast<cgltf_size>(*index) >= data->textures_count) {
        return std::nullopt;
    }
    return loadTexture(&data->textures[static_cast<cgltf_size>(*index)], dir);
}

glm::mat4 localNodeTransform(const cgltf_node* node) {
    if (node->has_matrix) {
        return glm::make_mat4(node->matrix);
    }
    float local[16];
    cgltf_node_transform_local(node, local);
    return glm::make_mat4(local);
}

// Appends this primitive's triangles to outWorldTriangles, each vertex baked to world space by transform -- EmbreeAccel (embree_accel.h) operates on world-space triangles, not the model-space Vertex data read from the accessors.
void appendWorldTriangles(const std::vector<Vertex>& vertices,
                           const std::vector<unsigned int>& indices, const glm::mat4& transform,
                           std::vector<Triangle>& outWorldTriangles) {
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        const auto toWorld = [&](unsigned int index) {
            return glm::vec3(transform * glm::vec4(vertices[index].position, 1.0F));
        };
        outWorldTriangles.push_back(
            Triangle{toWorld(indices[i]), toWorld(indices[i + 1]), toWorld(indices[i + 2])});
    }
}

// Parallel to appendWorldTriangles: normal via inverse-transpose, tangent via transform directly.
void appendShadingTriangles(const std::vector<Vertex>& vertices,
                             const std::vector<unsigned int>& indices, const glm::mat4& transform,
                             int instanceIndex, std::vector<ShadingTriangle>& outShadingTriangles) {
    const glm::mat3 linear(transform);
    const glm::mat3 normalMatrix = glm::inverseTranspose(linear);
    const auto toWorldVertex = [&](unsigned int index) {
        const Vertex& v = vertices[index];
        return ShadingVertex{
            glm::vec3(transform * glm::vec4(v.position, 1.0F)),
            glm::normalize(normalMatrix * v.normal),
            v.uv,
            glm::vec4(glm::normalize(linear * glm::vec3(v.tangent)), v.tangent.w),
        };
    };
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        outShadingTriangles.push_back(ShadingTriangle{
            toWorldVertex(indices[i]),
            toWorldVertex(indices[i + 1]),
            toWorldVertex(indices[i + 2]),
            instanceIndex,
        });
    }
}

struct RequiredAccessors {
    const cgltf_accessor* position;
    const cgltf_accessor* normal;
    const cgltf_accessor* uv;
    const cgltf_accessor* tangent;
};

// Locates the position/normal/uv0/tangent accessors this loader requires and rejects (nullopt) a primitive missing any of them, or -- since cgltf_accessor_read_float can't signal failure through its return value for a sparse accessor (their own source: "This is an error case, but we can't communicate the error with existing interface") -- a primitive using a sparse accessor for any of them, which this loader doesn't support.
std::optional<RequiredAccessors> findAttributeAccessors(const cgltf_primitive& prim) {
    RequiredAccessors acc{nullptr, nullptr, nullptr, nullptr};
    for (cgltf_size ai = 0; ai < prim.attributes_count; ++ai) {
        const cgltf_attribute& attr = prim.attributes[ai];
        if (attr.type == cgltf_attribute_type_position) {
            acc.position = attr.data;
        } else if (attr.type == cgltf_attribute_type_normal) {
            acc.normal = attr.data;
        } else if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) {
            acc.uv = attr.data;
        } else if (attr.type == cgltf_attribute_type_tangent) {
            acc.tangent = attr.data;
        }
    }
    if (acc.position == nullptr || acc.normal == nullptr || acc.uv == nullptr ||
        acc.tangent == nullptr) {
        std::cerr << "loadGltf: primitive missing position/normal/uv/tangent\n";
        return std::nullopt;
    }
    if (acc.position->is_sparse || acc.normal->is_sparse || acc.uv->is_sparse ||
        acc.tangent->is_sparse) {
        std::cerr << "loadGltf: sparse accessors are not supported\n";
        return std::nullopt;
    }
    return acc;
}

std::vector<Vertex> readVertices(const RequiredAccessors& acc) {
    std::vector<Vertex> vertices(acc.position->count);
    for (cgltf_size vi = 0; vi < acc.position->count; ++vi) {
        Vertex& v = vertices[vi];
        cgltf_accessor_read_float(acc.position, vi, &v.position.x, 3);
        cgltf_accessor_read_float(acc.normal, vi, &v.normal.x, 3);
        // No V flip: glTF's v=0-at-top already matches loadExr's row-0-at-top convention.
        cgltf_accessor_read_float(acc.uv, vi, &v.uv.x, 2);
        cgltf_accessor_read_float(acc.tangent, vi, &v.tangent.x, 4);
    }
    return vertices;
}

// Rejects a missing or sparse index accessor for the same reason findAttributeAccessors rejects sparse vertex attributes -- cgltf can't signal a sparse-index read failure through its return value either.
std::optional<std::vector<unsigned int>> readIndices(const cgltf_accessor* indicesAcc) {
    if (indicesAcc == nullptr) {
        std::cerr << "loadGltf: primitive has no index accessor\n";
        return std::nullopt;
    }
    if (indicesAcc->is_sparse) {
        std::cerr << "loadGltf: sparse accessors are not supported\n";
        return std::nullopt;
    }
    std::vector<unsigned int> indices(indicesAcc->count);
    for (cgltf_size ii = 0; ii < indicesAcc->count; ++ii) {
        indices[ii] = static_cast<unsigned int>(cgltf_accessor_read_index(indicesAcc, ii));
    }
    return indices;
}

std::optional<Material> loadMaterialTextures(const cgltf_data* data, const cgltf_material& mat,
                                              const std::string& dir) {
    const cgltf_pbr_metallic_roughness& pbr = mat.pbr_metallic_roughness;

    auto baseColor = loadTexture(pbr.base_color_texture.texture, dir);
    auto normal = loadTexture(mat.normal_texture.texture, dir);
    auto ao = loadTexture(mat.occlusion_texture.texture, dir);
    auto roughness =
        loadTextureByIndex(data, extrasTextureIndex(mat.extras.data, "roughnessTexture"), dir);
    auto specular =
        loadTextureByIndex(data, extrasTextureIndex(mat.extras.data, "specularTexture"), dir);
    if (!baseColor || !normal || !ao || !roughness || !specular) {
        std::cerr << "loadGltf: material '" << (mat.name != nullptr ? mat.name : "<unnamed>")
                   << "' is missing one or more of the 5 required textures\n";
        return std::nullopt;
    }

    // Defaults match the glTF extension specs.
    const float ior = mat.has_ior ? mat.ior.ior : 1.5F;
    const float transmissionFactor =
        mat.has_transmission ? mat.transmission.transmission_factor : 0.0F;

    return Material{
        glm::make_vec4(pbr.base_color_factor),
        pbr.metallic_factor,
        pbr.roughness_factor,
        std::move(*baseColor),
        std::move(*normal),
        std::move(*roughness),
        std::move(*specular),
        std::move(*ao),
        ior,
        transmissionFactor,
    };
}

// Builds one MeshInstance's Vertex/index arrays and Material from a single triangle primitive. Fails clearly (nullopt) rather than substituting a placeholder for a primitive this loader doesn't support (non-triangle mode, missing attributes/material/textures).
std::optional<MeshInstance> loadPrimitive(const cgltf_data* data, const cgltf_primitive& prim,
                                           const glm::mat4& transform, const std::string& dir,
                                           int instanceIndex,
                                           std::vector<Triangle>& outWorldTriangles,
                                           std::vector<ShadingTriangle>& outShadingTriangles) {
    if (prim.type != cgltf_primitive_type_triangles) {
        std::cerr << "loadGltf: skipping non-triangle primitive\n";
        return std::nullopt;
    }
    if (prim.material == nullptr) {
        std::cerr << "loadGltf: primitive has no material\n";
        return std::nullopt;
    }

    const std::optional<RequiredAccessors> acc = findAttributeAccessors(prim);
    if (!acc.has_value()) {
        return std::nullopt;
    }
    const std::optional<std::vector<unsigned int>> indices = readIndices(prim.indices);
    if (!indices.has_value()) {
        return std::nullopt;
    }
    std::optional<Material> material = loadMaterialTextures(data, *prim.material, dir);
    if (!material.has_value()) {
        return std::nullopt;
    }

    const std::vector<Vertex> vertices = readVertices(*acc);
    appendWorldTriangles(vertices, *indices, transform, outWorldTriangles);
    appendShadingTriangles(vertices, *indices, transform, instanceIndex, outShadingTriangles);

    return MeshInstance{
        std::move(*material),
        transform,
    };
}

// Hard cap on node-graph recursion depth. glTF's node hierarchy is untrusted external data -- cgltf_validate doesn't check for cycles or pathological nesting depth, so an unbounded recursion here would let a malformed/cyclic file overflow the stack. 256 comfortably covers any legitimate scene hierarchy.
constexpr int kMaxNodeDepth = 256;

bool walkNodes(const cgltf_data* data, cgltf_node* const* nodes, cgltf_size count,
               const glm::mat4& parentTransform, const std::string& dir,
               std::vector<MeshInstance>& instances, std::vector<Triangle>& worldTriangles,
               std::vector<ShadingTriangle>& shadingTriangles, int depth = 0) {
    if (depth >= kMaxNodeDepth) {
        std::cerr << "loadGltf: node hierarchy exceeds max depth " << kMaxNodeDepth
                   << " (cyclic or pathologically nested)\n";
        return false;
    }
    for (cgltf_size ni = 0; ni < count; ++ni) {
        const cgltf_node* node = nodes[ni];
        const glm::mat4 world = parentTransform * localNodeTransform(node);

        if (node->mesh != nullptr) {
            for (cgltf_size pi = 0; pi < node->mesh->primitives_count; ++pi) {
                const int instanceIndex = static_cast<int>(instances.size());  // index this primitive's MeshInstance will get
                std::optional<MeshInstance> instance =
                    loadPrimitive(data, node->mesh->primitives[pi], world, dir, instanceIndex,
                                  worldTriangles, shadingTriangles);
                if (!instance.has_value()) {
                    return false;
                }
                instances.push_back(std::move(*instance));
            }
        }

        if (!walkNodes(data, node->children, node->children_count, world, dir, instances,
                       worldTriangles, shadingTriangles, depth + 1)) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::optional<LoadedModel> loadGltf(const std::string& path) {
    const cgltf_options options{};
    cgltf_data* data = nullptr;

    if (cgltf_parse_file(&options, path.c_str(), &data) != cgltf_result_success) {
        std::cerr << "loadGltf: failed to parse " << path << '\n';
        return std::nullopt;
    }
    if (cgltf_load_buffers(&options, data, path.c_str()) != cgltf_result_success) {
        std::cerr << "loadGltf: failed to load buffers for " << path << '\n';
        cgltf_free(data);
        return std::nullopt;
    }
    if (cgltf_validate(data) != cgltf_result_success) {
        std::cerr << "loadGltf: validation failed for " << path << '\n';
        cgltf_free(data);
        return std::nullopt;
    }

    LoadedModel model;
    const std::string dir = dirOf(path);
    const bool ok = data->scene != nullptr &&
                    walkNodes(data, data->scene->nodes, data->scene->nodes_count, glm::mat4(1.0F),
                              dir, model.instances, model.worldTriangles, model.shadingTriangles);

    cgltf_free(data);

    if (!ok || model.instances.empty()) {
        std::cerr << "loadGltf: no renderable primitives found in " << path << '\n';
        return std::nullopt;
    }
    return model;
}

}  // namespace engine::scene
