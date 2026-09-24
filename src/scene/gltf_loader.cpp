#include "pathtracer/scene/gltf_loader.h"

#include <cgltf.h>

#include <charconv>
#include <cstddef>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "pathtracer/gfx/hdr_image.h"

namespace pathtracer::scene {

namespace {

// Raw glTF-read vertex, one per accessor entry: an intermediate the world-triangle and shading-triangle builders
// consume, not retained past load.
struct Vertex {
    glm::vec3 position;
    glm::vec2 uv;
    glm::vec3 normal;
    glm::vec4 tangent;  // .w = bitangent handedness (glTF convention)
    glm::vec3 colour;   // COLOR_0, multiplies baseColor; white (1,1,1) when the primitive has none
};

std::string dirOf(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return pos == std::string::npos ? "." : path.substr(0, pos);
}

// This project's glTF material `extras` are hand-authored to look like {"roughnessTexture":{"index":2}, ...}, so the
// loader reads them as a parallel texture-slot table beside the core material.
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

std::optional<pathtracer::gfx::ImageTexture> loadTexture(const cgltf_texture* texture, const std::string& dir,
                                                      pathtracer::gfx::ScalarType textureType) {
    if (texture == nullptr || texture->image == nullptr || texture->image->uri == nullptr) {
        return std::nullopt;
    }
    return pathtracer::gfx::loadImageTexture(dir + "/" + texture->image->uri, textureType);
}

std::optional<pathtracer::gfx::ImageTexture> loadTextureByIndex(const cgltf_data* data, std::optional<int> index,
                                                             const std::string& dir, pathtracer::gfx::ScalarType textureType) {
    if (!index.has_value() || *index < 0 ||
        static_cast<cgltf_size>(*index) >= data->textures_count) {
        return std::nullopt;
    }
    return loadTexture(&data->textures[static_cast<cgltf_size>(*index)], dir, textureType);
}

glm::mat4 localNodeTransform(const cgltf_node* node) {
    if (node->has_matrix) {
        return glm::make_mat4(node->matrix);
    }
    float local[16];
    cgltf_node_transform_local(node, local);
    return glm::make_mat4(local);
}

// Appends this primitive's triangles to outWorldTriangles, each vertex baked to world space by transform: EmbreeAccel
// operates on one flat world-space soup rather than per-instance geometry with transforms.
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
            v.colour,
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
    const cgltf_accessor* color;  // COLOR_0, optional -- nullptr means "no vertex colour"
};

// Locates the position/normal/uv0/tangent accessors this loader requires, plus an optional COLOR_0, and returns
// nullopt for a primitive missing any required one or using a sparse accessor for any of them: cgltf_accessor_read_
// float cannot communicate that failure through its return value, in their own source's words.
std::optional<RequiredAccessors> findAttributeAccessors(const cgltf_primitive& prim) {
    RequiredAccessors acc{nullptr, nullptr, nullptr, nullptr, nullptr};
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
        } else if (attr.type == cgltf_attribute_type_color && attr.index == 0) {
            acc.color = attr.data;
        }
    }
    if (acc.position == nullptr || acc.normal == nullptr || acc.uv == nullptr ||
        acc.tangent == nullptr) {
        std::cerr << "loadGltf: primitive missing position/normal/uv/tangent\n";
        return std::nullopt;
    }
    if (acc.position->is_sparse || acc.normal->is_sparse || acc.uv->is_sparse ||
        acc.tangent->is_sparse || (acc.color != nullptr && acc.color->is_sparse)) {
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
        if (acc.color != nullptr) {
        // COLOR_0 may be VEC3 or VEC4 per the glTF 2.0 core spec. cgltf normalizes the component type transparently
        // but will not default a missing 4th component, so the read is sized to the accessor's own type and alpha is
        // dropped -- nothing in this engine consumes vertex-colour alpha.
            float raw[4] = {1.0F, 1.0F, 1.0F, 1.0F};
            cgltf_accessor_read_float(acc.color, vi, raw, cgltf_num_components(acc.color->type));
            v.colour = glm::vec3(raw[0], raw[1], raw[2]);
        } else {
            v.colour = glm::vec3(1.0F);
        }
    }
    return vertices;
}

// Rejects a missing or sparse index accessor for the same reason findAttributeAccessors rejects sparse vertex
// attributes: cgltf cannot signal that read failure through its return value.
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

    // A slot referencing no texture at all is not a failure and substitutes the fallback. A slot that does reference
    // one but fails to resolve or decode it is a real error and must propagate as nullopt: distinguishing the two is
    // exactly what loadTexture cannot do alone.
std::optional<pathtracer::gfx::ImageTexture> resolveTexture(const cgltf_texture* texture, const std::string& dir,
                                                         pathtracer::gfx::ScalarType textureType, pathtracer::gfx::ImageTexture fallback) {
    if (texture == nullptr) {
        return fallback;
    }
    return loadTexture(texture, dir, textureType);
}

std::optional<pathtracer::gfx::ImageTexture> resolveTextureByIndex(const cgltf_data* data, std::optional<int> index,
                                                                 const std::string& dir, pathtracer::gfx::ScalarType textureType,
                                                                 pathtracer::gfx::ImageTexture fallback) {
    if (!index.has_value()) {
        return fallback;
    }
    return loadTextureByIndex(data, index, dir, textureType);
}

std::optional<Material> loadMaterialTextures(const cgltf_data* data, const cgltf_material& mat,
                                              const std::string& dir, pathtracer::gfx::ScalarType textureType) {
    const Material defaults = makeDefaultMaterial();
    auto baseColor = resolveTexture(mat.pbr_metallic_roughness.base_color_texture.texture, dir, textureType,
                                     defaults.baseColorTexture);
    auto normal = resolveTexture(mat.normal_texture.texture, dir, textureType, defaults.normalTexture);
    auto ao = resolveTexture(mat.occlusion_texture.texture, dir, textureType, defaults.aoTexture);
    auto roughness = resolveTextureByIndex(
        data, extrasTextureIndex(mat.extras.data, "roughnessTexture"), dir, textureType, defaults.roughnessTexture);
    auto specular = resolveTextureByIndex(
        data, extrasTextureIndex(mat.extras.data, "specularTexture"), dir, textureType, defaults.specularTexture);
    auto bump = resolveTextureByIndex(data, extrasTextureIndex(mat.extras.data, "bumpTexture"), dir, textureType,
                                       defaults.bumpTexture);
    if (!baseColor || !normal || !ao || !roughness || !specular || !bump) {
        std::cerr << "loadGltf: material '" << (mat.name != nullptr ? mat.name : "<unnamed>")
                   << "' references a texture that failed to load\n";
        return std::nullopt;
    }

    return Material{
        std::move(*baseColor), std::move(*normal), std::move(*bump),
        std::move(*roughness), std::move(*specular), std::move(*ao),
    };
}

// Builds one MeshInstance's vertex and index arrays and its Material from a single triangle primitive. Fails clearly
// with nullopt rather than substituting defaults for geometry it cannot read.
std::optional<MeshInstance> loadPrimitive(const cgltf_data* data, const cgltf_primitive& prim,
                                           const glm::mat4& transform, const std::string& dir,
                                           pathtracer::gfx::ScalarType textureType, int instanceIndex, const std::string& name,
                                           std::vector<Triangle>& outWorldTriangles,
                                           std::vector<ShadingTriangle>& outShadingTriangles) {
    if (prim.type != cgltf_primitive_type_triangles) {
        std::cerr << "loadGltf: skipping non-triangle primitive\n";
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
    // Zero-initialized: every texture pointer and the name/extras fields are null, matching cgltf's
    // own representation of "this field wasn't in the JSON" -- loadMaterialTextures's per-slot
    // resolveTexture(..., default...()) calls already treat that as "use the neutral default".
    static const cgltf_material kDefaultMaterial{};
    std::optional<Material> material =
        loadMaterialTextures(data, prim.material != nullptr ? *prim.material : kDefaultMaterial, dir, textureType);
    if (!material.has_value()) {
        return std::nullopt;
    }

    const std::vector<Vertex> vertices = readVertices(*acc);
    appendWorldTriangles(vertices, *indices, transform, outWorldTriangles);
    appendShadingTriangles(vertices, *indices, transform, instanceIndex, outShadingTriangles);

    return MeshInstance{
        std::move(*material),
        transform,
        name,
    };
}

// Hard cap on node-graph recursion depth. glTF's node hierarchy is untrusted external data and cgltf_validate checks
// neither cycles nor pathological depth, so the walk bounds itself.
constexpr int kMaxNodeDepth = 256;

// A glTF node hierarchy is a tree, so recursion is its structure; the depth cap above is what makes it safe on
// untrusted input, and an explicit stack would restate the call stack while gaining no invariant.
// NOLINTNEXTLINE(misc-no-recursion)
bool walkNodes(const cgltf_data* data, cgltf_node* const* nodes, cgltf_size count,
               const glm::mat4& parentTransform, const std::string& dir, pathtracer::gfx::ScalarType textureType,
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
            const std::string name = node->name != nullptr ? node->name : "";
            for (cgltf_size pi = 0; pi < node->mesh->primitives_count; ++pi) {
                const int instanceIndex = static_cast<int>(instances.size());  // index this primitive's MeshInstance will get
                std::optional<MeshInstance> instance =
                    loadPrimitive(data, node->mesh->primitives[pi], world, dir, textureType, instanceIndex, name,
                                  worldTriangles, shadingTriangles);
                if (!instance.has_value()) {
                    return false;
                }
                instances.push_back(std::move(*instance));
            }
        }

        if (!walkNodes(data, node->children, node->children_count, world, dir, textureType, instances,
                       worldTriangles, shadingTriangles, depth + 1)) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::optional<LoadedModel> loadGltf(const std::string& path, pathtracer::gfx::ScalarType textureType,
                                     const glm::mat4& rootTransform, const std::string& textureDir) {
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
    const std::string dir = textureDir.empty() ? dirOf(path) : textureDir;
    const bool ok = data->scene != nullptr &&
                    walkNodes(data, data->scene->nodes, data->scene->nodes_count, rootTransform,
                              dir, textureType, model.instances, model.worldTriangles, model.shadingTriangles);

    cgltf_free(data);

    if (!ok || model.instances.empty()) {
        std::cerr << "loadGltf: no renderable primitives found in " << path << '\n';
        return std::nullopt;
    }
    return model;
}

void appendQuadLights(LoadedModel& model, const std::vector<QuadLight>& lights,
                       std::vector<int>& instanceLightIndex) {
    for (std::size_t i = 0; i < lights.size(); ++i) {
        const QuadLight& light = lights[i];
        const glm::vec3& normal = light.normal;
        const glm::vec4 tangent(glm::normalize(light.edge0), 1.0F);
        const auto vertex = [&](const glm::vec3& position, glm::vec2 uv) {
            return ShadingVertex{position, normal, uv, tangent};
        };
        // Corners: p00 = origin, p10/p01 along edge0/edge1, p11 the far corner -- split along the
        // p00-p11 diagonal into two triangles, both wound so cross(v1-v0, v2-v0) reproduces `normal`
        // (matching geometricNormalOf's convention, gbuffer_shading.cpp).
        const ShadingVertex p00 = vertex(light.origin, glm::vec2(0.0F, 0.0F));
        const ShadingVertex p10 = vertex(light.origin + light.edge0, glm::vec2(1.0F, 0.0F));
        const ShadingVertex p01 = vertex(light.origin + light.edge1, glm::vec2(0.0F, 1.0F));
        const ShadingVertex p11 = vertex(light.origin + light.edge0 + light.edge1, glm::vec2(1.0F, 1.0F));

        const int instanceIndex = static_cast<int>(model.instances.size());
        model.worldTriangles.push_back(Triangle{p00.position, p10.position, p11.position});
        model.worldTriangles.push_back(Triangle{p00.position, p11.position, p01.position});
        model.shadingTriangles.push_back(ShadingTriangle{p00, p10, p11, instanceIndex});
        model.shadingTriangles.push_back(ShadingTriangle{p00, p11, p01, instanceIndex});
        model.instances.push_back(MeshInstance{makeDefaultMaterial(), glm::mat4(1.0F),
                                                 "__quadLight" + std::to_string(i)});
        instanceLightIndex.push_back(static_cast<int>(i));
    }
}

}  // namespace pathtracer::scene
