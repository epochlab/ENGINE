#include "engine/scene/gltf_loader.h"

#include <cgltf.h>

#include <charconv>
#include <cstddef>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>

namespace engine::scene {

namespace {

std::string dirOf(const std::string& path) {
    const std::size_t pos = path.find_last_of('/');
    return pos == std::string::npos ? "." : path.substr(0, pos);
}

// This project's gltf material `extras` are hand-authored to look like
// {"roughnessTexture":{"index":2}, ...} (see the gltf fix-up this
// loader depends on) -- not a general JSON parser, just enough to pull
// an integer index back out of that exact, self-controlled shape.
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
    // from_chars, not atoi: atoi can't distinguish "parsed 0" from
    // "failed to parse", and 0 is a valid texture index.
    int value = 0;
    const std::from_chars_result result =
        std::from_chars(text.c_str() + numPos, text.c_str() + text.size(), value);
    if (result.ec != std::errc{}) {
        return std::nullopt;
    }
    return value;
}

// This project's gltf assets ship linear EXR maps, not glTF's usual
// PNG/JPEG -- every texture slot resolves through the same EXR loader.
std::optional<engine::gfx::Texture> loadTexture(const cgltf_texture* texture,
                                                 const std::string& dir) {
    if (texture == nullptr || texture->image == nullptr || texture->image->uri == nullptr) {
        return std::nullopt;
    }
    // Repeat: a tiled material texture, not a single fixed-scale image.
    return engine::gfx::Texture::createFromExr(dir + "/" + texture->image->uri, GL_REPEAT);
}

std::optional<engine::gfx::Texture> loadTextureByIndex(const cgltf_data* data,
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

// Appends this primitive's triangles to outWorldTriangles, each vertex
// baked to world space by transform -- the BVH (bvh.h) operates on
// world-space triangles, not the model-space data Mesh uploads to the GPU.
void appendWorldTriangles(const std::vector<engine::gfx::Vertex>& vertices,
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

// Builds one MeshInstance's Vertex/index arrays and Material from a
// single triangle primitive. Fails clearly (nullopt) rather than
// substituting a placeholder for a primitive this loader doesn't
// support (non-triangle mode, missing attributes/material/textures).
std::optional<MeshInstance> loadPrimitive(const cgltf_data* data, const cgltf_primitive& prim,
                                           const glm::mat4& transform, const std::string& dir,
                                           std::vector<Triangle>& outWorldTriangles) {
    if (prim.type != cgltf_primitive_type_triangles) {
        std::cerr << "loadGltf: skipping non-triangle primitive\n";
        return std::nullopt;
    }

    const cgltf_accessor* posAcc = nullptr;
    const cgltf_accessor* normAcc = nullptr;
    const cgltf_accessor* uvAcc = nullptr;
    const cgltf_accessor* tanAcc = nullptr;
    for (cgltf_size ai = 0; ai < prim.attributes_count; ++ai) {
        const cgltf_attribute& attr = prim.attributes[ai];
        if (attr.type == cgltf_attribute_type_position) {
            posAcc = attr.data;
        } else if (attr.type == cgltf_attribute_type_normal) {
            normAcc = attr.data;
        } else if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) {
            uvAcc = attr.data;
        } else if (attr.type == cgltf_attribute_type_tangent) {
            tanAcc = attr.data;
        }
    }
    if (posAcc == nullptr || normAcc == nullptr || uvAcc == nullptr || tanAcc == nullptr) {
        std::cerr << "loadGltf: primitive missing position/normal/uv/tangent\n";
        return std::nullopt;
    }
    if (prim.indices == nullptr) {
        std::cerr << "loadGltf: primitive has no index accessor\n";
        return std::nullopt;
    }
    if (prim.material == nullptr) {
        std::cerr << "loadGltf: primitive has no material\n";
        return std::nullopt;
    }

    std::vector<engine::gfx::Vertex> vertices(posAcc->count);
    for (cgltf_size vi = 0; vi < posAcc->count; ++vi) {
        engine::gfx::Vertex& v = vertices[vi];
        cgltf_accessor_read_float(posAcc, vi, &v.position.x, 3);
        cgltf_accessor_read_float(normAcc, vi, &v.normal.x, 3);
        // No V flip: glTF's v=0-at-top already matches how
        // Texture::createFromExr uploads EXR rows (see its own note).
        cgltf_accessor_read_float(uvAcc, vi, &v.uv.x, 2);
        cgltf_accessor_read_float(tanAcc, vi, &v.tangent.x, 4);
    }

    std::vector<unsigned int> indices(prim.indices->count);
    for (cgltf_size ii = 0; ii < prim.indices->count; ++ii) {
        indices[ii] = static_cast<unsigned int>(cgltf_accessor_read_index(prim.indices, ii));
    }

    const cgltf_material& mat = *prim.material;
    const cgltf_pbr_metallic_roughness& pbr = mat.pbr_metallic_roughness;

    auto baseColor = loadTexture(pbr.base_color_texture.texture, dir);
    auto normal = loadTexture(mat.normal_texture.texture, dir);
    auto ao = loadTexture(mat.occlusion_texture.texture, dir);
    auto roughness =
        loadTextureByIndex(data, extrasTextureIndex(mat.extras.data, "roughnessTexture"), dir);
    auto bump = loadTextureByIndex(data, extrasTextureIndex(mat.extras.data, "bumpTexture"), dir);
    auto specular =
        loadTextureByIndex(data, extrasTextureIndex(mat.extras.data, "specularTexture"), dir);
    if (!baseColor || !normal || !ao || !roughness || !bump || !specular) {
        std::cerr << "loadGltf: material '" << (mat.name != nullptr ? mat.name : "<unnamed>")
                   << "' is missing one or more of the 6 required textures\n";
        return std::nullopt;
    }

    Material material{
        glm::make_vec4(pbr.base_color_factor),
        pbr.metallic_factor,
        pbr.roughness_factor,
        std::move(*baseColor),
        std::move(*normal),
        std::move(*roughness),
        std::move(*bump),
        std::move(*specular),
        std::move(*ao),
    };

    appendWorldTriangles(vertices, indices, transform, outWorldTriangles);

    return MeshInstance{
        engine::gfx::Mesh(vertices, indices),
        std::move(material),
        transform,
    };
}

bool walkNodes(const cgltf_data* data, cgltf_node* const* nodes, cgltf_size count,
               const glm::mat4& parentTransform, const std::string& dir,
               std::vector<MeshInstance>& instances, std::vector<Triangle>& worldTriangles) {
    for (cgltf_size ni = 0; ni < count; ++ni) {
        const cgltf_node* node = nodes[ni];
        const glm::mat4 world = parentTransform * localNodeTransform(node);

        if (node->mesh != nullptr) {
            for (cgltf_size pi = 0; pi < node->mesh->primitives_count; ++pi) {
                std::optional<MeshInstance> instance =
                    loadPrimitive(data, node->mesh->primitives[pi], world, dir, worldTriangles);
                if (!instance.has_value()) {
                    return false;
                }
                instances.push_back(std::move(*instance));
            }
        }

        if (!walkNodes(data, node->children, node->children_count, world, dir, instances,
                       worldTriangles)) {
            return false;
        }
    }
    return true;
}

}  // namespace

std::optional<LoadedModel> loadGltf(const std::string& path) {
    cgltf_options options{};
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
                              dir, model.instances, model.worldTriangles);

    cgltf_free(data);

    if (!ok || model.instances.empty()) {
        std::cerr << "loadGltf: no renderable primitives found in " << path << '\n';
        return std::nullopt;
    }
    return model;
}

}  // namespace engine::scene
