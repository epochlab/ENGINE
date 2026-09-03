#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <nlohmann/json.hpp>

namespace {

using nlohmann::json;

std::vector<std::uint8_t> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("could not read " + path.string());
    }
    const std::streamsize size = file.tellg();
    file.seekg(0);
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
    file.read(reinterpret_cast<char*>(bytes.data()), size);
    return bytes;
}

// bufferView/accessor byteOffsets are optional in glTF, default 0.
std::size_t byteOffset(const json& j, const char* key) {
    return j.contains(key) ? j.at(key).get<std::size_t>() : 0;
}

// Reads a tightly-packed float VEC3 accessor. This tool only ever reads NORMAL (to synthesize a
// tangent from), which Houdini's glTF ROP -- this tool's sole input format -- always exports as
// float, non-interleaved; no stride/componentType generality is needed beyond that one case.
std::vector<glm::vec3> readNormals(const json& accessor, const json& bufferViews,
                                    const std::vector<std::uint8_t>& bin) {
    if (accessor.at("componentType").get<int>() != 5126) {
        throw std::runtime_error("expected float (5126) NORMAL accessor");
    }
    const json& bufferView = bufferViews.at(accessor.at("bufferView").get<std::size_t>());
    const std::size_t offset = byteOffset(bufferView, "byteOffset") + byteOffset(accessor, "byteOffset");
    const std::size_t count = accessor.at("count").get<std::size_t>();
    std::vector<float> raw(count * 3);
    std::memcpy(raw.data(), bin.data() + offset, raw.size() * sizeof(float));
    std::vector<glm::vec3> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        out.emplace_back(raw[(i * 3) + 0], raw[(i * 3) + 1], raw[(i * 3) + 2]);
    }
    return out;
}

// Branchless orthonormal-basis-from-normal (Duff et al., "Building an Orthonormal Basis,
// Revisited"). Direction is provably irrelevant here: with no normal/bump texture in play, no
// in-plane tangent direction shows up in the shaded result.
glm::vec4 tangentFromNormal(const glm::vec3& n) {
    const float sign = n.z >= 0.0F ? 1.0F : -1.0F;
    const float a = -1.0F / (sign + n.z);
    const float b = n.x * n.y * a;
    const glm::vec3 t(1.0F + (sign * n.x * n.x * a), sign * b, -sign * n.x);
    return glm::vec4(glm::normalize(t), 1.0F);
}

void appendFloats(std::vector<std::uint8_t>& bin, const float* data, std::size_t count) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
    bin.insert(bin.end(), bytes, bytes + (count * sizeof(float)));
}

// Adds a synthesized TANGENT accessor/bufferView/binary segment to one primitive that lacks one.
// Every other attribute (POSITION, NORMAL, TEXCOORD_0, COLOR_0, or anything else), every material,
// and every other primitive is never read into memory at all, so nothing else is ever dropped or
// rewritten -- this is the tool's entire remaining job.
bool addTangentsToPrimitive(json& primitive, json& accessors, json& bufferViews,
                             std::vector<std::uint8_t>& bin) {
    json& attrs = primitive.at("attributes");
    if (attrs.contains("TANGENT")) {
        return false;
    }
    if (!attrs.contains("NORMAL")) {
        throw std::runtime_error("primitive has no NORMAL attribute to synthesize a tangent from");
    }
    const json& normAccessor = accessors.at(attrs.at("NORMAL").get<std::size_t>());
    const std::vector<glm::vec3> normals = readNormals(normAccessor, bufferViews, bin);

    const std::size_t tanOffset = bin.size();
    for (const glm::vec3& n : normals) {
        const glm::vec4 t = tangentFromNormal(n);
        appendFloats(bin, &t.x, 4);
    }

    const std::size_t bufferViewIndex = bufferViews.size();
    bufferViews.push_back(
        {{"buffer", 0}, {"byteOffset", tanOffset}, {"byteLength", bin.size() - tanOffset}});
    const std::size_t accessorIndex = accessors.size();
    accessors.push_back({{"bufferView", bufferViewIndex},
                          {"componentType", 5126},
                          {"type", "VEC4"},
                          {"count", normals.size()}});
    attrs["TANGENT"] = accessorIndex;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: gltf_tangent <file.gltf>\n";
        return EXIT_FAILURE;
    }

    try {
        const std::filesystem::path gltfPath = argv[1];
        std::ifstream inFile(gltfPath);
        if (!inFile) {
            throw std::runtime_error("could not read " + gltfPath.string());
        }
        json j;
        inFile >> j;

        // Single-buffer input assumed throughout, matching Houdini's glTF ROP output (this tool's
        // sole supported input format).
        const std::string bufferUri = j.at("buffers").at(0).at("uri").get<std::string>();
        const std::filesystem::path binPath = gltfPath.parent_path() / bufferUri;
        std::vector<std::uint8_t> bin = readBinaryFile(binPath);

        json& accessors = j.at("accessors");
        json& bufferViews = j.at("bufferViews");
        int tangentsAdded = 0;
        for (json& mesh : j.at("meshes")) {
            for (json& primitive : mesh.at("primitives")) {
                tangentsAdded += addTangentsToPrimitive(primitive, accessors, bufferViews, bin) ? 1 : 0;
            }
        }
        j["buffers"][0]["byteLength"] = bin.size();

        {
            std::ofstream binFile(binPath, std::ios::binary);
            binFile.write(reinterpret_cast<const char*>(bin.data()),
                           static_cast<std::streamsize>(bin.size()));
        }
        std::ofstream gltfFile(gltfPath);
        gltfFile << j.dump(2);

        std::cout << "gltf_tangent: patched " << gltfPath.string() << " (" << tangentsAdded
                   << " primitive(s) gained a synthesized TANGENT)\n";
    } catch (const std::exception& e) {
        std::cerr << "gltf_tangent: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
