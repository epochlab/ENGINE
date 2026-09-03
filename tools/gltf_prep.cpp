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

struct Mesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> uvs;
    std::vector<std::uint32_t> indices;
};

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

// Reads a tightly-packed float accessor (this tool's own output and Houdini's exports both write
// accessors this way -- no interleaving/stride support needed).
std::vector<float> readFloats(const json& accessor, const json& bufferViews,
                               const std::vector<std::uint8_t>& bin, int numComponents) {
    const json& bufferView = bufferViews.at(accessor.at("bufferView").get<std::size_t>());
    if (accessor.at("componentType").get<int>() != 5126) {
        throw std::runtime_error("expected float (5126) accessor");
    }
    const std::size_t offset = byteOffset(bufferView, "byteOffset") + byteOffset(accessor, "byteOffset");
    const std::size_t count = accessor.at("count").get<std::size_t>();
    std::vector<float> out(count * static_cast<std::size_t>(numComponents));
    std::memcpy(out.data(), bin.data() + offset, out.size() * sizeof(float));
    return out;
}

std::vector<std::uint32_t> readIndices(const json& accessor, const json& bufferViews,
                                        const std::vector<std::uint8_t>& bin) {
    const json& bufferView = bufferViews.at(accessor.at("bufferView").get<std::size_t>());
    const std::size_t offset = byteOffset(bufferView, "byteOffset") + byteOffset(accessor, "byteOffset");
    const std::size_t count = accessor.at("count").get<std::size_t>();
    const int componentType = accessor.at("componentType").get<int>();

    std::vector<std::uint32_t> out(count);
    const std::uint8_t* src = bin.data() + offset;
    switch (componentType) {
        case 5121:  // UNSIGNED_BYTE
            for (std::size_t i = 0; i < count; ++i) {
                out[i] = src[i];
            }
            break;
        case 5123: {  // UNSIGNED_SHORT
            std::vector<std::uint16_t> raw(count);
            std::memcpy(raw.data(), src, count * sizeof(std::uint16_t));
            for (std::size_t i = 0; i < count; ++i) {
                out[i] = raw[i];
            }
            break;
        }
        case 5125:  // UNSIGNED_INT
            std::memcpy(out.data(), src, count * sizeof(std::uint32_t));
            break;
        default:
            throw std::runtime_error("unsupported index componentType " +
                                      std::to_string(componentType));
    }
    return out;
}

// Loads a Houdini-exported glTF holding POSITION + NORMAL + TEXCOORD_0 + an index accessor on its
// single mesh primitive -- this tool's whole job is filling in what Houdini's glTF ROP doesn't
// produce (TANGENT, a material), so the input shape it accepts is deliberately narrow. Also
// returns the buffer's own uri so the caller can overwrite that exact file in place.
struct LoadedMesh {
    Mesh mesh;
    std::string bufferUri;
};

LoadedMesh loadMesh(const std::filesystem::path& gltfPath) {
    std::ifstream file(gltfPath);
    if (!file) {
        throw std::runtime_error("could not read " + gltfPath.string());
    }
    json j;
    file >> j;

    const json& prim = j.at("meshes").at(0).at("primitives").at(0);
    const json& attrs = prim.at("attributes");
    const json& accessors = j.at("accessors");
    const json& bufferViews = j.at("bufferViews");

    const json& posAccessor = accessors.at(attrs.at("POSITION").get<std::size_t>());
    const json& normAccessor = accessors.at(attrs.at("NORMAL").get<std::size_t>());
    const json& uvAccessor = accessors.at(attrs.at("TEXCOORD_0").get<std::size_t>());
    const json& idxAccessor = accessors.at(prim.at("indices").get<std::size_t>());

    const json& buffer = j.at("buffers").at(0);
    const std::string bufferUri = buffer.at("uri").get<std::string>();
    const std::vector<std::uint8_t> bin = readBinaryFile(gltfPath.parent_path() / bufferUri);

    Mesh mesh;
    const std::vector<float> posFloats = readFloats(posAccessor, bufferViews, bin, 3);
    const std::vector<float> normFloats = readFloats(normAccessor, bufferViews, bin, 3);
    const std::vector<float> uvFloats = readFloats(uvAccessor, bufferViews, bin, 2);
    const std::size_t vertexCount = posAccessor.at("count").get<std::size_t>();

    mesh.positions.reserve(vertexCount);
    mesh.normals.reserve(vertexCount);
    mesh.uvs.reserve(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i) {
        mesh.positions.emplace_back(posFloats[i * 3], posFloats[i * 3 + 1], posFloats[i * 3 + 2]);
        mesh.normals.emplace_back(normFloats[i * 3], normFloats[i * 3 + 1], normFloats[i * 3 + 2]);
        mesh.uvs.emplace_back(uvFloats[i * 2], uvFloats[i * 2 + 1]);
    }
    mesh.indices = readIndices(idxAccessor, bufferViews, bin);
    return LoadedMesh{std::move(mesh), bufferUri};
}

// Branchless orthonormal-basis-from-normal (Duff et al., "Building an Orthonormal Basis,
// Revisited"). Direction is provably irrelevant here: with no normal/bump texture in play, no
// in-plane tangent direction shows up in the shaded result.
glm::vec4 tangentFromNormal(const glm::vec3& n) {
    const float sign = n.z >= 0.0F ? 1.0F : -1.0F;
    const float a = -1.0F / (sign + n.z);
    const float b = n.x * n.y * a;
    const glm::vec3 t(1.0F + sign * n.x * n.x * a, sign * b, -sign * n.x);
    return glm::vec4(glm::normalize(t), 1.0F);
}

void appendFloats(std::vector<std::uint8_t>& bin, const float* data, std::size_t count) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
    bin.insert(bin.end(), bytes, bytes + count * sizeof(float));
}

// Overwrites gltfPath and its buffer file in place: adds a synthesized TANGENT accessor and a
// bare material (no texture references -- gltf_loader.cpp substitutes neutral in-memory defaults
// for any texture slot a material doesn't reference, so no placeholder image files are needed).
void writeOutputs(const std::filesystem::path& gltfPath, const std::string& bufferUri,
                   const Mesh& mesh, const std::vector<glm::vec4>& tangents) {
    const std::size_t vertexCount = mesh.positions.size();
    const std::filesystem::path binPath = gltfPath.parent_path() / bufferUri;

    std::vector<std::uint8_t> bin;
    bin.reserve(vertexCount * (3 + 3 + 2 + 4) * sizeof(float) + mesh.indices.size() * sizeof(std::uint32_t));

    const std::size_t posOffset = bin.size();
    for (const glm::vec3& p : mesh.positions) {
        appendFloats(bin, &p.x, 3);
    }
    const std::size_t normOffset = bin.size();
    for (const glm::vec3& n : mesh.normals) {
        appendFloats(bin, &n.x, 3);
    }
    const std::size_t uvOffset = bin.size();
    for (const glm::vec2& uv : mesh.uvs) {
        appendFloats(bin, &uv.x, 2);
    }
    const std::size_t tanOffset = bin.size();
    for (const glm::vec4& t : tangents) {
        appendFloats(bin, &t.x, 4);
    }
    const std::size_t idxOffset = bin.size();
    const auto* idxBytes = reinterpret_cast<const std::uint8_t*>(mesh.indices.data());
    bin.insert(bin.end(), idxBytes, idxBytes + mesh.indices.size() * sizeof(std::uint32_t));

    {
        std::ofstream binFile(binPath, std::ios::binary);
        binFile.write(reinterpret_cast<const char*>(bin.data()),
                       static_cast<std::streamsize>(bin.size()));
    }

    glm::vec3 posMin = mesh.positions.front();
    glm::vec3 posMax = mesh.positions.front();
    for (const glm::vec3& p : mesh.positions) {
        posMin = glm::min(posMin, p);
        posMax = glm::max(posMax, p);
    }

    json j;
    j["asset"] = {{"generator", "gltf_prep"}, {"version", "2.0"}};
    j["buffers"] = {{{"uri", bufferUri}, {"byteLength", bin.size()}}};
    j["bufferViews"] = {
        {{"buffer", 0}, {"byteOffset", posOffset}, {"byteLength", normOffset - posOffset}},
        {{"buffer", 0}, {"byteOffset", normOffset}, {"byteLength", uvOffset - normOffset}},
        {{"buffer", 0}, {"byteOffset", uvOffset}, {"byteLength", tanOffset - uvOffset}},
        {{"buffer", 0}, {"byteOffset", tanOffset}, {"byteLength", idxOffset - tanOffset}},
        {{"buffer", 0}, {"byteOffset", idxOffset}, {"byteLength", bin.size() - idxOffset}},
    };
    j["accessors"] = {
        {{"bufferView", 0}, {"componentType", 5126}, {"type", "VEC3"}, {"count", vertexCount},
         {"min", {posMin.x, posMin.y, posMin.z}}, {"max", {posMax.x, posMax.y, posMax.z}}},
        {{"bufferView", 1}, {"componentType", 5126}, {"type", "VEC3"}, {"count", vertexCount}},
        {{"bufferView", 2}, {"componentType", 5126}, {"type", "VEC2"}, {"count", vertexCount}},
        {{"bufferView", 3}, {"componentType", 5126}, {"type", "VEC4"}, {"count", vertexCount}},
        {{"bufferView", 4}, {"componentType", 5125}, {"type", "SCALAR"}, {"count", mesh.indices.size()}},
    };
    // No texture references: gltf_loader.cpp's loadMaterialTextures substitutes a neutral default
    // for every slot this material doesn't mention. Just enough for prim.material to be non-null.
    j["materials"] = {{{"name", "cornell"}}};
    j["meshes"] = {{{"primitives",
                      {{{"indices", 4},
                        {"material", 0},
                        {"attributes",
                         {{"POSITION", 0}, {"NORMAL", 1}, {"TEXCOORD_0", 2}, {"TANGENT", 3}}}}}}}};
    j["nodes"] = {{{"mesh", 0}, {"name", "cornell"}}};
    j["scene"] = 0;
    j["scenes"] = {{{"nodes", {0}}}};

    std::ofstream gltfFile(gltfPath);
    gltfFile << j.dump(2);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: gltf_prep <file.gltf>\n";
        return EXIT_FAILURE;
    }

    try {
        const std::filesystem::path gltfPath = argv[1];
        const LoadedMesh loaded = loadMesh(gltfPath);

        std::vector<glm::vec4> tangents;
        tangents.reserve(loaded.mesh.normals.size());
        for (const glm::vec3& n : loaded.mesh.normals) {
            tangents.push_back(tangentFromNormal(n));
        }

        writeOutputs(gltfPath, loaded.bufferUri, loaded.mesh, tangents);

        std::cout << "gltf_prep: wrote " << gltfPath.string() << " (" << loaded.mesh.positions.size()
                   << " verts, " << loaded.mesh.indices.size() / 3 << " tris)\n";
    } catch (const std::exception& e) {
        std::cerr << "gltf_prep: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
