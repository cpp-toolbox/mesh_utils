#ifndef MESH_UTILS_HPP
#define MESH_UTILS_HPP

#include <glm/glm.hpp>

#include <numeric>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <sstream>

#include "sbpt_generated_includes.hpp"

/**
 * todo:
 * - in the future it might be useful to cache certain things like normals, until it's a problem I don't care
 *
 *
 */

namespace mesh_utils {

using FaceIdx = uint32_t;
using VertexIdx = uint32_t;
using EdgeIdx = uint32_t;

struct Vertex {
    glm::vec3 position;
    std::vector<FaceIdx> incident_face_indices;
    std::vector<EdgeIdx> incident_edge_indices;
};

struct EdgeByIndices {
    VertexIdx v0_idx;
    VertexIdx v1_idx;
    std::vector<FaceIdx> incident_face_indices;

    bool is_boundary() const { return incident_face_indices.size() == 1; }
    bool is_internal() const { return incident_face_indices.size() == 2; }
    bool is_non_manifold() const { return incident_face_indices.size() > 2; }
};

struct FaceByIndices {
    std::vector<VertexIdx> vertex_indices;
    std::vector<EdgeIdx> edge_indices;
};

struct EdgeKey {
    VertexIdx a, b;
    EdgeKey(uint32_t v0, uint32_t v1) : a(std::min(v0, v1)), b(std::max(v0, v1)) {}
    bool operator==(const EdgeKey &other) const { return a == other.a && b == other.b; }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey &k) const { return (static_cast<size_t>(k.a) << 32) ^ k.b; }
};

class Mesh {
  public:
    LogSection::LogMode log_mode;

    /// registers a vertex into this mesh, can be used later to create faces or edges with
    VertexIdx add_vertex(const glm::vec3 &position) {
        GlobalLogSection _("add_vertex", log_mode);
        vertices.push_back(Vertex{position});
        return static_cast<VertexIdx>(vertices.size() - 1);
    }

    std::vector<FaceIdx> get_all_face_indices() {
        std::vector<FaceIdx> v(faces.size());
        std::iota(v.begin(), v.end(), 0);
        return v;
    };

    /**
     * @brief create a triangular face out of existing vertices in this mesh, return the idx of the new face
     *
     * TODO: can we pass in an std::array 3 instead of enforcing size on the inside?
     */
    FaceIdx add_face(const std::vector<VertexIdx> &vertex_indices_to_construct_face) {
        GlobalLogSection _("add_face", log_mode);

        // Ensure the face is a triangle
        if (vertex_indices_to_construct_face.size() != 3) {
            global_logger->debug("add_face: rejected non-triangle face (size = {})",
                                 vertex_indices_to_construct_face.size());
            throw std::runtime_error("Only triangular faces are supported");
        }

        uint32_t face_index = static_cast<uint32_t>(faces.size());
        global_logger->debug("add_face: creating triangle face with index {}", face_index);

        FaceByIndices face;
        face.vertex_indices = vertex_indices_to_construct_face;

        // process the 3 edges of the triangle
        for (size_t i = 0; i < 3; ++i) {
            uint32_t v0 = vertex_indices_to_construct_face[i];
            uint32_t v1 = vertex_indices_to_construct_face[i + 1 == 3 ? 0 : i + 1]; // Wrap around

            global_logger->debug("Processing edge ({}, {})", v0, v1);

            EdgeKey key(v0, v1);
            uint32_t edge_index;

            auto it = edge_lookup.find(key);
            if (it == edge_lookup.end()) {
                edge_index = static_cast<uint32_t>(edges.size());
                edges.push_back(EdgeByIndices{key.a, key.b});
                edge_lookup.emplace(key, edge_index);
                global_logger->debug("Created new edge {} with index {}", edge_index, edge_index);

                vertices[key.a].incident_edge_indices.push_back(edge_index);
                vertices[key.b].incident_edge_indices.push_back(edge_index);
                global_logger->debug("Added edge {} to vertices {} and {}", edge_index, key.a, key.b);
            } else {
                edge_index = it->second;
                global_logger->debug("Found existing edge {} for vertices ({}, {})", edge_index, v0, v1);
            }

            edges[edge_index].incident_face_indices.push_back(face_index);
            face.edge_indices.push_back(edge_index);
            global_logger->debug("Linked edge {} with face {}", edge_index, face_index);
        }

        // link vertices to face
        for (uint32_t v : vertex_indices_to_construct_face) {
            vertices[v].incident_face_indices.push_back(face_index);
            global_logger->debug("Linked vertex {} with face {}", v, face_index);
        }

        faces.push_back(std::move(face));
        global_logger->debug("Triangle face {} added successfully", face_index);

        return face_index;
    }

    void flip_face_winding(FaceIdx face_index) {
        if (face_index >= faces.size())
            throw std::out_of_range("Invalid face index");

        FaceByIndices &f = faces[face_index];

        // only works for triangles, reverse order of vertices
        if (f.vertex_indices.size() == 3) {
            std::swap(f.vertex_indices[1], f.vertex_indices[2]);
        } else {
            // for n-gons, just reverse all vertices
            std::reverse(f.vertex_indices.begin(), f.vertex_indices.end());
        }

        // recompute edges for this face
        f.edge_indices.clear();
        for (size_t i = 0; i < f.vertex_indices.size(); ++i) {
            uint32_t v0 = f.vertex_indices[i];
            uint32_t v1 = f.vertex_indices[(i + 1) % f.vertex_indices.size()];

            EdgeKey key(v0, v1);
            auto it = edge_lookup.find(key);
            if (it != edge_lookup.end()) {
                f.edge_indices.push_back(it->second);
            } else {
                uint32_t edge_index = static_cast<uint32_t>(edges.size());
                edges.push_back(EdgeByIndices{key.a, key.b});
                edge_lookup.emplace(key, edge_index);
                vertices[key.a].incident_edge_indices.push_back(edge_index);
                vertices[key.b].incident_edge_indices.push_back(edge_index);
                f.edge_indices.push_back(edge_index);
            }
        }
    }

    // i don't think returning references is good because the vector can be re-allocated right?
    const Vertex &vertex(VertexIdx i) const { return vertices[i]; }
    const EdgeByIndices &edge(EdgeIdx i) const { return edges[i]; }
    const FaceByIndices &face(FaceIdx i) const { return faces[i]; }

    const std::vector<uint32_t> &faces_using_vertex(uint32_t v) const { return vertices[v].incident_face_indices; }

    struct FaceNormal {
        glm::vec3 start;
        glm::vec3 end;
    };

    std::vector<FaceNormal> get_face_normals() const {
        std::vector<FaceNormal> result;
        result.reserve(faces.size());

        for (const FaceByIndices &f : faces) {
            glm::vec3 normal = compute_face_normal(f);
            if (glm::length(normal) == 0.0f)
                continue; // skip degenerate faces

            glm::vec3 centroid = compute_face_centroid(f);

            result.push_back(FaceNormal{centroid, centroid + normal});
        }

        return result;
    }

    bool has_non_manifold_edges() const {
        for (const auto &e : edges)
            if (e.is_non_manifold())
                return true;
        return false;
    }

    void move_face(uint32_t face_idx, const glm::vec3 &delta) {
        FaceByIndices &f = faces.at(face_idx);

        for (uint32_t vi : f.vertex_indices) {
            vertices[vi].position += delta;
        }
    }

    void move_face_along_normal(FaceIdx face_index, float distance) {
        glm::vec3 n = compute_face_normal(faces.at(face_index));
        move_face(face_index, n * distance);
    }

    bool face_has_internal_edge(FaceIdx face_index) const {
        if (face_index >= faces.size())
            throw std::out_of_range("Invalid face index");

        const FaceByIndices &f = faces[face_index];
        for (uint32_t e_idx : f.edge_indices) {
            if (edges[e_idx].is_internal())
                return true;
        }
        return false;
    }

    std::vector<uint32_t> extrude_faces(const std::vector<uint32_t> &face_indices, float magnitude = 1.0f,
                                        const glm::vec3 &direction = glm::vec3(0.0f)) {
        GlobalLogSection _("extrude_faces", log_mode);

        if (face_indices.empty()) {
            global_logger->debug("no face indices");
            return {};
        }

        // NOTE: A: for a collection of faces, it defines a collection of boundary edges, for any boundary edge which is
        // an internal edge relative to the original mesh, then when we extrude these faces, it creates an egde with
        // more than 2 adjacent faces which makes the mesh non-manifold, if this would occur we should delete the
        // original face we extruded to keep the mesh a manifold

        // NOTE: B: if none of the edges involved were internal edges then since we moved along that face to produce the
        // extruded face both their normals will be pointing in the same direction, and the original face's normal will
        // be pointing inward, to fix this we will change the winding order of the original extruded face

        std::vector<std::pair<uint32_t, uint32_t>> boundary_edges = compute_boundary_edges(face_indices);

        // create a lookup set of edgekeys for fast boundary edge checking
        std::unordered_set<EdgeKey, EdgeKeyHash> boundary_edge_set;
        for (auto [v0, v1] : boundary_edges) {
            boundary_edge_set.emplace(v0, v1); // edgekey constructor normalizes the order
        }

        std::vector<uint32_t> faces_to_remove;
        std::vector<uint32_t> faces_to_flip_winding_order;

        for (uint32_t fi : face_indices) {
            // TODO: need to put guards in place that the face idx actually exists there.
            const FaceByIndices &f = faces[fi];
            bool has_internal_boundary_edge = false;

            for (uint32_t e_idx : f.edge_indices) {
                const EdgeByIndices &e = edges[e_idx];
                EdgeKey key(e.v0_idx, e.v1_idx); // normalized edge key

                // Only consider edges that are both internal and on the boundary
                if (e.is_internal() && boundary_edge_set.count(key)) {
                    has_internal_boundary_edge = true;
                    break;
                }
            }

            if (has_internal_boundary_edge) {
                faces_to_remove.push_back(fi);
            } else {
                faces_to_flip_winding_order.push_back(fi);
            }
        }

        glm::vec3 extrude_dir = direction;
        if (glm::length(extrude_dir) == 0.0f) {
            // TODO: this is bad.
            extrude_dir = compute_face_normal(faces.at(face_indices[0])); // approximate with first face
        }

        std::unordered_set<uint32_t> vertices_to_extrude;
        for (uint32_t fi : face_indices) {
            const FaceByIndices f = faces[fi];
            vertices_to_extrude.insert(f.vertex_indices.begin(), f.vertex_indices.end());
        }

        std::unordered_map<uint32_t, uint32_t> original_to_extruded;
        for (uint32_t vi : vertices_to_extrude) {
            uint32_t new_vi = add_vertex(vertices[vi].position + extrude_dir * magnitude);
            original_to_extruded[vi] = new_vi;
        }

        // create side faces
        for (auto [v0, v1] : boundary_edges) {
            uint32_t v0_new = original_to_extruded[v0];
            uint32_t v1_new = original_to_extruded[v1];

            // split quad (v0, v1, v1_new, v0_new) into two triangles
            add_face({v0, v1, v1_new});
            add_face({v0, v1_new, v0_new});
        }

        // create top faces
        std::vector<uint32_t> extruded_faces_indices;
        for (uint32_t fi : face_indices) {
            const FaceByIndices f = faces[fi];
            std::vector<uint32_t> extruded_vertices;
            for (uint32_t vi : f.vertex_indices)
                extruded_vertices.push_back(original_to_extruded[vi]);

            uint32_t top_face_index = add_face(extruded_vertices);
            extruded_faces_indices.push_back(top_face_index);
        }

        for (const auto &fi : faces_to_remove) {
            global_logger->debug("removing face with idx: {}", fi);
            // TODO: remove face
        }

        for (const auto &fi : faces_to_flip_winding_order) {
            global_logger->debug("flipping face with idx: {}", fi);
            flip_face_winding(fi);
        }

        return extruded_faces_indices;
    }

    uint32_t extrude_face(uint32_t face_index, float magnitude = 1.0f, const glm::vec3 &direction = glm::vec3(0.0f)) {
        std::vector<uint32_t> top_faces = extrude_faces({face_index}, magnitude, direction);
        return top_faces.empty() ? UINT32_MAX : top_faces.front();
    }

    void align_face(uint32_t face_index, const glm::vec3 &target_normal) {
        FaceByIndices f = faces.at(face_index);

        glm::vec3 current_normal = compute_face_normal(f);
        glm::vec3 desired_normal = glm::normalize(target_normal);

        if (glm::length(current_normal) == 0.0f || glm::length(desired_normal) == 0.0f)
            throw std::runtime_error("Invalid normal vector");

        // Already aligned?
        if (glm::dot(current_normal, desired_normal) > 0.9999f)
            return;

        // Rotation axis and angle
        glm::vec3 axis = glm::cross(current_normal, desired_normal);
        float axis_len = glm::length(axis);

        // Opposite direction (180 degrees)
        if (axis_len < 1e-6f) {
            axis = orthogonal_vector(current_normal);
            axis_len = glm::length(axis);
        }

        axis /= axis_len;
        float angle = std::acos(glm::clamp(glm::dot(current_normal, desired_normal), -1.0f, 1.0f));

        // Rotation center = face centroid
        glm::vec3 centroid = compute_face_centroid(f);

        glm::mat4 R = glm::rotate(glm::mat4(1.0f), angle, axis);

        // Rotate vertices incident to this face
        for (uint32_t vi : f.vertex_indices) {
            glm::vec3 p = vertices[vi].position;
            glm::vec4 tmp = glm::vec4(p - centroid, 1.0f);
            p = glm::vec3(R * tmp) + centroid;
        }
    }

    bool is_convex() const {
        GlobalLogSection _("is_convex");
        /**
         * We say that a mesh is convex iff for every face, all other vertices lie on the same side of the plane defined
         * by that face.
         */

        if (faces.empty() || vertices.size() <= 3) {
            global_logger->debug("Mesh is trivially convex (faces empty or <= 3 vertices).");
            return true;
        }

        for (size_t fi = 0; fi < faces.size(); ++fi) {
            const auto &f = faces[fi];
            glm::vec3 normal = compute_face_normal(f);

            if (glm::length(normal) == 0.0f) {
                global_logger->debug("Skipping degenerate face {}.", fi);
                continue; // degenerate face, skip
            }

            const glm::vec3 &p0 = vertices[f.vertex_indices[0]].position;
            global_logger->debug("Testing face {} with normal {}", fi, vec3_to_string(normal));

            // Check all other vertices
            for (size_t vi = 0; vi < vertices.size(); ++vi) {
                if (std::find(f.vertex_indices.begin(), f.vertex_indices.end(), static_cast<uint32_t>(vi)) !=
                    f.vertex_indices.end())
                    continue; // skip vertices in this face

                const glm::vec3 &p = vertices[vi].position;
                glm::vec3 vec = p - p0;
                float dot = glm::dot(vec, normal);

                global_logger->debug("  Vertex {} at {}: dot with face normal = {}", vi, vec3_to_string(p), dot);

                // TODO: why tf is the tolerance so bad...
                bool vertex_is_planar_with_face = std::abs(dot) < 0.1;

                if (vertex_is_planar_with_face) {
                    global_logger->debug("current vertex we just tested is planar with the face, ignoring it");
                    continue;
                }

                if (dot > 0) {
                    global_logger->debug("Face {} causes concavity due to vertex {}.", fi, vi);
                    return false;
                }
            }
        }

        global_logger->debug("Mesh is convex.");
        return true;
    }

    std::string to_string() const {
        std::ostringstream oss;

        oss << "Mesh:\n";
        oss << "  Vertices (" << vertices.size() << "):\n";
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto &v = vertices[i].position;
            oss << "    " << i << ": " << vec3_to_string(v) << "\n";
        }

        oss << "  Edges (" << edges.size() << "):\n";
        for (size_t i = 0; i < edges.size(); ++i) {
            const auto &e = edges[i];
            oss << "    " << i << ": (" << e.v0_idx << ", " << e.v1_idx << "), incident_faces = [";
            for (size_t j = 0; j < e.incident_face_indices.size(); ++j) {
                oss << e.incident_face_indices[j];
                if (j + 1 < e.incident_face_indices.size())
                    oss << ", ";
            }
            oss << "]\n";
        }

        oss << "  Faces (" << faces.size() << "):\n";
        for (size_t i = 0; i < faces.size(); ++i) {
            const auto &f = faces[i];
            oss << "    " << i << ": vertices = [";
            for (size_t j = 0; j < f.vertex_indices.size(); ++j) {
                oss << f.vertex_indices[j];
                if (j + 1 < f.vertex_indices.size())
                    oss << ", ";
            }
            oss << "], edges = [";
            for (size_t j = 0; j < f.edge_indices.size(); ++j) {
                oss << f.edge_indices[j];
                if (j + 1 < f.edge_indices.size())
                    oss << ", ";
            }
            oss << "]\n";
        }

        return oss.str();
    }

  private:
    // returns a pair of vertex indices
    std::vector<std::pair<VertexIdx, VertexIdx>>
    compute_boundary_edges(const std::vector<FaceIdx> &face_indices) const {
        std::unordered_set<uint32_t> face_set(face_indices.begin(), face_indices.end());
        std::vector<std::pair<uint32_t, uint32_t>> boundary_edges;

        // we compute the boundary edges by iterating over every edge of every face.
        for (FaceIdx fi : face_indices) {
            const FaceByIndices f = faces[fi];
            size_t num_vertices_on_face = f.vertex_indices.size();
            for (size_t i = 0; i < num_vertices_on_face; ++i) {
                uint32_t v0_idx = f.vertex_indices[i];
                uint32_t v1_idx = f.vertex_indices[(i + 1) % num_vertices_on_face];

                EdgeKey key(v0_idx, v1_idx);
                auto it = edge_lookup.find(key);
                if (it == edge_lookup.end())
                    continue; // should not happen

                const EdgeByIndices e = edges[it->second];

                // count how many selected faces use this edge
                size_t count_in_set = 0;
                for (uint32_t face_id : e.incident_face_indices)
                    if (face_set.count(face_id))
                        count_in_set++;

                // if edge is used by exactly one face in the set, it's a boundary edge
                if (count_in_set == 1)
                    boundary_edges.emplace_back(v0_idx, v1_idx);
            }
        }

        return boundary_edges;
    }

    glm::vec3 compute_face_normal(const FaceByIndices &f) const {
        if (f.vertex_indices.size() < 3)
            return glm::vec3(0.0f);

        const glm::vec3 &p0 = vertices[f.vertex_indices[0]].position;
        const glm::vec3 &p1 = vertices[f.vertex_indices[1]].position;
        const glm::vec3 &p2 = vertices[f.vertex_indices[2]].position;

        glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));

        return normal;
    }

    glm::vec3 compute_face_centroid(const FaceByIndices &f) const {
        glm::vec3 c(0.0f);
        for (uint32_t v : f.vertex_indices)
            c += vertices[v].position;
        return c / static_cast<float>(f.vertex_indices.size());
    }

    // finds any vector orthogonal to n
    static glm::vec3 orthogonal_vector(const glm::vec3 &n) {
        return glm::abs(n.x) < 0.9f ? glm::cross(n, glm::vec3(1, 0, 0)) : glm::cross(n, glm::vec3(0, 1, 0));
    }

  public:
    // these are mappings where the id is just the index
    std::vector<Vertex> vertices;
    std::vector<EdgeByIndices> edges;
    std::vector<FaceByIndices> faces;

  private:
    std::unordered_map<EdgeKey, uint32_t, EdgeKeyHash> edge_lookup;
};

// TODO: make it take in vertices and indices and build adapters ontop with templates and store in a different file?
inline Mesh to_mesh(const draw_info::IndexedVertexPositions &ivp,
                    const LogSection::LogMode &log_mode = LogSection::LogMode::inherit) {
    Mesh mesh;
    mesh.log_mode = log_mode;

    for (const auto &pos : ivp.xyz_positions) {
        mesh.add_vertex(pos);
    }

    if (ivp.indices.size() % 3 != 0)
        throw std::runtime_error("IVP indices size must be a multiple of 3 for triangle faces");

    for (size_t i = 0; i < ivp.indices.size(); i += 3) {
        mesh.add_face({ivp.indices[i], ivp.indices[i + 1], ivp.indices[i + 2]});
    }

    return mesh;
}

inline draw_info::IndexedVertexPositions to_indexed_vertex_positions(const Mesh &mesh) {
    draw_info::IndexedVertexPositions out;
    out.xyz_positions.reserve(mesh.vertices.size());

    for (const auto &v : mesh.vertices)
        out.xyz_positions.push_back(v.position);

    for (const auto &f : mesh.faces) {
        for (size_t i = 1; i + 1 < f.vertex_indices.size(); ++i) {
            out.indices.push_back(f.vertex_indices[0]);
            out.indices.push_back(f.vertex_indices[i]);
            out.indices.push_back(f.vertex_indices[i + 1]);
        }
    }

    return out;
}

} // namespace mesh_utils

#endif // MESH_UTILS_HPP
