#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>

#include "polygon.h"
#include "placement.h"
#include "simplify.h"
#include "symmetric_diff.h"
#include "post_process.h"

using Pt = std::pair<double, double>;

// Extract vertices from a ring into a vector of (x,y) pairs
static std::vector<Pt> extract_vertices(const Ring& ring)
{
    std::vector<Pt> verts;
    if (ring.vertices.head == nullptr) return verts;

    Node* cur = ring.vertices.head;
    do {
        verts.push_back({cur->x, cur->y});
        cur = cur->next;
    } while (cur != ring.vertices.head);

    return verts;
}

// Compute bounding box centroid of all rings
static void compute_centroid(const std::vector<Ring>& rings, double& cx, double& cy)
{
    double min_x =  std::numeric_limits<double>::max();
    double min_y =  std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& ring : rings) {
        Node* cur = ring.vertices.head;
        if (!cur) continue;
        do {
            min_x = std::min(min_x, cur->x);
            min_y = std::min(min_y, cur->y);
            max_x = std::max(max_x, cur->x);
            max_y = std::max(max_y, cur->y);
            cur = cur->next;
        } while (cur != ring.vertices.head);
    }

    cx = (min_x + max_x) / 2.0;
    cy = (min_y + max_y) / 2.0;
}

// Shift all vertices by (-dx, -dy)
static void shift_all_vertices(std::vector<Ring>& rings, double dx, double dy)
{
    for (auto& ring : rings) {
        Node* cur = ring.vertices.head;
        if (!cur) continue;
        do {
            cur->x -= dx;
            cur->y -= dy;
            cur = cur->next;
        } while (cur != ring.vertices.head);
    }
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: ./simplify <input_file.csv> <target_vertices>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    int target_vertices = std::stoi(argv[2]);

    // Parse CSV
    std::vector<Ring> rings = parse_csv(input_file);

    // ── Normalize coordinates ──
    // Translate so bounding box is centered at origin.
    // This keeps cross products ~1e8 instead of ~1e12 for coords ~1e6.
    double offset_x = 0.0, offset_y = 0.0;
    compute_centroid(rings, offset_x, offset_y);
    shift_all_vertices(rings, offset_x, offset_y);

    std::cerr << "Coordinate offset: (" << offset_x << ", " << offset_y << ")" << std::endl;

    // Compute input signed areas (in normalized coords — area is translation-invariant)
    double total_input_area = 0.0;
    int total_vertex_count = 0;

    for (auto& ring : rings) {
        ring.original_area = compute_signed_area(ring);
        total_input_area += ring.original_area;
        total_vertex_count += ring.vertices.size;
    }

    std::cerr << "Parsed " << rings.size() << " rings, "
              << total_vertex_count << " total vertices" << std::endl;

    // Store original vertices (in normalized coords) for symmetric diff later
    std::vector<std::vector<Pt>> original_vertices;
    for (auto& ring : rings) {
        original_vertices.push_back(extract_vertices(ring));
    }

    // Early exit
    if (total_vertex_count <= target_vertices) {
        print_output(rings, total_input_area, total_input_area, 0.0, offset_x, offset_y);
        return 0;
    }

    // Simplify
    double cumulative_displacement = simplify_polygon(rings, target_vertices);

    // Post-processing: adaptive based on input size
    // > 100K vertices: skip (too slow)
    // > 20K vertices: light pass (fewer iterations)
    // Otherwise: full pass
    int relocated = 0;
    if (total_vertex_count > 100000) {
        std::cerr << "\nSkipping post-processing (input > 100K vertices)" << std::endl;
    } else if (total_vertex_count > 20000) {
        std::cerr << "\nLight post-processing (input > 20K vertices)..." << std::endl;
        relocated = post_process(rings, original_vertices, 1);
        std::cerr << "Post-processing relocated " << relocated << " vertices" << std::endl;
    } else {
        std::cerr << "\nFull post-processing..." << std::endl;
        relocated = post_process(rings, original_vertices, 3);
        std::cerr << "Post-processing relocated " << relocated << " vertices" << std::endl;
    }

    // Compute output area
    double total_output_area = 0.0;
    for (auto& ring : rings) {
        total_output_area += compute_signed_area(ring);
    }

    // Always compute actual symmetric difference per ring
    double total_sym_diff = 0.0;
    for (size_t i = 0; i < rings.size(); i++) {
        std::vector<Pt> simplified_verts = extract_vertices(rings[i]);
        double ring_sym_diff = compute_symmetric_difference(original_vertices[i], simplified_verts);
        total_sym_diff += ring_sym_diff;
        std::cerr << "  Ring " << rings[i].ring_id
                  << ": symmetric diff = " << ring_sym_diff << std::endl;
    }

    std::cerr << "Input area:  " << total_input_area << std::endl;
    std::cerr << "Output area: " << total_output_area << std::endl;
    std::cerr << "Cumulative displacement: " << cumulative_displacement << std::endl;
    std::cerr << "Symmetric difference:    " << total_sym_diff << std::endl;

    // Print output with offset added back to coordinates
    print_output(rings, total_input_area, total_output_area, total_sym_diff, offset_x, offset_y);

    return 0;
}