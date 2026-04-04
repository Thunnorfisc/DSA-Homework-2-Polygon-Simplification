/**
 * @name    simplify.cpp
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Greedy polygon simplification loop: collapses lowest-cost vertex pairs while
 *          maintaining topology via spatial index intersection checks.
 */

#include "simplify.h"
#include "heap.h"
#include "topology.h"
#include "spatial_index.h"
#include <iostream>
#include <unordered_set>

static uint64_t g_serial = 0;

/**
 * @name  push_candidate_from
 * @brief Computes and pushes a collapse candidate starting at vertex A (for A->B->C->D) into the heap.
 */
static void push_candidate_from(Node* A, int ring_size, MinHeap& heap)
{
    if (!A || A->deleted) return;
    if (ring_size < 4) return;

    Node* B = A->next;
    Node* C = B->next;
    Node* D = C->next;

    if (B->deleted || C->deleted || D->deleted) return;

    CollapseCandidate cand;
    if (compute_candidate(A, B, C, D, cand)) {
        cand.serial = g_serial++;
        heap.push(cand);
    }
}

/**
 * @name  simplify_polygon
 * @brief Main simplification loop: greedily collapses the cheapest vertex pair, updates the spatial
 *        index, and regenerates affected candidates until the target vertex count is reached.
 */
double simplify_polygon(std::vector<Ring>& rings, int target_vertices)
{
    int total_vertices = 0;
    for (auto& ring : rings) {
        total_vertices += ring.vertices.size;
    }

    if (total_vertices <= target_vertices) return 0.0;

    // Build spatial index
    SpatialGrid grid;
    grid.build(rings);

    // Build the min-heap with all initial candidates
    MinHeap heap;
    g_serial = 0;

    for (auto& ring : rings) {
        if (ring.vertices.size < 4) continue;

        Node* start = ring.vertices.head;
        Node* cur = start;
        do {
            push_candidate_from(cur, ring.vertices.size, heap);
            cur = cur->next;
        } while (cur != start);
    }

    double total_displacement = 0.0;

    while (total_vertices > target_vertices && !heap.empty()) {
        CollapseCandidate cand = heap.pop();

        if (!cand.is_valid()) continue;

        // Find the ring
        Ring* target_ring = nullptr;
        for (auto& ring : rings) {
            if (ring.ring_id == cand.ring_id) {
                target_ring = &ring;
                break;
            }
        }
        if (!target_ring || target_ring->vertices.size <= 3) continue;

        // Topology check
        if (collapse_causes_intersection(cand.A, cand.ex, cand.ey, cand.D,
                                         cand.B, cand.C, grid)) {
            continue;
        }

        // ── Perform the collapse ──
        Node* A = cand.A;
        Node* B = cand.B;
        Node* C = cand.C;
        Node* D = cand.D;

        // Remove old segments from spatial index
        grid.remove(A, B);
        grid.remove(B, C);
        grid.remove(C, D);

        // Insert E into linked list after A
        Node* E = target_ring->vertices.insert_after(A, cand.ex, cand.ey, cand.ring_id);

        // Remove B and C
        target_ring->vertices.remove(B);
        target_ring->vertices.remove(C);

        // Insert new segments into spatial index
        grid.insert(A, E);
        grid.insert(E, D);

        total_vertices--;
        total_displacement += cand.displacement;

        // Update head if it was B or C
        if (target_ring->vertices.head == B || target_ring->vertices.head == C) {
            target_ring->vertices.head = E;
        }

        // ── Regenerate candidates ──
        // Push from 5 starting vertices: A->prev->prev, A->prev, A, E, D
        // This covers all 4-vertex sequences that could have changed
        std::unordered_set<Node*> seen;
        Node* starts[] = {A->prev->prev, A->prev, A, E, D};
        for (Node* s : starts) {
            if (s && !s->deleted && seen.insert(s).second) {
                push_candidate_from(s, target_ring->vertices.size, heap);
            }
        }
    }

    std::cerr << "Simplification done. Total displacement: " << total_displacement << std::endl;
    std::cerr << "Vertices remaining: " << total_vertices << std::endl;

    return total_displacement;
}