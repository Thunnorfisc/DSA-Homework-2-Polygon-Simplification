/**
 * @name    placement.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Steiner point placement for area-preserving two-vertex collapse operations.
 */

#pragma once
#include "polygon.h"
#include <cstdint>

/**
 * @name  CollapseCandidate
 * @brief Represents a candidate collapse of two consecutive vertices B,C into a single Steiner point E,
 *        storing the surrounding context vertices A,D and the resulting areal displacement cost.
 */
struct CollapseCandidate {
    Node* A;
    Node* B;  // will be removed
    Node* C;  // will be removed
    Node* D;
    double ex, ey;        // Steiner point position
    double displacement;  // areal displacement cost
    int ring_id;
    uint64_t serial;      // tie-breaking: lower serial = earlier candidate

    // Validity check: verify linked list structure is still intact
    bool is_valid() const {
        return A && B && C && D
            && !A->deleted && !B->deleted && !C->deleted && !D->deleted
            && A->next == B && B->next == C && C->next == D
            && A->ring_id == B->ring_id && B->ring_id == C->ring_id
            && C->ring_id == D->ring_id;
    }
};

/**
 * @name  CompareCandidates
 * @brief Comparator for CollapseCandidate ordering: lower displacement has higher priority, serial breaks ties.
 */
struct CompareCandidates {
    bool operator()(const CollapseCandidate& a, const CollapseCandidate& b) const {
        // Compare by displacement first, then by serial for tie-breaking
        double diff = a.displacement - b.displacement;
        if (diff > 1e-9) return true;    // a has larger displacement = lower priority
        if (diff < -1e-9) return false;   // a has smaller displacement = higher priority
        return a.serial > b.serial;       // earlier serial = higher priority
    }
};

/**
 * @name  compute_candidate
 * @brief Computes the best area-preserving Steiner point for collapsing B,C in sequence A->B->C->D.
 *        Tries placement on both ray A->B and ray D->C, picks the one with lower displacement.
 *        Returns false if no valid placement exists.
 */
bool compute_candidate(Node* A, Node* B, Node* C, Node* D, CollapseCandidate& out);