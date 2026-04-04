/**
 * @name    heap.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Min-heap priority queue for ordering vertex collapse candidates by displacement cost.
 */

#pragma once
#include "placement.h"
#include <vector>

/**
 * @name       MinHeap
 * @brief      Array-backed binary min-heap that prioritizes CollapseCandidate by displacement cost,
 *             with serial number tie-breaking for deterministic ordering.
 * @operations
 *   - push:  Insert a candidate into the heap — O(log n) time, O(1) amortized space.
 *   - pop:   Remove and return the minimum-cost candidate — O(log n) time, O(1) space.
 *   - top:   Peek at the minimum-cost candidate without removal — O(1) time, O(1) space.
 *   - empty: Check if the heap is empty — O(1) time, O(1) space.
 *   - size:  Return the number of elements — O(1) time, O(1) space.
 */
class MinHeap {
private:
    std::vector<CollapseCandidate> data;

    int parent(int i) { return (i - 1) / 2; }
    int left(int i) { return 2 * i + 1; }
    int right(int i) { return 2 * i + 2; }

    bool higher_priority(const CollapseCandidate& a, const CollapseCandidate& b);

    void sift_up(int i);
    void sift_down(int i);

public:
    void push(const CollapseCandidate& cand);
    CollapseCandidate pop();
    const CollapseCandidate& top() const;
    bool empty() const { return data.empty(); }
    int size() const { return static_cast<int>(data.size()); }
};