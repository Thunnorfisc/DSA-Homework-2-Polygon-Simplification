/**
 * @name    spatial_index.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   R-tree spatial index for efficient segment intersection queries during polygon simplification.
 */

#pragma once
#include "polygon.h"
#include <vector>
#include <unordered_set>
#include <functional>

/**
 * @name  Segment
 * @brief A directed line segment defined by two Node endpoint pointers.
 */
struct Segment {
    Node* from;
    Node* to;

    bool operator==(const Segment& other) const {
        return from == other.from && to == other.to;
    }
};

/**
 * @name  SegmentHash
 * @brief Hash functor for Segment, enabling use in unordered containers.
 */
struct SegmentHash {
    size_t operator()(const Segment& s) const {
        auto h1 = std::hash<Node*>{}(s.from);
        auto h2 = std::hash<Node*>{}(s.to);
        return h1 ^ (h2 << 1);
    }
};

/**
 * @name  AABB
 * @brief Axis-aligned bounding box used for spatial queries and R-tree node bounds.
 */
struct AABB {
    double min_x, min_y, max_x, max_y;

    AABB() : min_x(1e18), min_y(1e18), max_x(-1e18), max_y(-1e18) {}
    AABB(double x1, double y1, double x2, double y2);

    double area() const;
    double enlargement(const AABB& other) const;
    void expand(const AABB& other);
    bool overlaps(const AABB& other) const;

    static AABB from_segment(Node* a, Node* b);
};

/**
 * @name  RTreeNode
 * @brief A node in the R-tree; stores either child node pointers (internal) or segment entries (leaf).
 */
struct RTreeNode {
    static constexpr int MAX_ENTRIES = 16;
    static constexpr int MIN_ENTRIES = 4;  // must be <= MAX_ENTRIES/2

    AABB bbox;
    bool is_leaf;

    // Internal node: children
    std::vector<RTreeNode*> children;

    // Leaf node: entries (segments + their bounding boxes)
    std::vector<Segment> entries;
    std::vector<AABB> entry_boxes;

    RTreeNode(bool leaf) : is_leaf(leaf) {}
    ~RTreeNode();

    int count() const { return is_leaf ? (int)entries.size() : (int)children.size(); }
    void recompute_bbox();
};

/**
 * @name       SpatialGrid
 * @brief      R-tree based spatial index for polygon segments, supporting dynamic insert/remove and range queries.
 * @operations
 *   - build:  Bulk-insert all segments from all rings — O(V log V) time, O(V) space.
 *   - insert: Insert a single segment into the R-tree — O(log V) time, O(1) space.
 *   - remove: Remove a single segment from the R-tree — O(V) worst case due to reinsertion, O(log V) typical.
 *   - query:  Find all segments whose bounding box overlaps a query rectangle — O(sqrt(V) + k) typical time.
 */
class SpatialGrid {
private:
    RTreeNode* root;

    // Choose the best child to insert into (minimum area enlargement)
    RTreeNode* choose_leaf(RTreeNode* node, const AABB& box);

    // Split an overflowing node using quadratic split
    RTreeNode* split_node(RTreeNode* node);

    // Handle overflow: split and propagate up
    void handle_overflow(RTreeNode* node, std::vector<RTreeNode*>& path);

    // Find the leaf containing a specific segment
    RTreeNode* find_leaf(RTreeNode* node, const Segment& seg, const AABB& box,
                         std::vector<RTreeNode*>& path);

    // Condense tree after removal
    void condense_tree(std::vector<RTreeNode*>& path);

    // Query recursively
    void query_recursive(RTreeNode* node, const AABB& query_box,
                         std::vector<Segment>& result,
                         const std::unordered_set<Segment, SegmentHash>& exclude) const;

public:
    SpatialGrid() : root(nullptr) {}
    ~SpatialGrid();

    void build(const std::vector<Ring>& rings);
    void insert(Node* from, Node* to);
    void remove(Node* from, Node* to);
    void query(double qx1, double qy1, double qx2, double qy2,
               std::vector<Segment>& result,
               const std::unordered_set<Segment, SegmentHash>& exclude) const;
};