#pragma once
#include "polygon.h"
#include <vector>
#include <unordered_set>
#include <functional>

// A segment in the spatial index, defined by two node endpoints
struct Segment {
    Node* from;
    Node* to;

    bool operator==(const Segment& other) const {
        return from == other.from && to == other.to;
    }
};

struct SegmentHash {
    size_t operator()(const Segment& s) const {
        auto h1 = std::hash<Node*>{}(s.from);
        auto h2 = std::hash<Node*>{}(s.to);
        return h1 ^ (h2 << 1);
    }
};

// Axis-aligned bounding box
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

// R-tree node
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

class SpatialGrid {  // keeping the name for interface compatibility
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