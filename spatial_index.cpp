/**
 * @name    spatial_index.cpp
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   R-tree implementation with quadratic split for spatial indexing of polygon segments.
 */

#include "spatial_index.h"
#include <algorithm>
#include <cmath>
#include <limits>

// ── AABB ──

/**
 * @name  AABB::AABB
 * @brief Constructs an AABB from two corner points, auto-sorting min/max.
 */
AABB::AABB(double x1, double y1, double x2, double y2)
{
    min_x = std::min(x1, x2);
    min_y = std::min(y1, y2);
    max_x = std::max(x1, x2);
    max_y = std::max(y1, y2);
}

/**
 * @name  AABB::area
 * @brief Returns the area of the bounding box.
 */
double AABB::area() const
{
    if (max_x < min_x || max_y < min_y) return 0.0;
    return (max_x - min_x) * (max_y - min_y);
}

/**
 * @name  AABB::enlargement
 * @brief Computes how much this AABB's area would increase if expanded to include another AABB.
 */
double AABB::enlargement(const AABB& other) const
{
    double new_min_x = std::min(min_x, other.min_x);
    double new_min_y = std::min(min_y, other.min_y);
    double new_max_x = std::max(max_x, other.max_x);
    double new_max_y = std::max(max_y, other.max_y);
    double new_area = (new_max_x - new_min_x) * (new_max_y - new_min_y);
    return new_area - area();
}

/**
 * @name  AABB::expand
 * @brief Expands this AABB in-place to encompass another AABB.
 */
void AABB::expand(const AABB& other)
{
    min_x = std::min(min_x, other.min_x);
    min_y = std::min(min_y, other.min_y);
    max_x = std::max(max_x, other.max_x);
    max_y = std::max(max_y, other.max_y);
}

/**
 * @name  AABB::overlaps
 * @brief Returns true if this AABB overlaps with another AABB.
 */
bool AABB::overlaps(const AABB& other) const
{
    return !(max_x < other.min_x || min_x > other.max_x ||
             max_y < other.min_y || min_y > other.max_y);
}

/**
 * @name  AABB::from_segment
 * @brief Creates an AABB that tightly bounds the segment between two nodes.
 */
AABB AABB::from_segment(Node* a, Node* b)
{
    return AABB(a->x, a->y, b->x, b->y);
}

// ── RTreeNode ──

RTreeNode::~RTreeNode()
{
    if (!is_leaf) {
        for (auto* child : children) {
            delete child;
        }
    }
}

/**
 * @name  RTreeNode::recompute_bbox
 * @brief Recalculates this node's bounding box from its children or entries.
 */
void RTreeNode::recompute_bbox()
{
    bbox = AABB(); // reset to empty
    if (is_leaf) {
        for (const auto& box : entry_boxes) {
            bbox.expand(box);
        }
    } else {
        for (auto* child : children) {
            bbox.expand(child->bbox);
        }
    }
}

// ── SpatialGrid (R-tree) ──

SpatialGrid::~SpatialGrid()
{
    delete root;
}

/**
 * @name  SpatialGrid::build
 * @brief Builds the R-tree by inserting all segments from all polygon rings.
 */
void SpatialGrid::build(const std::vector<Ring>& rings)
{
    delete root;
    root = new RTreeNode(true); // start with empty leaf

    for (const auto& ring : rings) {
        if (ring.vertices.size < 2) continue;
        Node* cur = ring.vertices.head;
        do {
            insert(cur, cur->next);
            cur = cur->next;
        } while (cur != ring.vertices.head);
    }
}

/**
 * @name  SpatialGrid::choose_leaf
 * @brief Traverses the tree to find the leaf node whose bounding box needs least enlargement to fit the given box.
 */
RTreeNode* SpatialGrid::choose_leaf(RTreeNode* node, const AABB& box)
{
    if (node->is_leaf) return node;

    // Pick child whose bbox needs least enlargement
    double best_enlarge = std::numeric_limits<double>::max();
    double best_area = std::numeric_limits<double>::max();
    RTreeNode* best = node->children[0];

    for (auto* child : node->children) {
        double enlarge = child->bbox.enlargement(box);
        double a = child->bbox.area();
        if (enlarge < best_enlarge || (enlarge == best_enlarge && a < best_area)) {
            best_enlarge = enlarge;
            best_area = a;
            best = child;
        }
    }

    return choose_leaf(best, box);
}

/**
 * @name             SpatialGrid::split_node
 * @brief            Splits an overflowing R-tree node using the quadratic split algorithm.
 *                   Picks two seed entries that waste the most area when combined, then assigns
 *                   remaining entries to the group whose bounding box grows least.
 * @time_complexity  O(n^2) where n is the number of entries in the node.
 */
RTreeNode* SpatialGrid::split_node(RTreeNode* node)
{
    RTreeNode* new_node = new RTreeNode(node->is_leaf);

    if (node->is_leaf) {
        int n = (int)node->entries.size();

        // Pick seeds: pair with maximum wasted area
        int seed1 = 0, seed2 = 1;
        double max_waste = -1e18;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                AABB combined = node->entry_boxes[i];
                combined.expand(node->entry_boxes[j]);
                double waste = combined.area() - node->entry_boxes[i].area()
                             - node->entry_boxes[j].area();
                if (waste > max_waste) {
                    max_waste = waste;
                    seed1 = i;
                    seed2 = j;
                }
            }
        }

        // Group 1 gets seed1, group 2 gets seed2
        std::vector<Segment> entries1, entries2;
        std::vector<AABB> boxes1, boxes2;

        entries1.push_back(node->entries[seed1]);
        boxes1.push_back(node->entry_boxes[seed1]);
        AABB bbox1 = node->entry_boxes[seed1];

        entries2.push_back(node->entries[seed2]);
        boxes2.push_back(node->entry_boxes[seed2]);
        AABB bbox2 = node->entry_boxes[seed2];

        // Assign remaining entries
        for (int i = 0; i < n; i++) {
            if (i == seed1 || i == seed2) continue;

            // If one group needs all remaining to reach MIN_ENTRIES, force assign
            int remaining = n - (int)entries1.size() - (int)entries2.size();
            if ((int)entries1.size() + remaining <= RTreeNode::MIN_ENTRIES) {
                entries1.push_back(node->entries[i]);
                boxes1.push_back(node->entry_boxes[i]);
                bbox1.expand(node->entry_boxes[i]);
                continue;
            }
            if ((int)entries2.size() + remaining <= RTreeNode::MIN_ENTRIES) {
                entries2.push_back(node->entries[i]);
                boxes2.push_back(node->entry_boxes[i]);
                bbox2.expand(node->entry_boxes[i]);
                continue;
            }

            // Assign to group with less enlargement
            double e1 = bbox1.enlargement(node->entry_boxes[i]);
            double e2 = bbox2.enlargement(node->entry_boxes[i]);
            if (e1 < e2 || (e1 == e2 && bbox1.area() <= bbox2.area())) {
                entries1.push_back(node->entries[i]);
                boxes1.push_back(node->entry_boxes[i]);
                bbox1.expand(node->entry_boxes[i]);
            } else {
                entries2.push_back(node->entries[i]);
                boxes2.push_back(node->entry_boxes[i]);
                bbox2.expand(node->entry_boxes[i]);
            }
        }

        node->entries = std::move(entries1);
        node->entry_boxes = std::move(boxes1);
        node->bbox = bbox1;

        new_node->entries = std::move(entries2);
        new_node->entry_boxes = std::move(boxes2);
        new_node->bbox = bbox2;

    } else {
        // Internal node split
        int n = (int)node->children.size();

        int seed1 = 0, seed2 = 1;
        double max_waste = -1e18;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                AABB combined = node->children[i]->bbox;
                combined.expand(node->children[j]->bbox);
                double waste = combined.area() - node->children[i]->bbox.area()
                             - node->children[j]->bbox.area();
                if (waste > max_waste) {
                    max_waste = waste;
                    seed1 = i;
                    seed2 = j;
                }
            }
        }

        std::vector<RTreeNode*> group1, group2;
        group1.push_back(node->children[seed1]);
        AABB bbox1 = node->children[seed1]->bbox;
        group2.push_back(node->children[seed2]);
        AABB bbox2 = node->children[seed2]->bbox;

        for (int i = 0; i < n; i++) {
            if (i == seed1 || i == seed2) continue;

            int remaining = n - (int)group1.size() - (int)group2.size();
            if ((int)group1.size() + remaining <= RTreeNode::MIN_ENTRIES) {
                group1.push_back(node->children[i]);
                bbox1.expand(node->children[i]->bbox);
                continue;
            }
            if ((int)group2.size() + remaining <= RTreeNode::MIN_ENTRIES) {
                group2.push_back(node->children[i]);
                bbox2.expand(node->children[i]->bbox);
                continue;
            }

            double e1 = bbox1.enlargement(node->children[i]->bbox);
            double e2 = bbox2.enlargement(node->children[i]->bbox);
            if (e1 < e2 || (e1 == e2 && bbox1.area() <= bbox2.area())) {
                group1.push_back(node->children[i]);
                bbox1.expand(node->children[i]->bbox);
            } else {
                group2.push_back(node->children[i]);
                bbox2.expand(node->children[i]->bbox);
            }
        }

        node->children = std::move(group1);
        node->bbox = bbox1;

        new_node->children = std::move(group2);
        new_node->bbox = bbox2;
    }

    return new_node;
}

/**
 * @name  SpatialGrid::handle_overflow
 * @brief Splits an overflowing node and propagates the split upward, creating a new root if needed.
 */
void SpatialGrid::handle_overflow(RTreeNode* node, std::vector<RTreeNode*>& path)
{
    RTreeNode* new_node = split_node(node);

    if (path.empty()) {
        // node is root — create new root
        RTreeNode* new_root = new RTreeNode(false);
        new_root->children.push_back(node);
        new_root->children.push_back(new_node);
        new_root->recompute_bbox();
        root = new_root;
    } else {
        // Add new_node as sibling in parent
        RTreeNode* parent = path.back();
        parent->children.push_back(new_node);
        parent->recompute_bbox();

        if (parent->count() > RTreeNode::MAX_ENTRIES) {
            path.pop_back();
            handle_overflow(parent, path);
        }
    }
}

/**
 * @name             SpatialGrid::insert
 * @brief            Inserts a segment into the R-tree, splitting nodes as needed on overflow.
 * @time_complexity  O(log V) amortized.
 */
void SpatialGrid::insert(Node* from, Node* to)
{
    AABB box = AABB::from_segment(from, to);
    Segment seg{from, to};

    if (!root) {
        root = new RTreeNode(true);
    }

    // Build path from root to leaf
    std::vector<RTreeNode*> path;
    RTreeNode* node = root;
    while (!node->is_leaf) {
        path.push_back(node);
        // Pick child with least enlargement
        double best_enlarge = std::numeric_limits<double>::max();
        double best_area = std::numeric_limits<double>::max();
        RTreeNode* best = node->children[0];
        for (auto* child : node->children) {
            double enlarge = child->bbox.enlargement(box);
            double a = child->bbox.area();
            if (enlarge < best_enlarge || (enlarge == best_enlarge && a < best_area)) {
                best_enlarge = enlarge;
                best_area = a;
                best = child;
            }
        }
        node = best;
    }

    // Insert into leaf
    node->entries.push_back(seg);
    node->entry_boxes.push_back(box);
    node->bbox.expand(box);

    // Update bboxes along path
    for (auto* p : path) {
        p->bbox.expand(box);
    }

    // Handle overflow
    if (node->count() > RTreeNode::MAX_ENTRIES) {
        handle_overflow(node, path);
    }
}

/**
 * @name  SpatialGrid::find_leaf
 * @brief Recursively searches the R-tree for the leaf node containing a specific segment.
 */
RTreeNode* SpatialGrid::find_leaf(RTreeNode* node, const Segment& seg, const AABB& box,
                                  std::vector<RTreeNode*>& path)
{
    if (node->is_leaf) {
        for (size_t i = 0; i < node->entries.size(); i++) {
            if (node->entries[i] == seg) {
                return node;
            }
        }
        return nullptr;
    }

    for (auto* child : node->children) {
        if (child->bbox.overlaps(box)) {
            path.push_back(node);
            RTreeNode* result = find_leaf(child, seg, box, path);
            if (result) return result;
            path.pop_back();
        }
    }
    return nullptr;
}

/**
 * @name  SpatialGrid::condense_tree
 * @brief Rebalances the R-tree after removal by eliminating empty nodes and reinserting entries
 *        from underflowing leaf nodes.
 */
void SpatialGrid::condense_tree(std::vector<RTreeNode*>& path)
{
    // Collect entries from underflowing nodes to reinsert
    std::vector<Segment> reinsert_segs;
    std::vector<AABB> reinsert_boxes;

    while (!path.empty()) {
        RTreeNode* parent = path.back();
        path.pop_back();

        // Remove empty or underflowing children
        std::vector<RTreeNode*> keep;
        for (auto* child : parent->children) {
            if (child->count() == 0) {
                delete child;
            } else if (child->count() < RTreeNode::MIN_ENTRIES && parent != root) {
                // Collect entries for reinsertion
                if (child->is_leaf) {
                    for (size_t i = 0; i < child->entries.size(); i++) {
                        reinsert_segs.push_back(child->entries[i]);
                        reinsert_boxes.push_back(child->entry_boxes[i]);
                    }
                }
                // For internal nodes, collect leaf entries recursively
                // (simplified: just keep underflowing internal nodes)
                if (!child->is_leaf) {
                    keep.push_back(child);
                } else {
                    delete child;
                }
            } else {
                keep.push_back(child);
            }
        }
        parent->children = std::move(keep);
        parent->recompute_bbox();
    }

    // If root has only one child, make that child the root
    while (root && !root->is_leaf && root->children.size() == 1) {
        RTreeNode* old_root = root;
        root = root->children[0];
        old_root->children.clear(); // prevent recursive delete
        delete old_root;
    }

    // Reinsert collected entries
    for (size_t i = 0; i < reinsert_segs.size(); i++) {
        insert(reinsert_segs[i].from, reinsert_segs[i].to);
    }
}

/**
 * @name  SpatialGrid::remove
 * @brief Removes a segment from the R-tree and condenses the tree to maintain balance.
 */
void SpatialGrid::remove(Node* from, Node* to)
{
    if (!root) return;

    Segment seg{from, to};
    AABB box = AABB::from_segment(from, to);

    std::vector<RTreeNode*> path;
    RTreeNode* leaf = find_leaf(root, seg, box, path);

    if (!leaf) return;

    // Remove the entry from the leaf
    for (size_t i = 0; i < leaf->entries.size(); i++) {
        if (leaf->entries[i] == seg) {
            leaf->entries.erase(leaf->entries.begin() + i);
            leaf->entry_boxes.erase(leaf->entry_boxes.begin() + i);
            leaf->recompute_bbox();
            break;
        }
    }

    // Condense tree
    path.push_back(leaf); // not actually used this way -- condense from parent
    // Actually, condense needs the path from root to leaf's parent
    // path already has root-to-parent, let's use it
    path.pop_back(); // remove leaf itself

    // Recompute bboxes and handle underflow
    if (!path.empty()) {
        condense_tree(path);
    }

    // Handle root with no children
    if (root && !root->is_leaf && root->children.empty()) {
        delete root;
        root = new RTreeNode(true);
    }
}

void SpatialGrid::query_recursive(RTreeNode* node, const AABB& query_box,
                                  std::vector<Segment>& result,
                                  const std::unordered_set<Segment, SegmentHash>& exclude) const
{
    if (!node->bbox.overlaps(query_box)) return;

    if (node->is_leaf) {
        for (size_t i = 0; i < node->entries.size(); i++) {
            if (node->entry_boxes[i].overlaps(query_box)) {
                if (exclude.count(node->entries[i]) == 0) {
                    result.push_back(node->entries[i]);
                }
            }
        }
    } else {
        for (auto* child : node->children) {
            query_recursive(child, query_box, result, exclude);
        }
    }
}

/**
 * @name  SpatialGrid::query
 * @brief Returns all segments whose bounding box overlaps the given query rectangle,
 *        excluding segments in the exclude set.
 */
void SpatialGrid::query(double qx1, double qy1, double qx2, double qy2,
                        std::vector<Segment>& result,
                        const std::unordered_set<Segment, SegmentHash>& exclude) const
{
    result.clear();
    if (!root) return;

    AABB query_box(qx1, qy1, qx2, qy2);
    query_recursive(root, query_box, result, exclude);
}