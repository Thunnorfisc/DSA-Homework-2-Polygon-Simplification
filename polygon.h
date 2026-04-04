/**
 * @name    polygon.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Core polygon data structures and I/O for ring-based polygon representation.
 */

#pragma once
#include <cstdint>
#include <string>
#include <vector>

/**
 * @name  Node
 * @brief A vertex in a circular doubly linked list representing a polygon ring.
 */
struct Node {
    double x, y;
    int ring_id;
    Node* prev;
    Node* next;
    uint64_t version;  // incremented when neighbors change
    bool deleted;
};

/**
 * @name       CircularDoublyLinkedList
 * @brief      Circular doubly linked list storing polygon vertices for a single ring.
 * @operations
 *   - append:       Insert a vertex at the tail — O(1) time, O(1) space.
 *   - remove:       Unlink a vertex by pointer — O(1) time, O(1) space.
 *   - insert_after: Insert a new vertex after a given node — O(1) time, O(1) space.
 */
class CircularDoublyLinkedList {
public:
    Node* head = nullptr;
    int size = 0;

    Node* append(double x, double y, int ring_id);

    void remove(Node* node);

    Node* insert_after(Node* after, double x, double y, int ring_id);
};

/**
 * @name  Ring
 * @brief Represents a single polygon ring (outer boundary or hole) with its vertices and metadata.
 */
struct Ring {
    CircularDoublyLinkedList vertices;
    int ring_id{};
    double original_area{};
};

/**
 * @name  parse_csv
 * @brief Parses a CSV file with columns ring_id,vertex_id,x,y into a vector of Ring structures.
 */
std::vector<Ring> parse_csv(const std::string& filename);

/**
 * @name  compute_signed_area
 * @brief Computes the signed area of a ring using the shoelace formula.
 */
double compute_signed_area(const Ring& ring);

/**
 * @name  print_output
 * @brief Prints the simplified polygon as CSV to stdout along with area and displacement statistics.
 */
void print_output(const std::vector<Ring>& rings, double input_area, double output_area, double displacement,
                  double offset_x = 0.0, double offset_y = 0.0);