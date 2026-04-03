#pragma once
#include <cstdint>
#include <string>
#include <vector>

struct Node {
    double x, y;
    int ring_id;
    Node* prev;
    Node* next;
    uint64_t version;  // incremented when neighbors change
    bool deleted;
};

class CircularDoublyLinkedList {
public:
    Node* head = nullptr;
    int size = 0;

    Node* append(double x, double y, int ring_id);

    void remove(Node* node);

    Node* insert_after(Node* after, double x, double y, int ring_id);
};

struct Ring {
    CircularDoublyLinkedList vertices;
    int ring_id{};
    double original_area{};
};

std::vector<Ring> parse_csv(const std::string& filename);
double compute_signed_area(const Ring& ring);
void print_output(const std::vector<Ring>& rings, double input_area, double output_area, double displacement,
                  double offset_x = 0.0, double offset_y = 0.0);