#include "polygon.h"
#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <algorithm>

Node* CircularDoublyLinkedList::append(double x, double y, int ring_id)
{
	Node* node = new Node{ x, y, ring_id, nullptr, nullptr, 0, false };

	if (head == nullptr) {
		node->prev = node;
		node->next = node;
		head = node;
	}
	else {
		Node* tail = head->prev;
		tail->next = node;
		node->prev = tail;
		node->next = head;
		head->prev = node;
	}

	size++;
	return node;
}

void CircularDoublyLinkedList::remove(Node* node)
{
	if (size <= 1) return;

	node->prev->next = node->next;
	node->next->prev = node->prev;

	if (head == node) head = node->next;

	node->deleted = true;
	size--;
}

Node* CircularDoublyLinkedList::insert_after(Node* after, double x, double y, int ring_id)
{
	Node* node = new Node{ x, y, ring_id, after, after->next, 0, false };
	after->next->prev = node;
	after->next = node;
	size++;
	return node;
}

std::vector<Ring> parse_csv(const std::string& filename)
{
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Error: cannot open " << filename << std::endl;
		exit(1);
	}

	std::map<int, std::vector<std::pair<int, std::pair<double, double>>>> ring_data;

	std::string line;
	std::getline(file, line);

	while (std::getline(file, line)) {
		if (line.empty()) continue;

		std::stringstream ss(line);
		std::string token;

		std::getline(ss, token, ',');
		int ring_id = std::stoi(token);

		std::getline(ss, token, ',');
		int vertex_id = std::stoi(token);

		std::getline(ss, token, ',');
		double x = std::stod(token);

		std::getline(ss, token, ',');
		double y = std::stod(token);

		ring_data[ring_id].push_back({ vertex_id, {x, y} });
	}

	std::vector<Ring> rings;

	for (auto& [rid, verts] : ring_data) {
		std::sort(verts.begin(), verts.end());

		Ring ring;
		ring.ring_id = rid;
		ring.original_area = 0.0;

		for (auto& [vid, coords] : verts) ring.vertices.append(coords.first, coords.second, rid);

		rings.push_back(std::move(ring));
	}
	return rings;
}

double compute_signed_area(const Ring& ring)
{
	if (ring.vertices.size < 3) return 0.0;

	double area = 0.0;
	Node* current = ring.vertices.head;

	do {
		Node* next = current->next;
		area += (current->x * next->y) - (next->x * current->y);
		current = next;
	} while (current != ring.vertices.head);

	return area / 2.0;
}

void print_output(const std::vector<Ring>& rings, double input_area, double output_area, double displacement,
                  double offset_x, double offset_y)
{
	std::printf("ring_id,vertex_id,x,y\n");

	for (const auto& ring : rings) {
		Node* current = ring.vertices.head;
		int vid = 0;

		do {
			std::printf("%d,%d,%.15g,%.15g\n", ring.ring_id, vid,
			            current->x + offset_x, current->y + offset_y);
			vid++;
			current = current->next;
		} while (current != ring.vertices.head);
	}

	std::printf("Total signed area in input: %e\n", input_area);
	std::printf("Total signed area in output: %e\n", output_area);
	std::printf("Total areal displacement: %e\n", displacement);
}