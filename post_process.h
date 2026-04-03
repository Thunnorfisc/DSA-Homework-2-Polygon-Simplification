#pragma once
#include "polygon.h"
#include "symmetric_diff.h"
#include <vector>
#include <utility>

// Post-processing pass: slide each vertex along its area-preserving line
// to minimize total symmetric difference against the original polygon.
// Also removes collinear vertices (zero-cost improvement).
// Returns the number of vertices relocated.
int post_process(std::vector<Ring>& rings,
                 const std::vector<std::vector<std::pair<double, double>>>& original_vertices,
                 int max_iterations = 3);