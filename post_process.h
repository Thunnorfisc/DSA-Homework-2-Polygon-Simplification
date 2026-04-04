/**
 * @name    post_process.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Post-processing pass that refines simplified polygons by sliding vertices to minimize
 *          symmetric difference and removing collinear vertices.
 */

#pragma once
#include "polygon.h"
#include "symmetric_diff.h"
#include <vector>
#include <utility>

/**
 * @name  post_process
 * @brief Iteratively slides each vertex along its neighbor-to-neighbor line to reduce symmetric
 *        difference with the original polygon. Also removes collinear vertices. Returns
 *        the number of vertices relocated.
 */
int post_process(std::vector<Ring>& rings,
                 const std::vector<std::vector<std::pair<double, double>>>& original_vertices,
                 int max_iterations = 3);