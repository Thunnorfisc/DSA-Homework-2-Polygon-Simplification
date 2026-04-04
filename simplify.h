/**
 * @name    simplify.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Core polygon simplification algorithm using greedy area-preserving vertex collapse.
 */

#pragma once
#include "polygon.h"
#include "placement.h"
#include <vector>

/**
 * @name             simplify_polygon
 * @brief            Iteratively collapses vertex pairs with minimum areal displacement until
 *                   the target vertex count is reached, using a min-heap and R-tree spatial index.
 * @time_complexity  O(V log V) where V is the number of vertices, due to heap operations per collapse.
 * @space_complexity O(V) for the heap and spatial index.
 */
double simplify_polygon(std::vector<Ring>& rings, int target_vertices);