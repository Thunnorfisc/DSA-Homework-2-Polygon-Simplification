/**
 * @name    symmetric_diff.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Computation of symmetric difference area between original and simplified polygons
 *          to measure simplification quality.
 */

#pragma once
#include <vector>
#include <utility>

/**
 * @name             compute_symmetric_difference
 * @brief            Computes the area of the symmetric difference between two simple polygons
 *                   using fan-triangulation of the original and Sutherland-Hodgman clipping.
 * @time_complexity  O(n * m) where n is the original vertex count and m is the simplified vertex count.
 */
double compute_symmetric_difference(
    const std::vector<std::pair<double, double>>& original,
    const std::vector<std::pair<double, double>>& simplified);