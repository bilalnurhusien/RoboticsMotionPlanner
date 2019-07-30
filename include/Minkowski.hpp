#pragma once

#include "Types.hpp"

/**
 * @brief Calculate the Minkowksi difference between a robot and list of polygon obstacles
 * @param robot - robot shape
 * @param polygonObstacles - polygon obstacles
 * @param minkowskiObstacles - Minkowski C-obstacles
 * @return true (success), false (failure)
 */
bool MinkowskiDifference(const sf::CircleShape& robot,
                         const std::vector<sf::CircleShape>& polygonObstacles,
                         std::vector<sf::ConvexShape>& minkowskiObstacles);