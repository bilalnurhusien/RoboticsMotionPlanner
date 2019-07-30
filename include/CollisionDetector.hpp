#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <iostream>
#include <boost/geometry.hpp>

#include "Types.hpp"

/**
 * @class CollisionDetector
 * @file CollisionDetector.hpp
 * @brief Detects if collisions exist
 */
class CollisionDetector
{
   
public:
    
    explicit CollisionDetector(const std::vector<sf::ConvexShape> polygonObstacles);
    ~CollisionDetector();

    /**
     * @brief Detects if point intersects any of the polygon obstacles
     * @param p
     * @return true or false
     */
    bool IsCollision(point_type p);
    /**
     * @brief Detects if line segment intersects any of the polygon obstacles
     * @param p1 - point 1
     * @param p2 - point 2
     * @return 
     */
    bool IsPathCollision(point_type p1, point_type p2);

private:
std::vector<polygon_type> m_collisionObstacles;
};

