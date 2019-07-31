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
    
    explicit CollisionDetector(const std::vector<std::shared_ptr<sf::Shape> >& polygonObstacles,
                               const std::shared_ptr<sf::Shape>& robot);

    ~CollisionDetector();

    /**
     * @brief Detects if robot intersects any of the polygon obstacles
     * @param robot
     * @return true or false
     */
    bool IsCollision(const point_type& p);
    /**
     * @brief Detects if line segment intersects any of the polygon obstacles
     * @param p1 - point 1
     * @param p2 - point 2
     * @return 
     */
    bool IsPathCollision(point_type p1, point_type p2);

private:
std::vector<polygon_type> m_collisionObstacles;
std::shared_ptr<sf::Shape> m_pRobot;
};

