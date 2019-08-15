#pragma once

#include <iostream>

#include "Types.hpp"

/**
 * @class CollisionDetector
 * @file CollisionDetector.hpp
 * @brief Detects if collisions exist
 */
class CollisionDetector
{
   
public:
    
    /**
     * @brief C'tor
     * @param polygonObstacles - obstacles
     * @param robot - robot
     * @return 
     */
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
     * @brief Detects if path intersects any of the polygon obstacles
     * @param p1 - point 1
     * @param p2 - point 2
     * @return true or false
     */
    bool IsPathCollision(point_type p1, point_type p2);

private:
std::vector<polygon_type> m_collisionObstacles;
std::shared_ptr<sf::Shape> m_pRobot;
};

