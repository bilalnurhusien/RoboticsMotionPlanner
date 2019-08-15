#pragma once

#include "Types.hpp"
#include <sstream>
#include <string>
#include <fstream>
#include <memory>

/**
  * @brief Print help messages
  */
void PrintHelp();

/**
 * @brief Process arguments
 * @param argc - number of arguments
 * @param argvp - array of argument strings
 * @param robot -  robot to initialize
 * @param polygonObstacles - polygon obstacles in workspace
 * @param fullScreen - display window as fullscreen
 * @param numNodes - number of nodes
 * @param nodeDistance - minimum distance between nodes
 * @return 
 */
bool ProcessArguments(int argc,
                      char* argvp[],
                      std::shared_ptr<sf::Shape>& robot,
                      std::vector<std::shared_ptr<sf::Shape> >& polygonObstacles,
                      bool& fullScreen,
                      uint32_t& numNodes,
                      float& nodeDistance);
                      
/**
  * Get opposite angles (180 deg rotation of an angle)
  */
/**
 * @brief Get opposite angles (180 deg rotation of an angle)
 * @param angles - angles to rotate
 * @param oppositeAngles - rotated angles
 */
void GetOppositeAngles(const std::vector<float>& angles,
                       std::vector<float>& oppositeAngles);

/**
 * @brief Get normal vectors of all edges in a polygon
 * @param shape - polygon shape
 * @param normalVectors - normal vectors
 * @param outwardNormal - outward normal or inward normal
 */
void GetNormalVectors(const sf::CircleShape& shape,
                      std::vector<std::vector<sf::Vertex> >& normalVectors,
                      bool outwardNormal = true);

/**
 * @brief Get angle of normal vectors
 * @param normalVectors - normal vectors
 * @param angleOfNormalVectors - angle of each vector
 * @return 
 */
bool GetAngleOfNormalVectors(const std::vector<std::vector<sf::Vertex> >& normalVectors,
                             std::vector<float>& angleOfNormalVectors);

/**
 * Check if point is in workspace
 */
bool IsInWorkSpace(float x, float y);

/**
 * Return a string with a limit number of decimal places of a float or double
 *
 */
template <typename T>
std::string ToStringSetPrecision(const T a_value, const int n = 4)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

/**
 * @brief Center robot position
 * @param robot - robot
 * @param p - point in which to center
 */
void CenterPosition(sf::Shape* const pRobot, const point_type& p);