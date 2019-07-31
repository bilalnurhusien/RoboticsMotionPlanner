#include <sstream>
#include <string>
#include <limits> 

using namespace std;

#include "../include/CollisionDetector.hpp"
#include "../include/Helpers.hpp"

CollisionDetector::CollisionDetector(const std::vector<sf::CircleShape> polygonObstacles,
                                     const sf::CircleShape& robot)
    : m_robot(robot)
{
    uint32_t vectorSize = polygonObstacles.size();
    
    for (uint32_t i = 0; i < vectorSize; ++i)
    {
        uint32_t verticesCount = polygonObstacles[i].getPointCount();
        std::ostringstream out;    
        out << "POLYGON((";
        
        for (uint32_t j = 0; j <= verticesCount; ++j)
        {
            sf::Vector2f vertex = polygonObstacles[i].getPoint(j % verticesCount);
            out << to_string(polygonObstacles[i].getPosition().x + vertex.x) << " " << to_string(polygonObstacles[i].getPosition().y + vertex.y);
            
            if (j != verticesCount)
            {
                out << ",";
            }
        }
        
        out << "))";

#ifdef DEBUG
        cout << out.str() << endl;
#endif
        polygon_type poly;
        bg::read_wkt(
            out.str(),
            poly);
            
        m_collisionObstacles.push_back(poly);
    }
}

CollisionDetector::~CollisionDetector()
{
}

std::vector<bgm::linestring<point_type> > GetLineSegments(const sf::CircleShape& shape)
{
    std::vector<bgm::linestring<point_type> > linesegments;
    uint32_t verticesCount = shape.getPointCount();
        
    for (uint32_t j = 0; j < verticesCount; ++j)
    {
        std::ostringstream out;
        sf::Vector2f vertex1 = shape.getPoint(j);
        sf::Vector2f vertex2 = shape.getPoint((j + 1) % verticesCount);

        out << "LINESTRING(";
        out << to_string(shape.getPosition().x + vertex1.x);
        out << " ";
        out << to_string(shape.getPosition().y + vertex1.y);
        out << ", ";
        out << to_string(shape.getPosition().x + vertex2.x);
        out << " ";
        out << to_string(shape.getPosition().y + vertex2.y);    
        out << ")";
#ifdef DEBUG
        cout << "==================== " << j <<endl;
        cout << out.str() << endl;
        cout << "====================" <<endl;
#endif
        bgm::linestring<point_type> ls;
        bg::read_wkt(out.str(), ls);
        linesegments.push_back(ls);
    }
    
    return linesegments;
}

bool CollisionDetector::IsCollision(const point_type& p)
{
    CenterRobotPosition(m_robot, p);
    
    std::vector<bgm::linestring<point_type> > robotLineSegments = GetLineSegments(m_robot);
    
    if (!IsInWorkSpace(p.get<0>() - ShapeRadiusSize, p.get<1>() - ShapeRadiusSize))
    {        
        return true;
    }

    for (uint32_t i = 0; i < m_collisionObstacles.size(); ++i)
    {
        for (uint32_t j = 0; j < robotLineSegments.size(); ++j)
        {
            std::deque<point_type> output;

            boost::geometry::intersection(robotLineSegments[j], m_collisionObstacles[i], output);
        
            if (!output.empty())
            {
                return true;
            }
        }
    }

    return false;
}

bool CollisionDetector::IsPathCollision(point_type p1, point_type p2)
{
    //
    // Use a parameterization to create points along line segment
    //
    // x(t) = a1+(b1−a1)t,  0≤t≤1.
    // y(t) = a2+(b2−a2)t,  0≤t≤1.
    //
    std::vector<point_type> vecP;

    for (float t = 0; t <= 1.f; t+=0.2f)
    {
        point_type newPoint;
        newPoint.set<0>(p1.get<0>() + (p2.get<0>() - p1.get<0>()) * t);
        newPoint.set<1>(p1.get<1>() + (p2.get<1>() - p1.get<1>()) * t);
        vecP.push_back(newPoint);
    }
    
    for (uint32_t i = 0; i < vecP.size(); ++i)
    {
        if (IsCollision(vecP[i]))
        {
            return true;
        }
    }
    return false;
}