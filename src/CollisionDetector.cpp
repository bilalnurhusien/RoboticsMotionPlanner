#include <sstream>
#include <string>
#include <limits> 

using namespace std;

#include "../include/CollisionDetector.hpp"
#include "../include/Helpers.hpp"

CollisionDetector::CollisionDetector(const std::vector<sf::ConvexShape> polygonObstacles)
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


bool CollisionDetector::IsCollision(point_type p)
{
    bool retVal = false;
    float closestObstacle = std::numeric_limits<float>::max();

    if (!IsInWorkSpace(p.get<0>() - ShapeRadiusSize, p.get<1>() - ShapeRadiusSize))
    {        
        return true;
    }

    for (uint32_t i = 0; i < m_collisionObstacles.size(); ++i)
    {
        float dist = boost::geometry::distance(p, m_collisionObstacles[i]);
        
        if (closestObstacle > dist)
        {
            closestObstacle = dist;
        }

        if (dist < 0.1)
        {
            retVal = true;
            break;
        }
    }
    
    return retVal;
}

bool CollisionDetector::IsPathCollision(point_type p1, point_type p2)
{
    std::ostringstream out;    
    out << "LINESTRING(";
    out << to_string(p1.get<0>());
    out << " ";
    out << to_string(p1.get<1>());
    out << ", ";
    out << to_string(p2.get<0>());
    out << " ";
    out << to_string(p2.get<1>());    
    out << ")";
    boost::geometry::model::linestring<point_type> ls;
    boost::geometry::read_wkt(out.str(), ls);

    for (uint32_t i = 0; i < m_collisionObstacles.size(); ++i)
    {
        std::deque<point_type> output;
        boost::geometry::intersection(ls, m_collisionObstacles[i], output);
        
        if (!output.empty())
        {
            return true;
        }
    }
    
    return false;
}