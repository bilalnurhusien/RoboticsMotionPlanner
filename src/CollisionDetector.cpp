#include <sstream>
#include <string>
#include <limits> 
#include <exception>

using namespace std;

#include "../include/CollisionDetector.hpp"
#include "../include/Helpers.hpp"

CollisionDetector::CollisionDetector(const vector<shared_ptr<sf::Shape> >& polygonObstacles,
                                     const shared_ptr<sf::Shape>& pRobot)
    : m_pRobot(nullptr)
{
    sf::CircleShape* circleShape = nullptr;
    sf::ConvexShape* convexShape = nullptr;

    /*
     * Make a copy of the robot for detecting collisions
     */ 
    if (circleShape = dynamic_cast<sf::CircleShape*>(pRobot.get()))
    {
        m_pRobot = make_shared<sf::CircleShape>(*circleShape);
    }
    else if (convexShape = dynamic_cast<sf::ConvexShape*>(pRobot.get()))
    {
        m_pRobot = make_shared<sf::ConvexShape>(*convexShape);
    }
    else
    {
        throw "Invalid shape type";
    }
    
    uint32_t vectorSize = polygonObstacles.size();
    
    for (uint32_t i = 0; i < vectorSize; ++i)
    {
        uint32_t verticesCount = polygonObstacles[i]->getPointCount();
        std::ostringstream out;    
        out << "POLYGON((";
        
        for (uint32_t j = 0; j <= verticesCount; ++j)
        {
            sf::Vector2f vertex = polygonObstacles[i]->getTransform().transformPoint(polygonObstacles[i]->getPoint(j % verticesCount).x, polygonObstacles[i]->getPoint(j % verticesCount).y);
            out << ToStringSetPrecision(vertex.x) << " " << ToStringSetPrecision(vertex.y);
            
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

bool CollisionDetector::IsCollision(const point_type& p)
{
    if (!IsInWorkSpace(p.get<0>(), p.get<1>()))
    {
        return true;
    }
    
    CenterPosition(m_pRobot.get(), p);

    uint32_t verticesCount = m_pRobot->getPointCount();
    std::ostringstream out;    
    out << "POLYGON((";
    
    for (uint32_t j = 0; j <= verticesCount; ++j)
    {
        sf::Vector2f vertex = m_pRobot->getTransform().transformPoint(m_pRobot->getPoint(j % verticesCount).x, m_pRobot->getPoint(j % verticesCount).y);
        out << ToStringSetPrecision(vertex.x) << " " << ToStringSetPrecision(vertex.y);
        
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

    for (uint32_t i = 0; i < m_collisionObstacles.size(); ++i)
    {
        std::deque<polygon_type> output;

        boost::geometry::intersection(poly, m_collisionObstacles[i], output);
    
        if (!output.empty())
        {
            return true;
        }
    }

    return false;
}

bool CollisionDetector::IsPathCollision(point_type p1, point_type p2)
{
    //
    // Use a parameterization to discretize the path
    //
    // x(t) = a1+(a1−b1)t,  0≤t≤1.
    // y(t) = a2+(a2−b2)t,  0≤t≤1.
    // theta(t) = a3+(a3−b3)t,  0≤t≤1.
    //
    std::vector<point_type> vecP;

    point_type halfPoint;
    halfPoint.set<0>(p2.get<0>() + (p1.get<0>() - p2.get<0>()) * 0.5);
    halfPoint.set<1>(p2.get<1>() + (p1.get<1>() - p2.get<1>()) * 0.5);
    halfPoint.set<2>(p2.get<2>() + (p1.get<2>() - p2.get<2>()) * 0.5);

    if (IsCollision(halfPoint))
    {
        return true;
    }

    for (float t = 0; t <= 1.f; t+=0.1f)
    {
        point_type newPoint;
        newPoint.set<0>(p2.get<0>() + (p1.get<0>() - p2.get<0>()) * t);
        newPoint.set<1>(p2.get<1>() + (p1.get<1>() - p2.get<1>()) * t);
        newPoint.set<2>(p2.get<2>() + (p1.get<2>() - p2.get<2>()) * t);
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