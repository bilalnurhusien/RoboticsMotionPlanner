#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
#include <float.h>
#include <algorithm>

#include "../include/Minkowski.hpp"
#include "../include/Helpers.hpp"
#include "../include/Types.hpp"

using namespace std;

/**
 * Shortest distance between two angles.
 */
/**
* @brief Shortest distance between two angles.
* @param alpha - angle 1
* @param beta - angle 2
* @return 
*/
float Distance(float alpha, float beta) {
    float phi = fmod(fabs(beta - alpha),  360.f);
    float distance = phi > 180.0f ? 360.0f - phi : phi;
    return distance;
}

/**
 * @brief Determine if an angle is between two angles
 * @param a - angle a
 * @param b - angle b
 * @param targetAngle - target angle
 * @return 
 */
bool IsAngleBetween(float a, float b, float targetAngle)
{
    float distanceAtoB, distanceAtoTarget, distanceBtoTarget;

    distanceAtoB = Distance(b, a);
    distanceAtoTarget = Distance(targetAngle, a);
    distanceBtoTarget = Distance(targetAngle, b);

#ifdef DEBUG
cout << "a : " << a  << " b: " << b << " targetAngle: " << targetAngle << endl;
cout << "Distance AtoB " << distanceAtoB << endl;
cout << "Distance AtoTarget " << distanceAtoTarget << endl;
cout << "Distance BtoTarget " << distanceBtoTarget << endl;
#endif

    return ((distanceAtoB < 180.0f) && ((distanceAtoTarget + distanceBtoTarget) <= distanceAtoB));
}
 
/**
 * @brief Get next polygon point that is before or after a point
 *        on the unit circle. This is used to calculate the Minkowski
 *        difference of two polygons.
 * @param index - polygon vertex index
 * @param before - consider points before index
 * @param nextType - polygon vertex type to consider
 * @param polygonVertices - list of polygon vertices to consider
 * @param nextPoint - output polygon vertex that was found
 * @return true (success), false (failure)
 */
bool GetNextPolygonPoint(const uint32_t index,
                         const bool before,
                         const PolygonType nextType,
                         const vector<PolygonVertex>& polygonVertices,
                         PolygonVertex& nextPoint)
{
    const int32_t VertexCount = polygonVertices.size();
    int32_t i = index;
    
    do
    {
        if (before)
        {
            i = (i - 1 + VertexCount) % VertexCount;
        }
        else
        {
            i = (i + 1) % VertexCount;
        }

        if (polygonVertices[i].polygonType == nextType)
        {
            nextPoint = polygonVertices[i];

            return true;
        }
    } while ((i != index) &&
             (Distance(polygonVertices[i].normalAngle, polygonVertices[index].normalAngle) < 180.0f));

    return false;
}

/**
 * Calculate the Minkowksi difference between a robot and list of polygon obstacles
 * 
 * Note: This algorithm was taken from the textbook Robot Motion Planning by Jean Claude Latombe et al. pg. 124
 */
/**
 * @brief Calculate the Minkowksi difference between a robot and list of polygon obstacles
 *
 *        Note: This algorithm was taken from the textbook Robot Motion Planning by Jean Claude Latombe et al. pg. 124
 * @param polygonVertices - combined list of polygon robot and obstacle vertices
 * @param minkowskiVertices - output Minkowski C-obstacle vertices
 * @return 
 */
static bool MinkowskiDifference(const vector<PolygonVertex>& polygonVertices,
                                vector<PolygonVertex>& minkowskiVertices)
{       
    uint32_t vertexCount = polygonVertices.size();

    for (int32_t i = 0; i < vertexCount; ++i)
    {
        PolygonVertex vertexBefore;
        PolygonVertex vertexAfter;
        PolygonVertex polygonVertex = polygonVertices[i];
        PolygonType nextPolygonType;
        
        if (polygonVertex.polygonType == PolygonType::Robot)
        {
            nextPolygonType = PolygonType::Obstacle;
        }
        else if (polygonVertex.polygonType == PolygonType::Obstacle)
        {
            nextPolygonType = PolygonType::Robot;
        }

        if (!GetNextPolygonPoint(i, true, nextPolygonType, polygonVertices, vertexBefore))
        {
            continue;
        }
     
        if (!GetNextPolygonPoint(i, false, nextPolygonType, polygonVertices, vertexAfter))
        {
            continue;
        }

        if (IsAngleBetween(vertexBefore.normalAngle, vertexAfter.normalAngle, polygonVertex.normalAngle))
        {
            if (!GetNextPolygonPoint(i, false, polygonVertex.polygonType, polygonVertices, polygonVertex))
            {
                continue;
            }

#ifdef DEBUG
        cout << endl << "============== Found: =================== " << i <<  endl;
        cout << "polygonVertexType: " << polygonVertex.polygonType << " polygonVertexAngle: " << polygonVertex.normalAngle << endl;
        cout << "polygonVertexX: " << polygonVertex.vector.x << " polygonVertexY: " << polygonVertex.vector.y << endl;  
#endif

            PolygonVertex minkowskiVertex;
            float x, y;

            if (vertexAfter.polygonType == PolygonType::Obstacle)
            {
                x = vertexAfter.vector.x - polygonVertex.vector.x;
                y = vertexAfter.vector.y - polygonVertex.vector.y;
            }
            else
            {
                x = polygonVertex.vector.x - vertexAfter.vector.x;
                y = polygonVertex.vector.y - vertexAfter.vector.y;
            }

            /*
             * 57.2957795f = 180 / PI
             */
            minkowskiVertex.vector = sf::Vector2f(x, y);
            minkowskiVertex.normalAngle = fmod(atan2(y, x) * 57.2957795f + 360.0f, 360.0f);
            minkowskiVertices.push_back(minkowskiVertex);            

#ifdef DEBUG
            cout << "New Polygon Point: " << x << " y: " << y << endl;
            cout << "Angle: " << minkowskiVertex.normalAngle << endl;
#endif

        }
    }
    
    sort(minkowskiVertices.begin(), minkowskiVertices.end(), PolygonVertex::CompAngles);
}

bool MinkowskiDifference(const sf::CircleShape& robot,
                         const sf::CircleShape& obstacle,
                         vector<PolygonVertex>& minkowskiVertices)
{
    vector<vector<sf::Vertex> > normalVectors1;
    vector<vector<sf::Vertex> > normalVectors2;
    vector<float> angleOfInwardNormalVectors1;
    vector<float> angleOfNormalVectors2;
    vector<sf::CircleShape> polygonShapesVec;
    vector<PolygonVertex> sortedPolygonVertices;

    minkowskiVertices.clear();
    polygonShapesVec.push_back(robot);
    polygonShapesVec.push_back(obstacle);

    /* Get normal vectors */
    GetNormalVectors(robot, normalVectors1, false);
    GetNormalVectors(obstacle, normalVectors2);

    /* Get angle of normal vectors */
    GetAngleOfNormalVectors(normalVectors1, angleOfInwardNormalVectors1);
    GetAngleOfNormalVectors(normalVectors2, angleOfNormalVectors2);

    MergeAngleOfNormalVectors(polygonShapesVec, angleOfInwardNormalVectors1, angleOfNormalVectors2, sortedPolygonVertices);

    return MinkowskiDifference(sortedPolygonVertices, minkowskiVertices);
}

bool MinkowskiDifference(const sf::CircleShape& robot,
                         const vector<sf::CircleShape>& polygonObstacles,
                         vector<sf::ConvexShape>& minkowskiObstacles)
{
    for (uint32_t j = 0; j < polygonObstacles.size(); ++j)
    {
        sf::ConvexShape minkowskiObstacle;
        vector<PolygonVertex> minkowskiVertices;

        MinkowskiDifference(robot, polygonObstacles[j], minkowskiVertices);
        minkowskiObstacle.setPointCount(minkowskiVertices.size());
        minkowskiObstacle.setPosition(polygonObstacles[j].getPosition().x + ShapeRadiusSize, polygonObstacles[j].getPosition().y + ShapeRadiusSize);
        minkowskiObstacle.setFillColor(sf::Color::Red);

        for (uint32_t i = 0; i < minkowskiVertices.size(); ++i)
        {
#ifdef DEBUG
            cout << "Minkowski Vertex " << i << ":  " << minkowskiVertices[i].vector.x << " " << minkowskiVertices[i].vector.y << endl;
            cout << "Minkowski Angle " << i << ":  " << minkowskiVertices[i].normalAngle << endl << endl;
#endif
            minkowskiObstacle.setPoint(i, minkowskiVertices[i].vector);
        }
        
        minkowskiObstacles.push_back(minkowskiObstacle);
    }
}
