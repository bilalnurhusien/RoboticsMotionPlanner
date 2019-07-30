#include <string.h>
#include <fstream>
#include <stdlib.h> 

#include "../include/Helpers.hpp"

using namespace std;

/**
  * Print help messages
  */
void PrintHelp()
{
    cout << "Usage:" << endl;
    cout << "\t./pathplanning -r <num-of-robot-vertices> -o <num-of-obstacles>" << endl;
    cout << "\t-r: Number of robot vertices" << endl;
    cout << "\t-o: Number of obstacles" << endl;
    cout << "\t-f: Full screen" << endl;
    cout << "Example:\n\t./pathplanning -r 3 -o 5 -f # For a triangle robot with 5 obstacles in full screen workspace" << endl;
}

/* Check if a string is a numerical value */
bool IsNumeric(const std::string& str) {
    try
    {
        size_t sz;
        std::stol(str, &sz);
        return sz == str.size();
    }
    catch (const std::exception& ex)
    {
        // if no conversion could be performed.
        return false;   
    }
}

/**
  * Process arguments
  */
bool ProcessArguments(int argc,
                      char* argvp[],
                      sf::CircleShape& robot,
                      vector<sf::CircleShape>& polygonObstacles,
                      bool& fullScreen)
{
    if (argc < 3)
    {
        PrintHelp();
        return false;
    }

    int32_t robotVertexNum = -1;
    int32_t obstacleNum = -1;

    for (uint32_t i = 1; i < argc; ++i)
    {
        if (strcmp(argvp[i], "-f") == 0)
        {
            fullScreen = true;
            continue;
        }

        if (strcmp(argvp[i], "-r") == 0)
        {
            if ((i + 1) > argc)
            {
                cout << "Missing robot vertex couunt" << endl;

                return false;
            }

            ++i;

            if (!IsNumeric(argvp[i]))
            {
                cout << "Invalid numerical argument: " << argvp[i] << endl;

                PrintHelp();

                return false;
            }

            robotVertexNum = strtol(argvp[i], nullptr, 10);

            if (robotVertexNum < MinNumVertices)
            {
                cout << "Please use a convex polygon\n";

                PrintHelp();

                return false;
            }        

            continue;
        }

        if (strcmp(argvp[i], "-o") == 0)
        {
            if ((i + 1) > argc)
            {
                cout << "Missing obstacle couunt" << endl;
                
                return false;
            }
            
            ++i;

            if (!IsNumeric(argvp[i]))
            {
                cout << "Invalid numerical argument: " << argvp[i] << endl;

                PrintHelp();

                return false;
            }

            obstacleNum = strtol(argvp[i], nullptr, 10);

            if ((obstacleNum < 1) || (obstacleNum > MaxNumObstacles))
            {
                cout << "Please use more than 1 and less than " << MaxNumObstacles + 1 << " obstacles\n";

                PrintHelp();

                return false;
            }    

            continue;
        }
    }
    
    if ((robotVertexNum == -1) ||
        (obstacleNum == -1))
    {
        cout << "Robot or obstacle vertices not specified" << endl;
        return false;
    }

    /* initialize random seed: */
    srand (time(NULL));

    robot = sf::CircleShape (ShapeRadiusSize, robotVertexNum);
    robot.setFillColor(sf::Color::Green);
    robot.setPosition(RobotStartPosX, RobotStartPosY);

    for (uint32_t i = 0; i < obstacleNum; ++i)
    {
        sf::CircleShape obstacle(ShapeRadiusSize, rand() % MaxNumVertices + MinNumVertices);
        obstacle.setFillColor(sf::Color::Blue);
        obstacle.setPosition(ObstacleCoordinates[i][0], ObstacleCoordinates[i][1]);

        polygonObstacles.push_back(obstacle);
    }

    return true;        
}

/**
  * Get opposite angles (180 deg rotation of an angle)
  */
void GetOppositeAngles(const vector<float>& angles,
                       vector<float>& oppositeAngles)
{
    for (uint32_t i = 0; i < angles.size(); ++i)
    {
        oppositeAngles.push_back(fmod(angles[i] + 180, 360.0f));
    }
}

/**
  * Get normal vectors of all edges in a polygon
  */
void GetNormalVectors(const sf::CircleShape& shape,
                      vector<vector<sf::Vertex> >& normalVectors,
                      bool outwardNormal)
{
    vector<Point> vectors;
    vector<Point> midPoints;

    size_t vertexCount = shape.getPointCount();

    for (uint32_t i = 0; i < vertexCount; ++i)
    {    
        Point vector;
        Point midPoint;

        sf::Vector2f vertex = shape.getPoint(i);
        sf::Vector2f nextVertex;

        nextVertex = shape.getPoint((i + 1) % vertexCount);
        vector.x = nextVertex.x - vertex.x;
        vector.y = nextVertex.y - vertex.y;
        midPoint.x = (vertex.x + nextVertex.x) / 2  + shape.getPosition().x;
        midPoint.y = (vertex.y + nextVertex.y) / 2  + shape.getPosition().y;

#ifdef DEBUG
        cout << "Vertex " << i << " : " << vertex.x << ", " << vertex.y << endl;

        cout << "Vector " << i << " : " << vector.x << ", " << vector.y << endl;
            
        cout << "MidPoint vertex " << i << " : " << midPoint.x << ", " << midPoint.y << endl;
        
        cout << "Next Vertex " << (i + 1) % vertexCount  << " : " << nextVertex.x << ", " << nextVertex.y << endl;
#endif

        vectors.push_back(vector);
        midPoints.push_back(midPoint);
    }

    /*
     * Outward normal vector is calculated by using the 2D rotation matrix (270 deg rotation):
     * 
     * Rotation Matrix = [cos(θ) -sin(θ)] = [0  1]
     *                   [sin(θ)  cos(θ)]   [-1 0]
     *
     * If we define dx=x2-x1 and dy=y2-y1, then the normals are <-dy, dx> and <dy, -dx>.
     *
     * For example, rotating a vector <dx, dy> by 270 deg:
     *
     * [cos(θ) -sin(θ)][dx] = [0  1][dx] = [dy ]
     * [sin(θ)  cos(θ)][dy]   [-1 0][dy]   [-dx]
     */
    for (uint32_t i = 0; i < vectors.size(); ++i)
    {
        sf::Vertex v1;
        sf::Vertex v2;

        v1 = sf::Vertex(sf::Vector2f(midPoints[i].x, midPoints[i].y));

        if (outwardNormal)
        {
            v2 = sf::Vertex(sf::Vector2f(midPoints[i].x + vectors[i].y, midPoints[i].y - vectors[i].x));
        }
        else
        {
            v2 = sf::Vertex(sf::Vector2f(midPoints[i].x - vectors[i].y, midPoints[i].y + vectors[i].x));
        }

        normalVectors.push_back(vector<sf::Vertex>());
        normalVectors[i].push_back(v1);
        normalVectors[i].push_back(v2);

#ifdef EXTRA_DEBUG
        cout << "Normal vertex (x1, y1): " << normalVectors[i][0].position.x << " " << normalVectors[i][0].position.y  << endl;
        cout << "Normal vertex (x2, y2): " << normalVectors[i][1].position.x << " " << normalVectors[i][1].position.y  << endl;
        cout << normalVectors.size() << endl;
#endif
    }
}


/*
 * Get angle of normal vectors
 */
bool GetAngleOfNormalVectors(const vector<vector<sf::Vertex> >& normalVectors,
                             vector<float>& angleOfNormalVectors)
{
    angleOfNormalVectors.clear();
    for (uint32_t i  = 0; i < normalVectors.size(); ++i)
    {
        if (normalVectors[i].size() != 2)
        {
            cout << "Invalid number of vertices for normal vector";
            return false;
        }

        float yDiff = (float)normalVectors[i][1].position.y - (float)normalVectors[i][0].position.y;
        float xDiff = (float)normalVectors[i][1].position.x - (float)normalVectors[i][0].position.x;

        /*
         * 57.2957795f = 180 / PI
         */
        float angle = fmod(atan2(yDiff, xDiff) * 57.2957795f + 360.0f, 360.0f);

        angleOfNormalVectors.push_back(angle);

#ifdef DEBUG            
        cout << "Angle of Normal: " << i << ": "<< angle << endl;
#endif
    }
    
    return true;
}


/**
 * Combine list of normal vectors
 */
void MergeAngleOfNormalVectors(const vector<sf::CircleShape>& vecShapes,
                               const vector<float>& angleOfNormalVectors1,
                               const vector<float>& angleOfNormalVectors2,
                               vector<PolygonVertex>& sortedPolygonVertices)
{
    size_t vertexCount1 = vecShapes[0].getPointCount();
    size_t vertexCount2 = vecShapes[1].getPointCount();

    int32_t i = 0;
    int32_t j = 0;

    while (i < angleOfNormalVectors1.size())
    {
        PolygonVertex polygonVertex;
        polygonVertex.normalAngle = angleOfNormalVectors1[i];
        polygonVertex.vector = vecShapes[0].getPoint(i);
        polygonVertex.polygonType = PolygonType::Robot;
        sortedPolygonVertices.push_back(polygonVertex);
        ++i;
    }

    while (j < angleOfNormalVectors2.size())
    {       
        PolygonVertex polygonVertex;
        polygonVertex.normalAngle = angleOfNormalVectors2[j];
        polygonVertex.vector = vecShapes[1].getPoint(j);
        polygonVertex.polygonType = PolygonType::Obstacle;
        sortedPolygonVertices.push_back(polygonVertex);
        ++j;
    }
    
    sort(sortedPolygonVertices.begin(), sortedPolygonVertices.end(), PolygonVertex::CompAngles);
}

/**
 * @brief Is in workspace
 * @param x
 * @param y
 * @return true or false
 */
bool IsInWorkSpace(float x, float y)
{
    if ((x < WorkSpaceStartX) ||
        (y < WorkSpaceStartY) ||
        (x > (WorkSpaceStartX + WorkSpaceSizeX - ShapeRadiusSize * 2)) ||
        (y > (WorkSpaceStartY + WorkSpaceSizeY - ShapeRadiusSize * 2)))
    {       
        return false;
    }

    return true;
}

/**
 * @brief Center robot position
 * @param robot - robot
 * @param p - point in which to center
 */
void CenterRobotPosition(sf::CircleShape& robot, point_type p)
{
    robot.setPosition(p.get<0>(), p.get<1>());
    robot.setOrigin(robot.getRadius(), robot.getRadius());
}