#include <string.h>
#include <fstream>
#include <stdlib.h> 
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

#include "../include/Helpers.hpp"

namespace pt = boost::property_tree;

using namespace std;

/**
  * Print help messages
  */
void PrintHelp()
{
    cout << "Usage:" << endl;
    cout << "\t./pathplanning -r <num-of-robot-vertices> -o <num-of-obstacles> | -i <csv-input-file>" << endl;
    cout << "\t\t-r: Number of robot vertices" << endl;
    cout << "\t-o: Number of obstacles" << endl;
    cout << "\t-f: Full screen" << endl;
    cout << "\t-i: CSV input file" << endl;
    cout << "\t-n: Number of nodes in PRM graph" << endl;
    cout << "\t-d: Minimum distance between PRM nodes" << endl;
    cout << "\tExample:" << endl;
    cout << "\t\t# Input CSV file"<< endl;
    cout << "\t\t$./pathplanning -i inputconfig.xml" << endl << endl;
    cout << "\t\t# For a triangle robot with 5 obstacles in full screen workspace with 500 nodes in PRM" << endl;
	cout << "\t\t$./pathplanning -r 3 -o 5 -f -n 500" << endl;
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


bool ReadFile(string fileName,
              shared_ptr<sf::Shape>& robot,
              vector<shared_ptr<sf::Shape> >& polygonObstacles)
{
    pt::ptree tree;

    pt::read_xml(fileName, tree);

    tree.get<string>("config.filename");
    string robotPosition = tree.get<string>("config.robot.position");
    string robotPoints = tree.get<string>("config.robot.points");

    shared_ptr<sf::Shape> shape = make_shared<sf::ConvexShape>();
    size_t n = std::count(robotPoints.begin(), robotPoints.end(), ',');
    dynamic_cast<sf::ConvexShape*>(shape.get())->setPointCount(n);

    {
        std::istringstream iss(robotPoints);
        int x, y, point = 0;
        char comma;

        while (!iss.eof())
        {
            if (!(iss >> x >> comma >> y))
            {
                cout << "Failed to read robot point:  " << point << endl;
                return false;
            }
            
            dynamic_cast<sf::ConvexShape*>(shape.get())->setPoint(point, sf::Vector2f(x, y));
            point++;
        }
    }

    {
        std::istringstream iss(robotPosition);
        int x, y = 0;
        char comma;
        if (!(iss >> x >> comma >> y))
        {
            cout << "Failed to read robot position" << endl;
            return false;
        }
        CenterPosition(shape.get(), point_type(x, y, 0));
    }

    robot = shape;
    robot->setFillColor(sf::Color::Green);

    BOOST_FOREACH(const pt::ptree::value_type& v, tree.get_child("config.obstacles"))
    {
        // The data function is used to access the data stored in a node.
        string obstaclePoints = v.second.get<string>("points");
        string obstaclePosition = v.second.get<string>("position");

        // The data function is used to access the data stored in a node.
        shape = make_shared<sf::ConvexShape>();
        size_t n = std::count(obstaclePoints.begin(), obstaclePoints.end(), ',');
        dynamic_cast<sf::ConvexShape*>(shape.get())->setPointCount(n);

        std::istringstream iss(obstaclePoints);
        int x, y, point = 0;
        char comma;

        while (!iss.eof())
        {
            if (!(iss >> x >> comma >> y))
            {
                cout << "Failed to read robot point:  " << point << endl;
                return false;
            }
            
            dynamic_cast<sf::ConvexShape*>(shape.get())->setPoint(point, sf::Vector2f(x, y));
            point++;
        }
     
        {
            std::istringstream newIss(obstaclePosition);
            if (!(newIss >> x >> comma >> y))
            {
                cout << "Failed to read robot position" << endl;
                return false;
            }
            CenterPosition(shape.get(), point_type(x, y, 0));
        }

        sf::FloatRect rect = shape->getLocalBounds();

        shape->setFillColor(sf::Color::Blue);
        polygonObstacles.push_back(shape);
    }


    return true;
}

/**
  * Process arguments
  */
bool ProcessArguments(int argc,
                      char* argvp[],
                      shared_ptr<sf::Shape>& robot,
                      vector<shared_ptr<sf::Shape> >& polygonObstacles,
                      bool& fullScreen,
                      uint32_t& numNodes,
                      float& nodeDistance)
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
        
        else if (strcmp(argvp[i], "-n") == 0)
        {
            if ((i + 1) > argc)
            {
                cout << "Number of nodes missing" << endl;

                return false;
            }
            
            ++i;

            if (!IsNumeric(argvp[i]))
            {
                cout << "Invalid numerical argument: " << argvp[i] << endl;

                PrintHelp();

                return false;
            }
            
            numNodes = strtol(argvp[i], nullptr, 10);
            
            if (numNodes < MinNumOfNodes)
            {
                cout << "Please select larger number of nodes: " << argvp[i] << endl;

                PrintHelp();

                return false;
            }
            
            if (numNodes > LargeNumOfNodes)
            {
                cout << "Large num of nodes selected: " << argvp[i] << endl;
            }
        }
        else if (strcmp(argvp[i], "-r") == 0)
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
        else if (strcmp(argvp[i], "-d") == 0)
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

            nodeDistance = strtof(argvp[i], nullptr);

            if (nodeDistance > MaxNodeDistance)
            {
                cout << "Minimum node distance set to " << MaxNodeDistance << endl;

                nodeDistance = MaxNodeDistance;
            }        

            continue;
        }
        else if (strcmp(argvp[i], "-o") == 0)
        {
            if ((i + 1) > argc)
            {
                cout << "Missing obstacle count" << endl;
                
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
        else if (strcmp(argvp[i], "-i") == 0)
        {
            if ((i + 1) > argc)
            {
                cout << "Missing input file" << endl;
                
                return false;
            }
            
            ++i;

            ReadFile(argvp[i], robot, polygonObstacles);

            continue;
        }
    }
    
    if (((robotVertexNum == -1) &&
         (robot->getPointCount() == 0))||
        ((obstacleNum == -1)) &&
        (polygonObstacles.empty()))
    {
        cout << "Robot or obstacle vertices not specified" << endl;
        return false;
    }

    /* initialize random seed: */
    srand (time(NULL));

    if (robotVertexNum != -1)
    {   
        robot = make_shared<sf::CircleShape> (ShapeRadiusSize, robotVertexNum);
        robot->setFillColor(sf::Color::Green);
        CenterPosition(robot.get(), point_type(RobotStartPosX, RobotStartPosY, 0));
    }

    if (obstacleNum != -1)
    {
        for (uint32_t i = 0; i < obstacleNum; ++i)
        {
            shared_ptr<sf::Shape> pObstacle = make_shared<sf::CircleShape>(ShapeRadiusSize, rand() % MaxNumVertices + MinNumVertices);
            pObstacle->setFillColor(sf::Color::Blue);
            CenterPosition(pObstacle.get(), point_type(ObstacleCoordinates[i][0], ObstacleCoordinates[i][1], 0));
            polygonObstacles.push_back(pObstacle);
        }
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
void GetNormalVectors(const sf::ConvexShape* shape,
                      vector<vector<sf::Vertex> >& normalVectors,
                      bool outwardNormal)
{
    vector<Point> vectors;
    vector<Point> midPoints;

    size_t vertexCount = shape->getPointCount();

    for (uint32_t i = 0; i < vertexCount; ++i)
    {    
        Point vector;
        Point midPoint;

        sf::Vector2f vertex = shape->getPoint(i);
        sf::Vector2f nextVertex;

        nextVertex = shape->getPoint((i + 1) % vertexCount);
        vector.x = nextVertex.x - vertex.x;
        vector.y = nextVertex.y - vertex.y;
        midPoint.x = (vertex.x + nextVertex.x) / 2  + shape->getPosition().x;
        midPoint.y = (vertex.y + nextVertex.y) / 2  + shape->getPosition().y;

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
 * @brief Is in workspace
 * @param x
 * @param y
 * @return true or false
 */
bool IsInWorkSpace(float x, float y)
{
    if ((x <= WorkSpaceStartX) ||
        (y <= WorkSpaceStartY) ||
        (x >= (WorkSpaceStartX + WorkSpaceSizeX)) ||
        (y >= (WorkSpaceStartY + WorkSpaceSizeY)))
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
void CenterPosition(sf::Shape* const pRobot, const point_type& p)
{
    sf::FloatRect rect = pRobot->getLocalBounds();
    
    sf::CircleShape* circleShape = nullptr;
    sf::ConvexShape* convexShape = nullptr;
    if (circleShape = dynamic_cast<sf::CircleShape*>(pRobot))
    {
        pRobot->setOrigin(circleShape->getRadius(), circleShape->getRadius());
    }
    else if (convexShape = dynamic_cast<sf::ConvexShape*>(pRobot))
    {
        pRobot->setOrigin(rect.width / 2.f, rect.height / 2.f);
    }
    pRobot->setPosition(p.get<0>(), p.get<1>());
    pRobot->setRotation(p.get<2>());
}
