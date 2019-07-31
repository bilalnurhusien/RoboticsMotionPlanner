#include <vector>
#include <memory>

#include "../include/Helpers.hpp"
#include "../include/Types.hpp"
#include "../include/Minkowski.hpp"
#include "../include/CollisionDetector.hpp"
#include "../include/MotionPlanning.hpp"

using namespace std;

/**
 * @brief Get motion planning road map
 * @param motionPlanning - motion planner
 * @param roadMap - list of vertices on the roadmap
 * @param edges - list of edges connecting roadmap vertices
 */
void GetRoadMap(MotionPlanning& motionPlanning,
                vector<sf::CircleShape>& roadMap,
                vector<vector<sf::Vertex> >& edges)
{
    edges.clear();
    roadMap.clear();
    
    vector<Vertex_t> vertices = motionPlanning.GetRoadMap();

    for (uint32_t i = 0; i  < vertices.size(); ++i)
    {
        roadMap.push_back(sf::CircleShape(DotRadiusSize));
        CenterRobotPosition(roadMap[i], vertices[i].p);
        for (uint32_t j = 0; j < vertices[i].neighbors.size(); ++j)
        {
            edges.push_back(
            {
                sf::Vertex(sf::Vector2f(vertices[i].p.get<0>(), vertices[i].p.get<1>())),
                sf::Vertex(sf::Vector2f(vertices[i].neighbors[j].get<0>(), vertices[i].neighbors[j].get<1>()))
            });
        }

    }
}

int32_t main(int argc, char *argv[])
{

    sf::RenderWindow window;
    sf::CircleShape robot;
    sf::CircleShape robotDot = sf::CircleShape(DotRadiusSize);

    vector<sf::CircleShape> polygonObstacles;
    vector<sf::ConvexShape> minkowskiObstacles;

    vector<sf::CircleShape> roadMapVertices;
    vector<vector<sf::Vertex> > roadMapEdges;

    list<point_type> path;
    sf::Clock clock;

    uint32_t maxNumNodes = 100;
    float minDistance = 10.f;
    uint32_t maxOfNumNeighbors = 5;

    bool displayConfigSpace = false;
    bool fullScreen = false;
    bool displayPointLocations = false;
    bool displayObstacles = true;

    /* Process arguments */
    if (!ProcessArguments(argc, argv, robot, polygonObstacles, fullScreen))
    {
        return -1;
    }

    if (fullScreen)
    {
        window.create(sf::VideoMode(WindowSizeX, WindowSizeY), "Project: Path Planning", sf::Style::Fullscreen);
    }
    else
    {
        window.create(sf::VideoMode(WindowSizeX, WindowSizeY), "Project: Path Planning", sf::Style::Default);
    }

    /* Position dot at center of robot */
    CenterRobotPosition(robotDot, point_type(DotStartPosX, DotStartPosY));
    robotDot.setFillColor(robot.getFillColor());
    
    /* Create motion planning roadmap */
    shared_ptr<CollisionDetector> pCollisionDetector = make_shared<CollisionDetector>(polygonObstacles, robot);
    MotionPlanning motionPlanning(pCollisionDetector);

    bool result = motionPlanning.CreateRoadMap(maxNumNodes, maxOfNumNeighbors, minDistance);
    if (!result)
    {
        cout << "Failed to create roadmap";
        return -1;
    }
    
    GetRoadMap(motionPlanning, roadMapVertices, roadMapEdges);

    /* Select font */
    sf::Text text;
    sf::Font font;
    if (!font.loadFromFile(FontFile))
    {
        cout << "Warning: failed to load font" << endl;
    }

    text.setFont(font);
    text.setCharacterSize(18); 
    text.setColor(sf::Color::White);

    for (uint32_t i = 0; i < robot.getPointCount(); ++i)
    {
        sf::Vector2f vec = robot.getPoint(i);
    }

    window.clear();

    while (window.isOpen())
    {
        sf::Event event;

        while (window.pollEvent(event))
        {
            if ((event.type == sf::Event::Closed) ||
                (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)))
            {
                window.close();
            }
            else if ((event.type == sf::Event::MouseButtonPressed) || 
                     (event.type == sf::Event::TextEntered))
            {
                char text = 0;

                if (event.type == sf::Event::TextEntered)
                {
                    if (event.text.unicode < 128)
                    {
                       text = static_cast<char>(event.text.unicode);
                    }
                }
                if (text == Char_P)
                {
                    displayPointLocations = !displayPointLocations;
                    window.clear();
                }
                else if (text == Char_O)
                {
                    displayObstacles = !displayObstacles;
                    displayPointLocations = false;
                    window.clear();
                }
                else if (sf::Mouse::isButtonPressed(sf::Mouse::Right))
                {
                    displayConfigSpace = !displayConfigSpace;
                    window.clear();                    
                }
                else if ((sf::Mouse::isButtonPressed(sf::Mouse::Left)) ||
                         (text == Char_Space))
                {
                    sf::Vector2i position = sf::Mouse::getPosition(window);
                  
                    if (pCollisionDetector->IsCollision(point_type(position.x, position.y)))
                    {
                        cout << "Can't move to position. Collision detected at x: " << position.x << ", y: " << position.y << endl;
                        
                        continue;
                    }
                    
                    cout << "New point: " << position.x << " " << position.y << " not a collision " << endl;
                    
                    list <point_type> new_path = motionPlanning.GetShortestPath(
                                                        point_type(robotDot.getPosition().x, robotDot.getPosition().y),
                                                        point_type(position.x, position.y));

                    if (!new_path.empty())
                    {
                        GetRoadMap(motionPlanning, roadMapVertices, roadMapEdges);
                    
                        clock.restart();

                        window.clear();
                        
                        path.insert(path.begin(), new_path.begin(), new_path.end());
                    }
                }
            }
        }
       
        if (clock.getElapsedTime().asMilliseconds() > Frame_Delay_Ms)
        {
            if (!path.empty())
            {
                point_type p = path.back();
                path.pop_back();
                /* Center robot around this new point */
                CenterRobotPosition(robotDot, p);
                CenterRobotPosition(robot, p);
                clock.restart();
            }
        }
       
        if (displayConfigSpace)
        {
            for (uint32_t i = 0; i < minkowskiObstacles.size(); ++i)
            {
                window.draw(minkowskiObstacles[i]);
                text.setString("C-Obstacle" + ToStringSetPrecision(i, 1));
                text.setPosition(minkowskiObstacles[i].getPosition().x, minkowskiObstacles[i].getPosition().y);
                window.draw(text);
            }
            
            for (uint32_t i = 0; i  < roadMapVertices.size(); ++i)
            {
                sf::Vector2f coord = roadMapVertices[i].getPosition();

                /* Write coordinates above each roadmap vertex */
                text.setPosition(coord.x - DotRadiusSize, coord.y - 3 * DotRadiusSize);
                text.setString(ToStringSetPrecision(coord.x, 0) + ", " + ToStringSetPrecision(coord.y, 0));
                window.draw(text);
                window.draw(roadMapVertices[i]);
            }
            
            for (uint32_t i = 0; i < roadMapEdges.size(); ++i)
            {
                window.draw((sf::Vertex*)(&roadMapEdges[i][0]), 2, sf::Lines);
            }
            
            window.draw(robotDot);
        }
        else
        {
            text.setString("Robot");
            text.setPosition(robot.getPosition().x, robot.getPosition().y);
            window.draw(robot);
            window.draw(text);
            
            if (displayPointLocations)
            {
                for (uint32_t i = 0; i < robot.getPointCount(); ++i)
                {
                    float x = robot.getPosition().x + robot.getPoint(i).x;
                    float y = robot.getPosition().y + robot.getPoint(i).y;
                    text.setPosition(x - robot.getRadius(), y - robot.getRadius());

                    text.setString(ToStringSetPrecision(x, 0) + ", " + ToStringSetPrecision(y, 0));
                    window.draw(text);
                }
            }
        }

        if (displayObstacles)
        {
            for (uint32_t i = 0; i < polygonObstacles.size(); ++i)
            {
                window.draw(polygonObstacles[i]);
                text.setString("Obstacle" + to_string(i));
                text.setPosition(polygonObstacles[i].getPosition().x, polygonObstacles[i].getPosition().y);
                window.draw(text);
                
                if (displayPointLocations)
                {
                    for (uint32_t j = 0; j < polygonObstacles[i].getPointCount(); ++j)
                    {
                        float x = polygonObstacles[i].getPosition().x + polygonObstacles[i].getPoint(j).x;
                        float y = polygonObstacles[i].getPosition().y + polygonObstacles[i].getPoint(j).y;
                        text.setPosition(x - polygonObstacles[i].getRadius(), y - polygonObstacles[i].getRadius());
                        text.setString(ToStringSetPrecision(x, 0) + ", " + ToStringSetPrecision(y, 0));
                        window.draw(text);
                    }
                }
            }
        }

        window.display();
    }

    return 0;
}