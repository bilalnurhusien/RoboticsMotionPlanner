#include <vector>
#include <memory>

#include "../include/Helpers.hpp"
#include "../include/Types.hpp"
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
                vector<shared_ptr<sf::CircleShape> > & roadMap,
                vector<vector<sf::Vertex> >& edges)
{
    edges.clear();
    roadMap.clear();
    
    vector<Vertex_t> vertices = motionPlanning.GetRoadMap();

    for (uint32_t i = 0; i  < vertices.size(); ++i)
    {
        roadMap.push_back(make_shared<sf::CircleShape>(DotRadiusSize));
        CenterPosition(roadMap[i].get(), vertices[i].p);
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
    shared_ptr<sf::Shape> robot;
    shared_ptr<sf::CircleShape> robotDot = make_shared<sf::CircleShape>(DotRadiusSize);

    vector<shared_ptr<sf::Shape> > polygonObstacles;
    vector<shared_ptr<sf::CircleShape> > roadMapVertices;
    vector<vector<sf::Vertex> > roadMapEdges;

    list<point_type> path;
    sf::Clock clock;

    uint32_t maxNumNodes = 500;
    uint32_t maxOfNumNeighbors = 7;

    float minDistance = 50.f;
    bool displayConfigSpace = false;
    bool fullScreen = false;
    bool displayPointLocations = false;
    bool displayObstacles = true;
    bool displayIntermediates = false;

    /* Process arguments */
    if (!ProcessArguments(argc, argv, robot, polygonObstacles, fullScreen, maxNumNodes, minDistance))
    {
        return -1;
    }

    /* Position dot at center of robot */
    CenterPosition(robotDot.get(), point_type(robot->getPosition().x, robot->getPosition().y, robot->getRotation()));
    robotDot->setFillColor(robot->getFillColor());
    
    /* Create motion planning roadmap */
    shared_ptr<CollisionDetector> pCollisionDetector = make_shared<CollisionDetector>(polygonObstacles, robot);
    MotionPlanning motionPlanning(pCollisionDetector);

    bool result = motionPlanning.CreateRoadMap(maxNumNodes, maxOfNumNeighbors, minDistance, point_type(robot->getPosition().x, robot->getPosition().y, robot->getRotation()));
    if (!result)
    {
        cout << "Failed to create roadmap" << endl;
        return -1;
    }

    cout << "Creating window, robot, and obstacles..." << endl;
    
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

    if (fullScreen)
    {
        window.create(sf::VideoMode(WindowSizeX, WindowSizeY), "Project: Path Planning", sf::Style::Fullscreen);
    }
    else
    {
        window.create(sf::VideoMode(WindowSizeX, WindowSizeY), "Project: Path Planning", sf::Style::Default);
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
                if (text == Char_p)
                {
                    displayPointLocations = !displayPointLocations;
                    window.clear();
                }
                else if (text == Char_o)
                {
                    displayObstacles = !displayObstacles;
                    window.clear();
                }
                else if (text == Char_i)
                {
                    displayIntermediates = !displayIntermediates;
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
                  
                    if (pCollisionDetector->IsCollision(point_type(position.x, position.y, 0)))
                    {
                        cout << "Can't move to position. Collision detected at x: " << position.x << ", y: " << position.y << endl;
                        
                        continue;
                    }

                    list <point_type> new_path = motionPlanning.GetShortestPath(
                                                        point_type(robotDot->getPosition().x, robotDot->getPosition().y, 0),
                                                        point_type(position.x, position.y, 0));

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
                CenterPosition(robotDot.get(), p);
                CenterPosition(robot.get(), p);
                if (!displayIntermediates)
                {
                    window.clear();
                }
                clock.restart();
            }
        }
       
        if (displayConfigSpace)
        {
            for (uint32_t i = 0; i  < roadMapVertices.size(); ++i)
            {
                window.draw(*roadMapVertices[i]);
            }
            
            for (uint32_t i = 0; i < roadMapEdges.size(); ++i)
            {
                window.draw((sf::Vertex*)(&roadMapEdges[i][0]), 2, sf::Lines);
            }
            
            window.draw(*robotDot);
        }
        else
        {
            text.setString("Robot");
            text.setPosition(robot->getPosition().x, robot->getPosition().y);
            window.draw(*robot);
            window.draw(text);
            
            if (displayPointLocations)
            {
                for (uint32_t i = 0; i < robot->getPointCount(); ++i)
                {
                    float x = 0;
                    float y = 0;
                    sf::FloatRect rect = robot->getLocalBounds();

                    if (robot->getPoint(i).x < (rect.width / 2))
                    {
                        x = robot->getPosition().x - robot->getPoint(i).x;
                    }
                    else
                    {
                        x = robot->getPosition().x + robot->getPoint(i).x;
                    }
                    
                    
                    if (robot->getPoint(i).y < (rect.height / 2))
                    {
                        y = robot->getPosition().y - robot->getPoint(i).y;
                    }
                    else
                    {
                        y = robot->getPosition().y + robot->getPoint(i).y;
                    }                    

                    text.setPosition(x, y);
                    text.setString(ToStringSetPrecision(x, 0) + ", " + ToStringSetPrecision(y, 0));
                    window.draw(text);
                }
            }
        }

        if (displayObstacles)
        {
            for (uint32_t i = 0; i < polygonObstacles.size(); ++i)
            {
                window.draw(*polygonObstacles[i]);
                text.setString("Obstacle" + to_string(i));
                text.setPosition(polygonObstacles[i]->getPosition().x, polygonObstacles[i]->getPosition().y);
                window.draw(text);
                
                if (displayPointLocations)
                {
                    float x = 0;
                    float y = 0;

                    sf::FloatRect rect = polygonObstacles[i]->getLocalBounds();
                    for (uint32_t j = 0; j < polygonObstacles[i]->getPointCount(); ++j)
                    {
                        if (polygonObstacles[i]->getPoint(j).x < rect.width / 2)
                        {
                            x = polygonObstacles[i]->getPosition().x - polygonObstacles[i]->getPoint(j).x;
                        }
                        else
                        {
                            x = polygonObstacles[i]->getPosition().x + polygonObstacles[i]->getPoint(j).x;
                        }
                        
                        
                        if (polygonObstacles[i]->getPoint(j).y < rect.height / 2)
                        {
                            y = polygonObstacles[i]->getPosition().y - polygonObstacles[i]->getPoint(j).y;
                        }
                        else
                        {
                            y = polygonObstacles[i]->getPosition().y + polygonObstacles[i]->getPoint(j).y;
                        }                    
                        
                        text.setPosition(x, y);
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
