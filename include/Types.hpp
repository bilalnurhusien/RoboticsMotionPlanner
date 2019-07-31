#pragma once

#include <SFML/Graphics.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <iostream>
#include <vector>
#include <math.h>
#include <stdint.h>
#include <utility>
#include <limits.h>
#include <float.h>
#include <algorithm>

// #define DEBUG

/**
 * Const variables
 */
const float WindowSizeX = sf::VideoMode::getDesktopMode().width;
const float WindowSizeY = sf::VideoMode::getDesktopMode().height;
const float WorkSpaceSizeX = WindowSizeX;
const float WorkSpaceSizeY = WindowSizeY;
const float WorkSpaceStartX = (WindowSizeX - WorkSpaceSizeX)/2;
const float WorkSpaceStartY = (WindowSizeY - WorkSpaceSizeY)/2;
const float OffSetX1 = 150.0f;
const float OffSetY1 = 100.0f;
const float OffSetX2 = 700.0f;
const float OffSetY2 = 100.0f;
const float ShapeRadiusSize = 80.0f;
const float OriginX = 500.0f;
const float OriginY = 500.0f;
const float DotRadiusSize = 10.0f;
const uint32_t MaxNumObstacles = 15;
const uint32_t MaxNumVertices = 8;
const uint32_t MinNumVertices = 3;
const uint32_t RobotStartPosX = 400;
const uint32_t RobotStartPosY = 200;
const uint32_t DotStartPosX = RobotStartPosX + ShapeRadiusSize;
const uint32_t DotStartPosY = RobotStartPosY + ShapeRadiusSize;
const char FontFile[] = "/usr/share/fonts/truetype/freefont/FreeMono.ttf";
const char Char_Space = 32;
const char Char_P = 'r';
const char Char_O = 'o';
const uint32_t Frame_Delay_Ms = 200;

const float ObstacleCoordinates[MaxNumObstacles][2] =
{
    {170.0f, 170.0f},
    {800.0f, 800.0f},
    {275.0f, 400.0f}, 
    {540.0f, 430.0f},
    {800.0f, 400.0f},
    {80.0f, 800.0f},
    {650.0f, 300.0f},
    {1650.0f, 300.0f},
    {1650.0f, 600.0f},
    {1400.0f, 600.0f},
    {500.0f, 800.0f},
    {1300.0f, 800.0f},
    {1650.0f, 800.0f},
    {1500.0f, 200.0f},
    {WorkSpaceStartX, WorkSpaceStartY},       
};

namespace bg = boost::geometry;
namespace bgm = boost::geometry::model;

typedef bg::model::point<float, 2, bg::cs::cartesian>  point_type;
typedef bg::model::box<point_type> box_type;
typedef bg::model::polygon<point_type, false, false> polygon_type; // ccw, closed polygon
typedef std::pair<point_type, unsigned> value;

/**
 * Struct that represents
 * a point on a polygon.
 */
struct Point
{
    Point(float valX, float valY)
    {
        x = valX;
        y = valY;        
    }
    
    Point():
        x(0),
        y(0)
    {
    }
    
    float x = 0;
    float y = 0;
};

/**
 * Enum used for distinguishing
 * between types of polygons 
 */
enum PolygonType
{
    None = 0,
    Robot = 1,
    Obstacle = 2
};

/**
 * Struct that represents a polygon vertex
 */
struct PolygonVertex
{
    /* Polygon type */
    PolygonType polygonType;

    /* Vector */
    sf::Vector2f vector;
    
    /* Normal edge angle associated with vertex */
    float normalAngle;
    
    /* Used to sort normal angles */
    static bool CompAngles(const PolygonVertex& p1, const PolygonVertex& p2)
    {
        return p1.normalAngle < p2.normalAngle;
    }
};