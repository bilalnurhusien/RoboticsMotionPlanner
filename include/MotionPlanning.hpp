#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <iostream>
#include <memory>
#include <utility>

#include "Types.hpp"
#include "CollisionDetector.hpp"

namespace bgi = boost::geometry::index;

//
// Create a struct to hold properties for each vertex
//
struct Vertex_t {
    point_type p;
    std::vector<point_type> neighbors;
};  

//
// Create a struct to hold properties for each edge
//
struct EdgeProperties_t
{
  float   weight;
};

/*
 * Typedefs
 **/
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex_t, EdgeProperties_t>UndirectedGraph;
typedef boost::property_map<UndirectedGraph, boost::vertex_index_t>::type IdMap;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor vertex_descriptor_t;
typedef UndirectedGraph::edge_descriptor edge_descriptor_t;
typedef std::vector<edge_descriptor_t> path_t;


/**
 * @class MotionPlanning
 * 
 * @brief Creates a roadmap of vertices that a robot can traverse without collision
 */
class MotionPlanning
{
    
public:
    explicit MotionPlanning(const std::vector<std::shared_ptr<sf::Shape> >& polygonObstacles, 
                            const std::shared_ptr<CollisionDetector>&  pCollisionDetector);
    
    /**
     * @brief Create road map
     * @param maxNumOfNodes - maximum number of nodes to create
     * @param maxNumOfNeighbors - maximum number of neighbors
     * @return true (success), false (failure)
     */
    bool CreateRoadMap(uint32_t maxNumOfNodes, uint32_t maxNumOfNeighbors, float minDistance, point_type robotStart);
    
    /**
     * @brief Get road map
     * @return Road map
     */
    std::vector<Vertex_t> GetRoadMap();
    
    /**
     * @brief Find shortest path between two points
     * @param start Start point
     * @param end End point
     * @return list of points
     */
    std::list<point_type> GetShortestPath(point_type start,
                                          point_type end);
    virtual ~MotionPlanning();

private:

    // private constructors
    MotionPlanning(const MotionPlanning& rhs);
    MotionPlanning& operator=(const MotionPlanning& rhs);
    

    /**
     * @brief Add vertex to graph
     * @param p - vertex
     * @return index of vertex in graph
     */
    int AddVertex(point_type p);
    
    /**
     * @brief Select random coordinate in workspace
     * @param x
     * @param y
     */
    void SelectRandomCoordinate(uint32_t& x, uint32_t& y, uint32_t& t);

    /**
     * @brief Select random coordinate that's in an obstacle
     * @param x
     * @param y
     */
    void SelectRandomObstacleCoordinate(uint32_t& x, uint32_t& y, uint32_t& t);


    /**
     * @brief Get minimum distance from neighboring point(s)
     * @param x - x-coord point to consider
     * @param y - y-coord point to consider 
     * @return minimum distance
     */
    float GetMinDistanceFromNeighbors(uint32_t x, uint32_t y, uint32_t t);

    /**
     * @brief Find vertex
     * @param p
     */
    auto FindVertexById(point_type p) 
    {
        for(auto vd : boost::make_iterator_range(boost::vertices(*m_pGraph)))
        {
            if ((p.get<0>() == (*m_pGraph)[vd].p.get<0>()) &&
                (p.get<1>() == (*m_pGraph)[vd].p.get<1>()))
            {
                return vd;
            }
        }
        
        throw std::range_error("Vertex not found -> x: " + std::to_string(p.get<0>()) + ", y: " + std::to_string(p.get<1>()));
    }

    bgi::rtree< value, bgi::rstar<16> > m_rtree;
    std::shared_ptr<CollisionDetector>  m_pCollisionDetector = nullptr;
    std::vector<point_type> m_points;
    std::shared_ptr<UndirectedGraph> m_pGraph = std::make_shared<UndirectedGraph>();
    size_t RemoveId(point_type p);
    uint32_t m_maxNumOfNeighbors;
    std::vector<std::shared_ptr<sf::Shape> > m_polygonObstacles;
};

