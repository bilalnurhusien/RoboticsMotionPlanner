#include "../include/MotionPlanning.hpp"
#include "../include/CollisionDetector.hpp"
#include <boost/function_output_iterator.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/operators.hpp>
#include <boost/foreach.hpp>
#include <boost/ref.hpp>
#include <boost/graph/graph_utility.hpp>
#include <limits>
#include <algorithm>
#include <list>
#include <time.h>

#include "../include/Helpers.hpp"


using namespace std;

MotionPlanning::MotionPlanning(const vector<shared_ptr<sf::Shape> >& polygonObstacles,
                               const shared_ptr<CollisionDetector>&  pCollisionDetector)
    : m_polygonObstacles(polygonObstacles),
      m_pCollisionDetector(pCollisionDetector)
{
    /* initialize random seed: */
    srand (time(NULL));
}

MotionPlanning::~MotionPlanning()
{
    m_polygonObstacles.clear();
    m_pCollisionDetector = nullptr;
}


std::ostream& operator<<(std::ostream &os, const Vertex_t &v)
{
    os << "[x=\"" << v.p.get<0>() << "\", y=\"" << v.p.get<1>() << "\"]";
    return os;
}

std::ostream& operator<<(std::ostream &os, const point_type &p)
{
    os << "[x=\"" << p.get<0>() << "\", y=\"" << p.get<1>() << "\"]";
    return os;
}

float MotionPlanning::GetMinDistanceFromNeighbors(uint32_t x, uint32_t y, uint32_t t)
{
    vector<value> neighbors;
    m_rtree.query(bgi::nearest(point_type(x, y, t), m_maxNumOfNeighbors), std::back_inserter(neighbors));
    float minDist = std::numeric_limits<float>::max();
    for (uint32_t j = 1; j < neighbors.size(); ++j)
    {
        if (m_pCollisionDetector->IsPathCollision(point_type(x, y, t), neighbors[j].first))
        {
            continue;
        }

        float neighborDist = static_cast<float>(boost::geometry::distance(point_type(x, y, t), neighbors[j].first));

        if (neighborDist < minDist)
        {
            minDist = neighborDist;
        }
    }
    
    return minDist;
   
}
void MotionPlanning::SelectRandomCoordinate(uint32_t& x, uint32_t& y, uint32_t& t)
{
    x = rand() % ((uint32_t)WorkSpaceSizeX);
    y = rand() % ((uint32_t)WorkSpaceSizeY);
    t = rand() % 360;
}

void MotionPlanning::SelectRandomObstacleCoordinate(uint32_t& x, uint32_t& y, uint32_t& t)
{
    /*
     * If no obstacle in workspace, select a random coordinate
     */
    if (m_polygonObstacles.empty())
    {
        return SelectRandomCoordinate(x, y, t);
    }

    uint32_t i = rand() % m_polygonObstacles.size();
    x = m_polygonObstacles[i]->getPosition().x;
    y = m_polygonObstacles[i]->getPosition().y;
    t = rand() % 360;
}

bool MotionPlanning::CreateRoadMap(uint32_t maxNumOfNodes, uint32_t maxNumOfNeighbors, float minDistance, point_type robotStart)
{
    cout << "Step 1 of 2: Creating roadmap vertices..." << endl;

    //
    // Increment by one since boost::rtree treats each point as a neighbor of itself
    //
    m_maxNumOfNeighbors = maxNumOfNeighbors + 1;

    const uint32_t MaxTotalRetries = 5;
    const uint32_t MaxRandomSelectTries = 20;
    const uint32_t IncrementValue = 40;
    const float FractionOfRandomNodesInFreeSpace= 0.1f;
    uint32_t totalRetries = 0;
    float progress = 0;
    time_t start,end;
    time (&start);
    bool success = false;
    m_rtree.clear();
    m_pGraph->clear();

    for (uint32_t totalRetries = 0; (totalRetries < MaxTotalRetries) && !success; ++totalRetries)
    {
        uint32_t i = 0;
        uint32_t openNodes = maxNumOfNodes * FractionOfRandomNodesInFreeSpace;

        for (; i < openNodes; ++i)
        {
            uint32_t randomSelectTries = 0;
            uint32_t x = 0;
            uint32_t y = 0;
            uint32_t t = 0;
            
            bool collision = true;
            float distance = std::numeric_limits<float>::min();

            /*
             * Randomly select a point that is at least
             * MinDistance away from any neighboring point
             **/
            do
            {
                SelectRandomCoordinate(x, y, t);
                if (!(collision = m_pCollisionDetector->IsCollision(point_type(x, y, t))))
                {
                    distance = GetMinDistanceFromNeighbors(x, y, t);
                }

                randomSelectTries++;
            } while ((collision ||
                     (distance < minDistance)) &&
                     (randomSelectTries < MaxRandomSelectTries));

            if ((randomSelectTries >= MaxRandomSelectTries) ||
                (collision == true) ||
                (distance < minDistance))
            {
                break;
            }
            

            /* Insert new value into graph and rtree */
            Vertex_t vertex;
            vertex.p = point_type(x, y, t);
            boost::add_vertex(vertex, *m_pGraph);
            m_rtree.insert(std::make_pair(vertex.p, i));
            
            if ((i / (float)maxNumOfNodes) > progress)
            {
                progress = (i / (float)maxNumOfNodes);
                cout << "\rProgress: " << ToStringSetPrecision(progress * 100, 0) << "%";
            }
        }
        
        if (i < openNodes)
        {
            //
            // Decrement the number of nodes and distance to increase the likelihood of success next time
            //
            m_rtree.clear();
            m_pGraph->clear();

            if (maxNumOfNodes > 5)
            {
                maxNumOfNodes -= 5;
            }
            
            if (minDistance > 5)
            {
                minDistance -= 5;
            }
            
            continue;
        }

        for (; i < maxNumOfNodes; ++i)
        {
            uint32_t randomSelectTries = 0;
            uint32_t x = 0;
            uint32_t y = 0;
            uint32_t t = 0;
            bool collision = true;
            float distance = std::numeric_limits<float>::min();

            /*
             * Randomly select a point that is neighbouring an obstacle
             **/
            do
            {
                SelectRandomObstacleCoordinate(x, y, t);                
                
                if (!(collision = m_pCollisionDetector->IsCollision(point_type(x, y, t))))
                {
                    continue;
                }
                else
                {
                    /*
                     * If collision is detected, go in a random direction until
                     * Cfree is reached or the end of the workspace
                     */
                    float randomDirection = fmod(rand(), 2*Pi);
                    
                    while (IsInWorkSpace(x, y))
                    {
                        x = x + cosf(randomDirection) * IncrementValue;
                        y = y + sinf(randomDirection) * IncrementValue;
                        
                        collision = m_pCollisionDetector->IsCollision(point_type(x, y, t));
                        
                        if (!collision)
                        {
                            distance = GetMinDistanceFromNeighbors(x, y, t);

                            break;
                        }
                    }
                    
                    if ((!IsInWorkSpace(x, y)) ||
                        (distance < minDistance))
                    {
                        continue;
                    }
                }

                randomSelectTries++;
            } while ((collision ||
                     (distance < minDistance)) &&
                     (randomSelectTries < MaxRandomSelectTries));

            if ((randomSelectTries >= MaxRandomSelectTries) ||
                (collision == true) ||
                (distance < minDistance))
            {
                break;
            }

            /* Insert new value into graph and rtree */
            Vertex_t vertex;
            vertex.p = point_type(x, y, t);
            boost::add_vertex(vertex, *m_pGraph);
            m_rtree.insert(std::make_pair(vertex.p, i));
            
            if ((i / (float)maxNumOfNodes) > progress)
            {
                progress = (i / (float)maxNumOfNodes);
                cout << "\rProgress: " << ToStringSetPrecision(progress * 100, 0)  << "%";
            }
        }

        time (&end);
        double diff = difftime (end,start);
        if (i >= maxNumOfNodes)
        {
            progress = 100.f;
            cout << "\rProgress: " << ToStringSetPrecision(progress, 0)  << "%" << endl;
            cout << "Time: " << ToStringSetPrecision(diff, 2) << " seconds" << endl;
            success = true;
            break;
        }
        else if (maxNumOfNodes == 0)
        {
            return false;
        }
        else
        {
            //
            // Decrement the number of nodes and distance to increase the likelihood of success next time
            //
            m_rtree.clear();
            m_pGraph->clear();
            if (maxNumOfNodes > 5)
            {
                maxNumOfNodes -= 5;
            }
            
            if (minDistance > 5)
            {
                minDistance -= 5;
            }
        }
    }
    
    if (!success)
    {
        return false;
    }

    cout << "Step 2 of 2: Creating roadmap edges..." << endl;
    progress = 0;
    uint32_t i = 0;
    time (&start);

    /*
     * Create edges between vertices in graph 
     **/
    UndirectedGraph::vertex_iterator vertexIter, vend;
    for (boost::tie(vertexIter, vend) = vertices(*m_pGraph); vertexIter != vend; ++vertexIter)
    {
        progress = (i++ / (float)maxNumOfNodes);
        cout << "\rProgress: " << ToStringSetPrecision(progress * 100, 0)  << "%";
        
        vector<value> neighbors;
#ifdef DEBUG
        cout << "Vertex: x - " <<  (*m_pGraph)[*vertexIter].p.get<0>() << " y - " << (*m_pGraph)[*vertexIter].p.get<1>() << endl;
#endif
        m_rtree.query(bgi::nearest((*m_pGraph)[*vertexIter].p, maxNumOfNeighbors), std::back_inserter(neighbors));

        auto startId = FindVertexById((*m_pGraph)[*vertexIter].p);

        for (uint32_t j = 0; j < neighbors.size(); ++j)
        {
            
#ifdef DEBUG
            cout << "Neighbor: x - " <<  neighbors[j].first.get<0>() << " y - " << neighbors[j].first.get<1>() << endl;
#endif
            if (m_pCollisionDetector->IsPathCollision((*m_pGraph)[*vertexIter].p, neighbors[j].first))
            {
                continue;
            }

            float dist = static_cast<float>(boost::geometry::distance((*m_pGraph)[*vertexIter].p, neighbors[j].first));
            
            if (dist == 0)
            {
                continue;
            }
#ifdef DEBUG
            cout << "Distance: " << dist << endl;
#endif
            Vertex_t dest;
            dest.p = neighbors[j].first;
            auto neighborId = FindVertexById(dest.p);

            std::pair<edge_descriptor_t, bool> e = boost::add_edge(startId, neighborId, *m_pGraph);
            (*m_pGraph)[e.first].weight = dist;
        }
    }
    m_maxNumOfNeighbors = maxNumOfNeighbors;

    AddVertex(robotStart);
    time (&end);
    double diff = difftime (end,start);
    cout << "\rProgress: " << ToStringSetPrecision(100, 0)  << "%" << endl;
    cout << "Time: " << ToStringSetPrecision(diff, 2) << " seconds" << endl;
    cout << "Num vertices: " << boost::num_vertices(*m_pGraph) << endl;
    cout << "Minimum distance: " << minDistance << endl;

    return true;
}

int MotionPlanning::AddVertex(point_type p)
{
    vector<value> neighbors;
    Vertex_t vertex;
    vertex.p = p;
    uint32_t id = boost::add_vertex(vertex, *m_pGraph);
    m_rtree.insert(std::make_pair(vertex.p, id));
    m_rtree.query(bgi::nearest((*m_pGraph)[id].p, m_maxNumOfNeighbors), std::back_inserter(neighbors));

    for (uint32_t j = 0; j < neighbors.size(); ++j)
    {
#ifdef DEBUG
        cout << "Neighbor: x - " <<  neighbors[j].first.get<0>() << " y - " << neighbors[j].first.get<1>() << endl;
#endif
        if (m_pCollisionDetector->IsPathCollision((*m_pGraph)[id].p, neighbors[j].first))
        {
            continue;
        }

        float dist = static_cast<float>(boost::geometry::distance((*m_pGraph)[id].p, neighbors[j].first));
        
        if (dist == 0)
        {
            continue;
        }

        Vertex_t dest;
        dest.p = neighbors[j].first;
        auto neighborId = FindVertexById(dest.p);

        std::pair<edge_descriptor_t, bool> e = boost::add_edge(id, neighborId, *m_pGraph);
        (*m_pGraph)[e.first].weight = dist;
    }

    return id;
}

size_t MotionPlanning::RemoveId(point_type p) {
    vector<value> neighbors;
    m_rtree.query(bgi::nearest(p, 1), std::back_inserter(neighbors));
    return m_rtree.remove(neighbors.begin(), neighbors.end());
}

list<point_type> MotionPlanning::GetShortestPath(point_type start,
                                                 point_type end)
{
    int startId = num_vertices(*m_pGraph) - 1;
    int endId = -1;

    try
    {
        endId = FindVertexById(end);
    }
    catch (const std::range_error&)
    {
        endId = AddVertex(end);
    }

    vertex_descriptor_t s = vertex(startId, *m_pGraph);
    vertex_descriptor_t e = vertex(endId, *m_pGraph);

    std::vector<vertex_descriptor_t> predecessors(boost::num_vertices(*m_pGraph));
    std::vector<float> distances(boost::num_vertices(*m_pGraph));
    auto predMap = boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index,*m_pGraph));
    auto distMap = boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, *m_pGraph));

    dijkstra_shortest_paths(*m_pGraph, s,
                        boost::weight_map(boost::get(&EdgeProperties_t::weight, *m_pGraph))
                        .predecessor_map(predMap)
                        .distance_map(distMap));

    boost::graph_traits < UndirectedGraph >::vertex_iterator vi, vend;
    list<point_type> verticesInPath;
    path_t path;
    
    vertex_descriptor_t v = e;
    for(vertex_descriptor_t u = predecessors[v]; u != v; v=u, u=predecessors[v])
    {
        std::pair<edge_descriptor_t,bool> edge_pair = boost::edge(u,v,*m_pGraph);
        path.push_back( edge_pair.first );
    }

    for(path_t::reverse_iterator riter = path.rbegin(); riter != path.rend(); ++riter)
    {
        vertex_descriptor_t u_tmp = boost::source(*riter, *m_pGraph);
        vertex_descriptor_t v_tmp = boost::target(*riter, *m_pGraph);
        edge_descriptor_t   e_tmp = boost::edge(u_tmp, v_tmp, *m_pGraph).first;
        verticesInPath.push_front((*m_pGraph)[u_tmp].p);
#ifdef DEBUG
        std::cout << "  " << (*m_pGraph)[u_tmp].p.get<0>() << " , " << (*m_pGraph)[v_tmp].p.get<1>() << "    (weight: " << (*m_pGraph)[e_tmp].weight << ")" << std::endl;
#endif
    }

    if (!verticesInPath.empty())
    {
        verticesInPath.push_front((*m_pGraph)[e].p);
    }
    else
    {
        /*
         * Ignore any points that can't be reached 
         **/
        size_t removeId = RemoveId((*m_pGraph)[e].p);
#ifdef DEBUG
        boost::print_graph(*m_pGraph, boost::get(boost::vertex_index, *m_pGraph));
        cout << "Removing " << (*m_pGraph)[e].p << endl;
        cout << "Removed: " <<  removeId << endl;
#endif
        boost::clear_vertex(e, *m_pGraph);
        boost::remove_vertex(e, *m_pGraph);
    }

    return verticesInPath;
}

vector<Vertex_t> MotionPlanning::GetRoadMap()
{
    boost::graph_traits < UndirectedGraph >::vertex_iterator vi, vend;
    vector<Vertex_t> roadMap;
    uint32_t i = 0;
    for (boost::tie(vi, vend) = vertices(*m_pGraph); vi != vend; ++vi)
    {
        roadMap.push_back((*m_pGraph)[*vi]);
        auto neighbours = boost::adjacent_vertices(*vi, *m_pGraph);
        for (auto vd : make_iterator_range(neighbours))
        {
            roadMap[i].neighbors.push_back((*m_pGraph)[vd].p);
        }
        ++i;
    }
    
    return roadMap;
}
