/**
 * @file    GridMap.h
 * @author  Jonathan Simmonds
 * @brief     A class to hold a basic map representation of the surroundings.
 */
#ifndef GRIDMAP_H
#define GRIDMAP_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <queue>
#include "Common.h"




/*-------------------- DEFINES --------------------*/
#define POINT_EQUALITY_EPSILON   50  // in mm.
#define POINT_EQUALITY_EPSILON2  (POINT_EQUALITY_EPSILON*POINT_EQUALITY_EPSILON)
#define TABLE_WEIGHT_INCREASE_GAP 3 // table weights will be updated by 1 every x seconds.




/*-------------- CLASS DEFINITIONS --------------*/
class AdjNode
{
public:
    uint32_t id;
    float    prox;

    AdjNode (uint32_t id_init, float prox_init) : id(id_init), prox(prox_init) {}
};
class GridNode
{
public:
    std::vector<AdjNode> adj;
    uint32_t id;
    float    dist;
    Point2i  p;

    GridNode (uint32_t id_init, int x_init, int y_init) : id(id_init), p(x_init, y_init) {}
    GridNode (uint32_t id_init, Point2i p_init) :         id(id_init), p(p_init) {}
    
    bool operator> (const GridNode& other)
    {
        return (dist > other.dist);
    }
};
class GridNodeComparator
{
public:
    bool operator() (const GridNode* a, const GridNode* b)
    {
        return (a->dist > b->dist);
    }
};



/**
 * @brief   A class to hold a basic collection of nodes as a discreet grid map. It stores at the
 *          millimeter resolution but is typically only reliable to the centimetre at most.
 */
class GridMap
{
public:
    /// @brief  Constructor.
    GridMap (void) : mCurrentOrientation(0), mCurrentNode(0)
    { 
        push_back(Point2i(0,0));  // we always start at (0,0)
    }
    
    
    /// @brief  Deconstructor
    ~GridMap (void) {}
    
    
    /// @brief  Adds the readings supplied to the map as a new node.
    ///         NB: THIS FUNCTION MAKES THE EXTREMELY DANGEROUS ASSUMPTION THAT THE ROTATION WAS
    ///             PERFORMED AFTER THE DISTANCE WAS TRAVELLED. THIS IS RARELY THE CASE HOWEVER
    ///             THERE IS NO WAY TO ACCURATELY DISCERN THE TRANSFORMATION FROM THESE READINGS.
    ///             As a result it is strongly advised to call this function after EITHER a rotation
    ///             OR a translation, meaning the graph will be uniquely made up of straight lines.
    /// @param  distance    The distance the robot has travelled since the previous GridNode was
    ///                     added. In mm. If this is 0 a new node will not be created but the
    ///                     orientation will be updated (to avoid placing nodes on top of each other).
    /// @param  angle       The angle the robot has turned through since the previous GridNode was
    ///                     added. In degrees.
    void addRelativeReadings (sint16_t distance, sint16_t angle)
    {
        if (distance != 0 && angle != 0)
            printf("WARNING: The map was updated with both a non-zero distance and angle (d=%d, a=%d). This can lead to errors.\n", distance, angle);
        
        if (distance != 0)
        {
            // apply the translation
            push_back(Point2i(mGraph[mCurrentNode].p, distance, mCurrentOrientation));
        }
        
        // apply the rotation (AFTER the translation). Adjusts the current orientation.
        mCurrentOrientation += angle;
        // fix overflow
        while (mCurrentOrientation > 360)
            mCurrentOrientation -= 360;
        while (mCurrentOrientation < -360)
            mCurrentOrientation += 360;
    }
    
    void addTable (uint32_t node_id)
    {
        mTableNodes.push_back(std::pair<uint32_t, uint8_t>(node_id, 0));
        if (mTableNodes.size() == 1)
            time(&mLastWeightUpdateTime);
    }
    
    void updateWeights (void)
    {
        int time_difference, point_increase;
        uint32_t i = 0;
        time_t currentTime;
        time(&currentTime);
        
        // check the time elapsed since the last update
        time_difference = (int) difftime(currentTime, mLastWeightUpdateTime);
        if (time_difference < TABLE_WEIGHT_INCREASE_GAP)
            return;
        
        // if necessary update the weights
        point_increase = time_difference / TABLE_WEIGHT_INCREASE_GAP;
        for (i = 0; i < mTableNodes.size(); i++)
            if (mTableNodes[i].second < 100)
                mTableNodes[i].second = MAX((mTableNodes[i].second + point_increase), 100);
        
        // if we have updated the weights, update the time at which we last did so.
        time(&mLastWeightUpdateTime);
    }
    
    void testGraphing (void)
    {
        // build the graph
        std::vector<uint32_t> adjacent_ids;
        // A (0)
        //push_back(Point2i(0, 0)); //-- this is done when the GridMap is initialised. it won't break if we do it again but there's no point.
        // B (1)
        adjacent_ids.clear();
        adjacent_ids.push_back(0);
        insert(Point2i(0, 10), adjacent_ids);
        // C (2)
        adjacent_ids.clear();
        adjacent_ids.push_back(0);
        adjacent_ids.push_back(1);
        insert(Point2i(5, 5), adjacent_ids);
        // D (3)
        adjacent_ids.clear();
        adjacent_ids.push_back(1);
        adjacent_ids.push_back(2);
        insert(Point2i(10, 10), adjacent_ids);
        
        // test the graph
        std::vector<uint32_t> results = dijkstra(0, 3);
        for (uint32_t i = 0 ; i < results.size(); i++)
            printf("results[%d] = %d\n", i, results[i]);
    }
    
    /// @brief  Returns the number of the node associated with the supplied point or UINT32_MAX if
    ///         the point doesn't exist in the graph.
    uint32_t lookupPoint (const Point2i& p)
    {
        for (uint32_t i = 0; i < mGraph.size(); i++)
            if (mGraph[i].p == p)
                return i;
        return UINT32_MAX;
    }
    
    uint32_t tableWeightsSum (void)
    {
        uint32_t i, total = 0;
        for (i = 0; i < mTableNodes.size(); i++)
            total += mTableNodes[i].second;
        return total;
    }
    
    uint32_t getNearestBackwardNode (const Point2i& p)
    {
        uint32_t graph_size = mGraph.size();
        uint32_t i, min_node;
        float min_dist, dist;
        int lineOrientation = (mCurrentOrientation + 90) % 360;
        std::vector<uint32_t> backward_nodes;
        
        // make sure we actually have a graph to check against.
        if (graph_size == 0)
            return 0;
        
        // find the subset of the graph which falls behind the given point.
        for (i = 0; i < graph_size; i++)
            if ((mGraph[i].p != p) && (behind_line(p, mGraph[i].p, lineOrientation)))
                backward_nodes.push_back(i);
        // check we have actually found something.
        if (backward_nodes.size() == 0)
            printf("Uh oh... we don't seem to have any backward nodes?\n");
        
        // calculate the closest node out of the found ones.
        min_dist = INFINITY;
        min_node = 0;
        for (i = 0; i < backward_nodes.size(); i++)
        {
            dist = euclidean_distance(p, mGraph[backward_nodes[i]].p);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_node = i;
            }
        }
        
        // return.
        return min_node;
    }
    
    
    // takes the id's of 2 nodes and returns the shortest path connecting them in mGraph as a list
    // with the start element at position[0] and the end element at position[-1]. returns an empty
    // list in the case there is no path between the nodes.
    std::vector<uint32_t> dijkstra (uint32_t start, uint32_t end)
    {
//        std::priority_queue<uint32_t, std::vector<uint32_t>, GridNodeComparator> Q;
        std::priority_queue<GridNode*, std::vector<GridNode*>, GridNodeComparator> Q;
        std::vector<uint32_t> shortest_path;
        uint32_t* prev = (uint32_t*) malloc(mGraph.size() * sizeof(uint32_t));
        uint32_t u, v, i;
        float    alt;
            
        // initialise
        for (i = 0; i < mGraph.size(); i++)
        {
            prev[i]        = UINT32_MAX;
            mGraph[i].dist = INFINITY;
        }
        mGraph[start].dist = 0;
        Q.push(&mGraph[start]);
        
        while (!Q.empty())
        {
            // grab the minimum element (and check it's not whack).
            u = Q.top()->id;
            Q.pop();
            if (mGraph[u].dist == INFINITY || u == end)
                break;

            // relax the dijkstra
            for (i = 0; i < mGraph[u].adj.size(); i++)
            {
                v   = mGraph[u].adj[i].id;                      // the ith neighbour of u
                alt = mGraph[u].dist + mGraph[u].adj[i].prox;   // the total distance to v
                if (alt < mGraph[v].dist)
                {
                    mGraph[v].dist = alt;
                    prev[v] = u;
                    Q.push(&mGraph[v]);
                }
            }
        }
        
        // extract the shortest path.
        u = end;
        while (prev[u] != UINT32_MAX)
        {
            shortest_path.push_back(u);
            u = prev[u];
        }
        if (shortest_path.size() > 0)
        {
            shortest_path.push_back(u);
            std::reverse(shortest_path.begin(), shortest_path.end());
        }
        
        // clean up + return.
        free(prev);
        return shortest_path;
        //return mGraph[end].dist;
    }
    

    sint16_t mCurrentOrientation;    ///< currentOrientation in degrees. This is measured from the north vector. Should always be -360 < x < 360.
    uint32_t mCurrentNode;
    std::vector<GridNode> mGraph;
    std::vector< std::pair<uint32_t, uint8_t> > mTableNodes;
    
private:
    
    time_t mLastWeightUpdateTime;
    
    bool points_roughly_equal (const Point2i& p1, const Point2i& p2)
    {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        return (dx*dx + dy*dy < POINT_EQUALITY_EPSILON2);
    }
    
    
    /// @brief  Returns true iff p2 falls behind the line made by a line at angle gamma through p1.
    ///         a value of 90 for gamma will return true for all points with a y co-ordinate less than
    ///         p1. likewise a value of 0 for gamma will return true for all points with an x co-ordinate
    ///         greater than p1.
    bool behind_line (const Point2i& p1, const Point2i& p2, const int gamma)
    {
        float gamma_rad = DEGTORAD(gamma);
        Point2i p1_prime(p1.x + ((int) (100 * sin(gamma_rad))), p1.y + ((int) (100 * cos(gamma_rad))));
        
        int dot_product = (p1_prime.x - p1.x)*(p2.y - p1.y) - (p1_prime.y - p1.y)*(p2.x - p1.x);
        return (dot_product < 0);
    }
    
    
    void push_back (Point2i p)
    {
        uint32_t i;
        uint32_t graph_size = mGraph.size();
        
        if (graph_size == 0)
        {
            // make a new node.
            GridNode n_new(graph_size, p);
            
            // add the new node to the graph and update our position.
            mGraph.push_back(n_new);
            mCurrentNode = graph_size;
        }
        else
        {
            // first check we're not trying to add a node that already exists.
            uint32_t equal_node = UINT32_MAX;
            for (i = 0; i < graph_size; i++)
                if (points_roughly_equal(p, mGraph[i].p))
                    equal_node = i;
            
            if (equal_node == UINT32_MAX)
            {
                // okay, we're cool - the node doesn't exist, let's make it and move on.
                GridNode n_new(graph_size, p);
                
                GridNode n_old = mGraph[mCurrentNode];
                float node_distance = euclidean_distance(n_old.p, p);
                
                // add the previous node to the new node's adjacency.
                AdjNode a_new(mCurrentNode, node_distance);
                n_new.adj.push_back(a_new);

                // add the new node to the previous node's adjacency.
                AdjNode a_old(graph_size,   node_distance);
                n_old.adj.push_back(a_old);
                
                // add the new node to the graph and update our position.
                mGraph.push_back(n_new);
                mCurrentNode = graph_size;
            }
            else
            {
                // okay we've actually just moved to an existing node... lets sort this out.
                // Note that we no longer use p, instead using the position of the nearby node.
                GridNode n_new = mGraph[equal_node];
                GridNode n_old = mGraph[mCurrentNode];
                
                // check we're not moving to ourself (in which case we may as well exit).
                if (equal_node == mCurrentNode)
                    return;
                // check we're not moving along a known edge (in which case we may as well exit).
                for (i = 0; i < n_old.adj.size(); i++)
                    if (n_old.adj[i].id == equal_node)
                        return;
                
                // okay, we're making a new edge but between two existing nodes... we can do this!
                float node_distance = euclidean_distance(n_new.p, n_old.p);
                
                // add the old node to the new_node's adjacency
                AdjNode a_new(mCurrentNode, node_distance);
                n_new.adj.push_back(a_new);
                
                // add the new node to the old node's adjacency.
                AdjNode a_old(equal_node,   node_distance);
                n_old.adj.push_back(a_old);
                
                // update our position.
                mCurrentNode = equal_node;
            }
        }
    }
    
    
    void insert (Point2i p, std::vector<uint32_t> connection_ids)
    {
        uint32_t i, graph_size = mGraph.size();
        std::vector<float> connection_distances;
        
        // calculate the connection distances
        for (i = 0; i < connection_ids.size(); i++)
            connection_distances.push_back(euclidean_distance(p, mGraph[connection_ids[i]].p));

        // make the new node and add the adjacent nodes to it.
        GridNode n_new(graph_size, p);
        for (i = 0; i < connection_ids.size(); i++)
            n_new.adj.push_back(AdjNode(connection_ids[i], connection_distances[i]));

        // add the new node to the adjacent nodes.
        AdjNode a_old(graph_size, UINT32_MAX);
        for (i = 0; i < connection_ids.size(); i++)
            mGraph[connection_ids[i]].adj.push_back(AdjNode(graph_size, connection_distances[i]));

        // add the new node to the graph.
        mGraph.push_back(n_new);
    }
};

#endif
