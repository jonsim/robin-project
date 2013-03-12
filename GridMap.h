/**
 * @file    GridMap.h
 * @author  Jonathan Simmonds
 * @brief     A class to hold a basic map representation of the surroundings.
 */
#ifndef GRIDMAP_H
#define GRIDMAP_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "Common.h"




/*-------------------- DEFINES --------------------*/
// None




/*-------------- CLASS DEFINITIONS --------------*/
class GridNode
{
public:
    GridNode  (const sint32_t x_init, const sint32_t y_init) : x(x_init), y(y_init), weight(1.0f) {}
    ~GridNode (void) {}
    sint32_t x, y;
    float    weight;
};



/**
 * @brief   A class to hold a basic collection of nodes as a discreet grid map. It stores at the
 *          millimeter resolution but is typically only reliable to the centimetre at most.
 */
class GridMap
{
public:
    /// @brief  Constructor.
    GridMap  (void) : currentOrientation(0)
    {
        mMap.push_back(GridNode(0, 0)); // we always start at (0,0)
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
        sint32_t nx, ny;
        
        if (distance != 0 && angle != 0)
            printf("WARNING: The map was updated with both a non-zero distance and angle (d=%d, a=%d). This can lead to errors.\n", distance, angle);
        
        if (distance != 0)
        {
            // apply the translation
            nx = mMap.back().x + (distance * sin(currentOrientation));
            ny = mMap.back().y + (distance * cos(currentOrientation));
            mMap.push_back(GridNode(nx, ny));
        }
        
        // apply the rotation (AFTER the translation). Adjusts the current orientation.
        currentOrientation += angle;
        // fix overflow
        while (currentOrientation > 360)
            currentOrientation -= 360;
        while (currentOrientation < -360)
            currentOrientation += 360;
    }


private:
    sint16_t currentOrientation;    ///< currentOrientation in degrees. This is measured from the
                                    ///  north vector. Should always be -360 < x < 360.
    std::vector<GridNode> mMap;     ///< The vector used to store the map's nodes.
};

#endif
