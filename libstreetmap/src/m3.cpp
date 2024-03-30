// /* 
//  * Copyright 2024 University of Toronto
//  *
//  * Permission is hereby granted, to use this software and associated 
//  * documentation files (the "Software") in course work at the University 
//  * of Toronto, or for personal use. Other uses are prohibited, in 
//  * particular the distribution of the Software either publicly or to third 
//  * parties.
//  *
//  * The above copyright notice and this permission notice shall be included in 
//  * all copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  * SOFTWARE.
//  */

#include <cmath>
#include <sstream>
#include <vector>
#include <queue>
#include <chrono>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>

extern float cos_latavg;

//External structs
struct connected_intersection_data{
    IntersectionIdx intersectionId;
    ezgl::point2d position;
    StreetIdx streetId;
    double distance;
    double speedlimit;
    double travel_time;
    bool open;
    bool closed;
};

struct Intersection_data {
   ezgl::point2d position;
   std::string name;
   bool highlight = false;
   bool processed = false;
   int reachingEdge = 0;
   int reachingNode = 0;
   double bestTime = 0;
};


//External Globals
extern std::vector<std::vector<connected_intersection_data>> connectedIntersections;
extern std::vector<Intersection_data> intersections;

struct WaveElem {
    IntersectionIdx nodeID;
    StreetSegmentIdx edgeID;
    IntersectionIdx reachingNodeID;
    double travelTime;
    WaveElem(int intersection, int segment, int inter2, float time) {nodeID = intersection; edgeID = segment; reachingNodeID = inter2; travelTime = time;}
    bool operator<(const WaveElem other) const {
        return travelTime > other.travelTime;
    }
};

#define NO_EDGE -1
#define STARTING_TIME 0.0


















double findDistanceBetweenTwoPointsxym3(ezgl::point2d point_1, ezgl::point2d point_2);
std::vector<StreetSegmentIdx> retracePath(int nodeId, int startingNode);
ezgl::point2d latlon_to_pointm3(LatLon position);
void resetNodes(std::vector<int> nodes);


// Returns the time required to travel along the path specified, in seconds.
// The path is given as a vector of street segment ids, and this function can
// assume the vector either forms a legal path or has size == 0. The travel
// time is the sum of the length/speed-limit of each street segment, plus the
// given turn_penalty (in seconds) per turn implied by the path. If there is
// no turn, then there is no penalty. Note that whenever the street id changes
// (e.g. going from Bloor Street West to Bloor Street East) we have a turn.
double computePathTravelTime(const double turn_penalty,
                             const std::vector<StreetSegmentIdx>& path) {
    
    // If empty path, then return 0 travel time
    if (path.size() == 0) {
        return 0.0;
    }

    // Initialize travelTime sum and previous streetID variables.
    // We keep track of previous streetID to determine if the streetID
    // changes, thus indicating when a turn penalty should be applied.
    double travelTime = 0.0;
    StreetIdx prevStreetIdx = getStreetSegmentInfo(path[0]).streetID;

    // Loop through all street segments in the given path:
    for (int segmentNum = 0; segmentNum < path.size(); segmentNum++) {
        
        // Add the travel time for street segment:
        travelTime += findStreetSegmentTravelTime(path[segmentNum]);

        // Retrieve the streetID for the current segment:
        StreetIdx currStreetIdx = getStreetSegmentInfo(path[segmentNum]).streetID;

        // Add the turn penalty if the current streetID is different
        // from the previous streetID:
        if (currStreetIdx != prevStreetIdx) {
            travelTime += turn_penalty;
        }

        // Update the previous streetID to the current streetID
        prevStreetIdx = currStreetIdx;
    }

    return travelTime;
}


// Returns a path (route) between the start intersection (intersect_id.first)
// and the destination intersection (intersect_id.second), if one exists.
// This routine should return the shortest path
// between the given intersections, where the time penalty to turn right or
// left is given by turn_penalty (in seconds). If no path exists, this routine
// returns an empty (size == 0) vector. If more than one path exists, the path
// with the shortest travel time is returned. The path is returned as a vector
// of street segment ids; traversing these street segments, in the returned
// order, would take one from the start to the destination intersection.
std::vector<StreetSegmentIdx> findPathBetweenIntersections(
            const double turn_penalty,
            const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {

    std::priority_queue<WaveElem> openHeap;
    std::vector<IntersectionIdx> nodesToReset;

    openHeap.push(WaveElem(intersect_ids.first, NO_EDGE, NO_EDGE, STARTING_TIME));
    nodesToReset.push_back(intersect_ids.first);

    ezgl::point2d destination= latlon_to_pointm3(getIntersectionPosition(intersect_ids.second));


    while(!openHeap.empty()){
        WaveElem node = openHeap.top();
        openHeap.pop();

        IntersectionIdx nodeId = node.nodeID;
        std::cout<<"loop"<<std::endl;

        if( node.travelTime < intersections[nodeId].bestTime){
            std::cout<< "node time:" <<node.travelTime << std::endl;
            std::cout<< "inter time:" <<intersections[nodeId].bestTime << std::endl;
            intersections[nodeId].reachingEdge = node.edgeID;
            intersections[nodeId].reachingNode = node.reachingNodeID;
            intersections[nodeId].bestTime = node.travelTime;
        }

        if(nodeId == intersect_ids.second){

            std::vector<StreetSegmentIdx> path = retracePath(nodeId,intersect_ids.first);
            resetNodes(nodesToReset);
            return path;
        }

        for(int connectedNode = 0; connectedNode < connectedIntersections[nodeId].size(); connectedNode++){

            connected_intersection_data newData = connectedIntersections[nodeId][connectedNode];

             //std::cout<< "inter time:" <<newData.travel_time << std::endl;

            if(intersections[newData.intersectionId].processed){
                continue;
            }
            nodesToReset.push_back(newData.intersectionId);

            
            double time = newData.travel_time + node.travelTime + findDistanceBetweenTwoPointsxym3(newData.position, destination);
            intersections[newData.intersectionId].processed = true;
            intersections[newData.intersectionId].reachingEdge = newData.streetId;
            intersections[newData.intersectionId].reachingNode = nodeId;
            intersections[nodeId].bestTime = time;

            

            openHeap.push(WaveElem(newData.intersectionId, newData.streetId, nodeId,time ));

        }



    }
    std::cout<<"noPath"<<std::endl;
    return {0};
}

void resetNodes(std::vector<int> nodes){
    for(int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++){
        intersections[nodes[nodeIndex]].processed = false;
        intersections[nodes[nodeIndex]].reachingEdge = 0;
        intersections[nodes[nodeIndex]].reachingNode = 0;
        intersections[nodes[nodeIndex]].bestTime = 1000000000;
    }
}

std::vector<StreetSegmentIdx> retracePath(int nodeId, int startingNode){

    std::vector<StreetSegmentIdx> path;
    int index = nodeId;
    while(index != startingNode &&index != 0){
        path.push_back(intersections[index].reachingNode);
        index = intersections[index].reachingNode;
        
        std::cout<< intersections[index].reachingEdge <<std::endl;
            
    }
    
    return path;

}


double findDistanceBetweenTwoPointsxym3(ezgl::point2d point_1, ezgl::point2d point_2) {
    // Return the distance by the Pythagoras theorem: d = sqrt((y2 - y1)^2, (x2 - x1)^2) [m]
    return sqrt(pow(point_2.y - point_1.y, 2) + pow(point_2.x - point_1.x, 2));
}

ezgl::point2d latlon_to_pointm3(LatLon position){
   float x = kEarthRadiusInMeters * kDegreeToRadian * position.longitude() * cos_latavg;
   float y = kEarthRadiusInMeters * kDegreeToRadian * position.latitude();

   return(ezgl::point2d(x,y));
}