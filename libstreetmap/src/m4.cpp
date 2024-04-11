#include "m4.h"
#include <cmath>
#include <chrono>
#include <list>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m2helpers.h"
#include "m3helpers.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>
#include <queue>
#include <limits>


#define NO_EDGE -1
#define STARTING_TIME 0.0

// ==================================== Declare Structs ====================================
struct connected_intersection_data{
    IntersectionIdx intersectionId;
    ezgl::point2d position;
    StreetIdx streetId;
    int primaryStreet;
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
   double bestTime = std::numeric_limits<double>::infinity();
};

struct WaveElemM4{
    IntersectionIdx nodeID;
    StreetSegmentIdx edgeID;
    IntersectionIdx reachingNodeID;
    double travelTime;
    int primaryStreet;
    WaveElemM4(int intersection, int segment, int inter2, float time, int s) {nodeID = intersection; edgeID = segment; reachingNodeID = inter2; travelTime = time; primaryStreet = s;}
    bool operator<(const WaveElemM4 other) const {
        return travelTime > other.travelTime;
    }
};



// ==================================== Declare Globals ====================================
extern std::vector<std::vector<connected_intersection_data>> connectedIntersections;
extern std::vector<Intersection_data> intersections;
extern float cos_latavg;

void resetNodes(std::vector<int> nodes);
std::vector<StreetSegmentIdx> retracePath(int nodeId, int startingNode);

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots)
{


    
}


std::vector<StreetSegmentIdx> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids) {

    auto startTime = std::chrono::high_resolution_clock::now();

    std::priority_queue<WaveElemM4> openHeap;
    std::vector<IntersectionIdx> nodesToReset;
    ezgl::point2d destination = intersections[intersect_ids.second].position;
    openHeap.push(WaveElemM4(intersect_ids.first, NO_EDGE, NO_EDGE, STARTING_TIME, NO_EDGE));
    nodesToReset.push_back(intersect_ids.first);

    while(!openHeap.empty()){
        WaveElemM4 node = openHeap.top();

        openHeap.pop();

        IntersectionIdx nodeId = node.nodeID;

        if( node.travelTime < intersections[nodeId].bestTime){

            intersections[nodeId].reachingEdge = node.edgeID;
            intersections[nodeId].reachingNode = node.reachingNodeID;
            intersections[nodeId].bestTime = node.travelTime;

            if(nodeId == intersect_ids.second){
                std::vector<StreetSegmentIdx> path = retracePath(intersect_ids.second,intersect_ids.first);
                resetNodes(nodesToReset);

                auto currTime = std::chrono::high_resolution_clock::now();
                auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
                std::cout << "findPath took " << wallClock.count() <<" seconds" << std::endl;
                return path;
            }

            for(int connectedNode = 0; connectedNode < connectedIntersections[nodeId].size(); connectedNode++){

                connected_intersection_data newData = connectedIntersections[nodeId][connectedNode];

                if(newData.intersectionId == node.reachingNodeID){
                    continue;
                }

                nodesToReset.push_back(newData.intersectionId);

                double delay;

                if(newData.primaryStreet != node.primaryStreet){
                    delay = turn_penalty;
                }else{
                    delay = 0.0;
                }
                
                double time = newData.travel_time + node.travelTime + delay;

                openHeap.push(WaveElemM4(newData.intersectionId, newData.streetId, nodeId, time, newData.primaryStreet));
            }
        }
    }
    return {0};
}

void resetNodes(std::vector<int> nodes){
    for(int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++){
        intersections[nodes[nodeIndex]].processed = false;
        intersections[nodes[nodeIndex]].reachingEdge = 0;
        intersections[nodes[nodeIndex]].reachingNode = 0;
        intersections[nodes[nodeIndex]].bestTime = std::numeric_limits<double>::infinity();
    }
}

std::vector<StreetSegmentIdx> retracePath(int nodeId, int startingNode){

    std::list<StreetSegmentIdx> path;
    int index = nodeId;
    while(index != startingNode ){
        path.push_front(intersections[index].reachingEdge);
        index = intersections[index].reachingNode;
    }
    return std::vector<StreetSegmentIdx>(path.begin(), path.end());
}