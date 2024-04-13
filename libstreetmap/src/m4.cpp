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

#include <unordered_set>



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

struct TravelMatrixElem{
    IntersectionIdx to;
    IntersectionIdx from;
    std::vector<StreetSegmentIdx> path;
    double travelTime;
    bool legal = false;
};



// ==================================== Declare Globals ====================================
extern std::vector<std::vector<connected_intersection_data>> connectedIntersections;
extern std::vector<Intersection_data> intersections;
extern float cos_latavg;

void resetNodesM4(std::vector<int> nodes);
std::vector<StreetSegmentIdx> retracePathM4(int startingNode, int nodeId, std::vector<Intersection_data>& intersectionLinks);
std::vector<TravelMatrixElem> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            IntersectionIdx intersect_Start, 
            int combinations, 
            bool includeDepots,
            const std::vector<DeliveryInf>& deliveries,
            const std::vector<IntersectionIdx>& depots
            );

    std::unordered_map<int, int> intersectionVectorIndicies;
    std::unordered_map<int, int> intersectionToDeliveryId;
    std::unordered_set<int> destinationSet;
    std::unordered_set<int> depotSet;
    std::vector<IntersectionIdx> indexToIntersectionId;
    std::vector<std::vector<TravelMatrixElem>> travelTimeMatrix;

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots)
{

     //=====================================START OF MATRIX SETUP================================================
    auto startTime = std::chrono::high_resolution_clock::now();
    int points = deliveries.size()*2;
    int indexTracker = 0;

    intersectionVectorIndicies.clear();
    intersectionToDeliveryId.clear();
    destinationSet.clear();
    depotSet.clear();
    indexToIntersectionId.clear();
    for(int i = 0; i< travelTimeMatrix.size(); i++){
        travelTimeMatrix[i].clear();
    }
    travelTimeMatrix.clear();
    travelTimeMatrix.resize(points+depots.size());

    //Populates data structs for delivery locations mapping interIds to indicies
    for(int i = 0; i < deliveries.size(); i++){
        destinationSet.insert(deliveries[i].pickUp);
        destinationSet.insert(deliveries[i].dropOff);
        indexToIntersectionId.push_back(deliveries[i].pickUp);
        indexToIntersectionId.push_back(deliveries[i].dropOff);
        intersectionToDeliveryId.insert(std::make_pair(deliveries[i].pickUp, i));
        intersectionToDeliveryId.insert(std::make_pair(deliveries[i].dropOff, i));
        intersectionVectorIndicies.insert(std::make_pair(deliveries[i].pickUp, indexTracker));
        indexTracker++;
        intersectionVectorIndicies.insert(std::make_pair(deliveries[i].dropOff, indexTracker));
        indexTracker++;
    }
    //Populates data structs for depot locations mapping interIds to indicies
    for(int i = 0; i < depots.size(); i++){
        depotSet.insert(depots[i]);
        indexToIntersectionId.push_back(depots[i]);
        intersectionVectorIndicies.insert(std::make_pair(depots[i], indexTracker));
        indexTracker++;
    }
    
    //Paths between points and other points + depots
    
    
    #pragma omp parallel for
    for(int i = 0; i < points; i++){
        travelTimeMatrix[i] = findPathBetweenIntersectionsM4(
            turn_penalty,
            indexToIntersectionId[i], 
            points + depots.size(), 
            true,
            deliveries,
            depots
            );
    }

    #pragma omp parallel for
    for(int i = deliveries.size()*2; i < deliveries.size()*2 + depots.size(); i++){

        travelTimeMatrix[i]  = findPathBetweenIntersectionsM4(
            turn_penalty,
            indexToIntersectionId[i], 
            points, 
            false,
            deliveries,
            depots
            );
        
    }

    //=====================================END OF MATRIX SETUP================================================







    //Min spanning Tree
    







    // //DRAWS OUT MATRIX
    // for(int i = 0; i< 6; i++){
    //     for(int j = 0; j < travelTimeMatrix[i].size(); j++){
    //         if(travelTimeMatrix[i][j].legal)
    //         std::cout<<travelTimeMatrix[i][j].travelTime << "       ";
    //         else
    //         std::cout<<"false         ";
    //         if(travelTimeMatrix[i][j].travelTime == 0)
    //             std::cout<<"      ";

    //     }
    //     std::cout<<std::endl;
    // }

    // Empty return
    CourierSubPath data;
    std::vector<CourierSubPath> temp;
    temp.push_back(data);

    //timing analysis
    auto currTime = std::chrono::high_resolution_clock::now();
                auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
                std::cout << "findPath took " << wallClock.count() <<" seconds" << std::endl;
    return temp;
}


std::vector<TravelMatrixElem> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            IntersectionIdx intersect_Start, 
            int combinations, 
            bool includeDepots,
            const std::vector<DeliveryInf>& deliveries,
            const std::vector<IntersectionIdx>& depots
            ) {
    
    std::vector<Intersection_data> localIntersections = intersections; 

    std::unordered_set<int> visitedDestinations;  

    std::priority_queue<WaveElemM4> openHeap;
    std::vector<IntersectionIdx> nodesToReset;
    openHeap.push(WaveElemM4(intersect_Start, NO_EDGE, NO_EDGE, STARTING_TIME, NO_EDGE));
    nodesToReset.push_back(intersect_Start);
    int pathsFound = 1;

    while(!openHeap.empty()){
        WaveElemM4 node = openHeap.top();

        openHeap.pop();

        IntersectionIdx nodeId = node.nodeID;

        if( node.travelTime < localIntersections[nodeId].bestTime){

            localIntersections[nodeId].reachingEdge = node.edgeID;
            localIntersections[nodeId].reachingNode = node.reachingNodeID;
            localIntersections[nodeId].bestTime = node.travelTime;
            
            if(destinationSet.find(nodeId)!= destinationSet.end() || (includeDepots && depotSet.find(nodeId) != depotSet.end())){
                pathsFound++;
            }

            if(pathsFound > combinations){
                //std::vector<TravelMatrixElem> path = retracePaths(intersect_ids.second,intersect_ids.first);
                //resetNodes(nodesToReset);

                if(includeDepots){
                    std::vector<TravelMatrixElem> tempData;

                    
                    for(int j = 0; j < combinations; j++){
                        
                        TravelMatrixElem data;

                        data.to = indexToIntersectionId[j];
                        data.from = intersect_Start;

                        if(data.to == data.from){
                                data.path = {-1};
                                data.travelTime = std::numeric_limits<double>::infinity();
                                data.legal = false;
                                tempData.push_back(data);
                                continue;
                            }
                        
                        if(j < deliveries.size()*2){
                            if(intersectionToDeliveryId.find(data.from)->second == intersectionToDeliveryId.find(data.to)->second)
                            {
                                if(deliveries[intersectionToDeliveryId.find(data.from)->second].pickUp == data.to){
                                    data.path = {-1};
                                    data.travelTime = std::numeric_limits<double>::infinity();
                                    data.legal = false;
                                    
                                    tempData.push_back(data);
                                    continue;
                                }
                            }
                            
                        }
                        data.path = retracePathM4(intersect_Start,indexToIntersectionId[j],localIntersections );
                        data.travelTime = localIntersections[indexToIntersectionId[j]].bestTime;
                        data.legal = true;

                        tempData.push_back(data);
                    }
                    return tempData;
                }else{
                    std::vector<TravelMatrixElem> tempData;

                    for(int j = 0; j < deliveries.size()*2; j++){
                        TravelMatrixElem data;

                        data.to = indexToIntersectionId[j];
                        data.from = intersect_Start;
                        data.path = retracePathM4(intersect_Start,indexToIntersectionId[j],localIntersections );
                        data.travelTime = localIntersections[indexToIntersectionId[j]].bestTime;
                        data.legal = true;

                        tempData.push_back(data);
                    }
                    return tempData;
                }
                
                
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
    std::vector<TravelMatrixElem> empty;
    return empty;
}

void resetNodesM4(std::vector<int> nodes){
    for(int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++){
        intersections[nodes[nodeIndex]].processed = false;
        intersections[nodes[nodeIndex]].reachingEdge = 0;
        intersections[nodes[nodeIndex]].reachingNode = 0;
        intersections[nodes[nodeIndex]].bestTime = std::numeric_limits<double>::infinity();
    }
}

std::vector<StreetSegmentIdx> retracePathM4(
            int startingNode, int nodeId, std::vector<Intersection_data>& intersectionLinks){
    std::list<StreetSegmentIdx> path;
    int index = nodeId;
    while(index != startingNode ){
        path.push_front(intersectionLinks[index].reachingEdge);
        index = intersectionLinks[index].reachingNode;
    
    }
    //std::cout<<"EXITED" << std::endl;
    return std::vector<StreetSegmentIdx>(path.begin(), path.end());
}