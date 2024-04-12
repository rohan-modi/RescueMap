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
std::vector<StreetSegmentIdx> retracePathM4(int startingNode, int nodeId);
std::vector<IntersectionIdx> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            IntersectionIdx intersect_Start, 
            int combinations, 
            std::unordered_set<int> destinationSet,
            std::unordered_set<int> depotSet,
            bool includeDepots
            );

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots)
{
    auto startTime = std::chrono::high_resolution_clock::now();
    int points = deliveries.size()*2;
    int indexTracker = 0;

    std::unordered_map<int, int> intersectionVectorIndicies;
    std::unordered_map<int, int> intersectionToDeliveryId;
    
    std::unordered_set<int> destinationSet;
    std::unordered_set<int> depotSet;

    std::vector<IntersectionIdx> indexToIntersectionId;
    
    std::vector<std::vector<TravelMatrixElem>> travelTimeMatrix;

    for(int i = 0; i < deliveries.size(); i++){
        destinationSet.insert(deliveries[i].pickUp);
        destinationSet.insert(deliveries[i].dropOff);
        indexToIntersectionId.push_back(deliveries[i].pickUp);
        intersectionVectorIndicies.insert(std::make_pair(deliveries[i].pickUp, indexTracker));
        indexTracker++;
        indexToIntersectionId.push_back(deliveries[i].dropOff);
        intersectionVectorIndicies.insert(std::make_pair(deliveries[i].dropOff, indexTracker));
        indexTracker++;
        intersectionToDeliveryId.insert(std::make_pair(deliveries[i].pickUp, i));
        intersectionToDeliveryId.insert(std::make_pair(deliveries[i].dropOff, i));
    }
    for(int i = 0; i < depots.size(); i++){
        depotSet.insert(depots[i]);
        indexToIntersectionId.push_back(depots[i]);
        intersectionVectorIndicies.insert(std::make_pair(depots[i], indexTracker));
        indexTracker++;
    }
    
    //Paths between points and other points + depots
    for(int i = 0; i < points; i++){
        std::vector<IntersectionIdx> nodesToReset = findPathBetweenIntersectionsM4(
            turn_penalty,
            indexToIntersectionId[i], 
            points + depots.size(), 
            destinationSet,
            depotSet,
            true
            );
        std::vector<TravelMatrixElem> tempData;

        for(int j = 0; j < points + depots.size(); j++){
            //std::cout<<"here" << std::endl;

            
            TravelMatrixElem data;

            data.to = indexToIntersectionId[j];
            data.from = indexToIntersectionId[i];

            //bugged (only used for better optimization doesnt retrace steps if its from drop off to pick up of same delivery)
            // for(const auto& pair: intersectionToDeliveryId){
            //     std::cout << "key: "<< pair.first << "value: " << pair.second<<std::endl;
            //     std::cout<< "finding " << intersectionToDeliveryId.find(data.from)->first<< std::endl;
            // }
            
            // //if(intersectionToDeliveryId.find(data.from) != intersectionToDeliveryId.end() && intersectionToDeliveryId.find(data.to) != intersectionToDeliveryId.end())
            // if(intersectionToDeliveryId.find(data.from)->second == intersectionToDeliveryId.find(data.to)->second)
            // {
            //      std::cout<<deliveries[intersectionToDeliveryId.find(data.from)->second].pickUp<< std::endl;
            //     // if(deliveries[intersectionToDeliveryId.find(data.from)->second].pickUp == data.to){
            //     //     // data.path = {-1};
            //     //     // data.travelTime = std::numeric_limits<double>::infinity();
            //     //     // data.legal = false;
            //     //     // std::cout<< "here" << std::endl;
            //     //     // tempData.push_back(data);
            //     //     //continue;
            //     // }
            // }
            data.path = retracePathM4(indexToIntersectionId[i],indexToIntersectionId[j]);
            data.travelTime = intersections[indexToIntersectionId[j]].bestTime;
            data.legal = true;

            tempData.push_back(data);
        }

        travelTimeMatrix.push_back(tempData);
        //std::cout<< "here2" << std::endl;
        resetNodesM4(nodesToReset);
    }

    //Paths between deopts to points excluding paths to other depots
    for(int i = deliveries.size()*2; i < deliveries.size()*2 + depots.size(); i++){

        std::vector<IntersectionIdx> nodesToReset = findPathBetweenIntersectionsM4(
            turn_penalty,
            indexToIntersectionId[i], 
            points, 
            destinationSet,
            depotSet,
            false
            );
        std::vector<TravelMatrixElem> tempData;

        for(int j = 0; j < points; j++){
            //std::cout<<"here" << std::endl;
            TravelMatrixElem data;

            data.to = indexToIntersectionId[j];
            data.from = indexToIntersectionId[i];
            data.path = retracePathM4(indexToIntersectionId[i],indexToIntersectionId[j]);
            data.travelTime = intersections[indexToIntersectionId[j]].bestTime;
            data.legal = true;

            tempData.push_back(data);
        }

        travelTimeMatrix.push_back(tempData);

        resetNodesM4(nodesToReset);
    }

    //DRAWS OUT MATRIX
    for(int i = 0; i< 6; i++){
        for(int j = 0; j < travelTimeMatrix[i].size(); j++){
            if(travelTimeMatrix[i][j].legal)
            std::cout<<travelTimeMatrix[i][j].travelTime << "       ";
            else
            std::cout<<"false  ";
            if(travelTimeMatrix[i][j].travelTime == 0)
                std::cout<<"      ";

        }
        std::cout<<std::endl;
    }

    //Empty return
    CourierSubPath data;
    std::vector<CourierSubPath> temp;
    temp.push_back(data);

    //timing analysis
    auto currTime = std::chrono::high_resolution_clock::now();
                auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
                std::cout << "findPath took " << wallClock.count() <<" seconds" << std::endl;

    return temp;
}


std::vector<IntersectionIdx> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            IntersectionIdx intersect_Start, 
            int combinations, 
            std::unordered_set<int> destinationSet,
            std::unordered_set<int> depotSet,
            bool includeDepots
            ) {

    

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

        if( node.travelTime < intersections[nodeId].bestTime){

            intersections[nodeId].reachingEdge = node.edgeID;
            intersections[nodeId].reachingNode = node.reachingNodeID;
            intersections[nodeId].bestTime = node.travelTime;
            
            if(destinationSet.find(nodeId)!= destinationSet.end()|| (includeDepots && depotSet.find(nodeId) != depotSet.end())){
                pathsFound++;
            }

            if(pathsFound > combinations){
                //std::vector<TravelMatrixElem> path = retracePaths(intersect_ids.second,intersect_ids.first);
                //resetNodes(nodesToReset);
                
                return nodesToReset;
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

    return nodesToReset;
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
            int startingNode, int nodeId){
    std::list<StreetSegmentIdx> path;
    int index = nodeId;
    while(index != startingNode ){
        path.push_front(intersections[index].reachingEdge);
        index = intersections[index].reachingNode;
    
    }
    //std::cout<<"EXITED" << std::endl;
    return std::vector<StreetSegmentIdx>(path.begin(), path.end());
}