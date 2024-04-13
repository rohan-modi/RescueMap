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
#include <random>
#include <algorithm>

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

struct twoOptData{
    bool legalPathFound;
    double timeDifference;
    double newTime;
    std::vector<IntersectionIdx> swappedSection;
    int swapStartIndex;
};



// ==================================== Declare Globals ====================================
extern std::vector<std::vector<connected_intersection_data>> connectedIntersections;
extern std::vector<Intersection_data> intersections;
extern float cos_latavg;

std::vector<bool> delivery_pickedup;
std::vector<bool> delivery_droppedoff;

double route_cost(const double turn_penalty, std::vector<CourierSubPath> route);

void resetNodesM4(std::vector<int> nodes);
bool twoOpt(std::vector<CourierSubPath>* initialPath, std::unordered_map<IntersectionIdx, std::unordered_set<IntersectionIdx>> deliveryInfos, double turnPenalty);
void twoOptAnneal(std::vector<CourierSubPath>* initialPath, std::unordered_map<IntersectionIdx, std::unordered_set<IntersectionIdx>> deliveryInfos, double turnPenalty, unsigned int seed, int perturbationSize, struct twoOptData* returnStruct);
bool checkLegal(std::unordered_map<IntersectionIdx, std::vector<IntersectionIdx>>* legalChecker, std::unordered_set<IntersectionIdx>* previousIntersections, IntersectionIdx nextIntersections);
IntersectionIdx findFastest(IntersectionIdx currentIntersection, std::unordered_map<IntersectionIdx, std::vector<IntersectionIdx>>* legalChecker, std::unordered_set<IntersectionIdx>* previousIntersections);
std::vector<StreetSegmentIdx> retracePathM4(int startingNode, int nodeId, std::vector<Intersection_data>& intersectionLinks);
std::vector<TravelMatrixElem> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            IntersectionIdx intersect_Start, 
            int combinations, 
            bool includeDepots,
            const std::vector<DeliveryInf>& deliveries,
            const std::vector<IntersectionIdx>& depots,
            int duplicates
            );

std::vector<CourierSubPath> get_greedy_route(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots);
//std::pair<int, int> get_nearest_legal(IntersectionIdx srcInter, const std::vector<DeliveryInf>& deliveries);
//bool is_legal_delivery(TravelMatrixElem& travel_elem, const std::vector<DeliveryInf>& deliveries);
//void reset_delivery_flags(const std::vector<DeliveryInf>& deliveries);
//bool deliveries_complete();


    std::unordered_map<int, int> intersectionVectorIndices;
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
    int duplicates = 0;

    intersectionVectorIndices.clear();
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


        if(destinationSet.find(deliveries[i].pickUp)!=destinationSet.end()){
            duplicates++;
        }
        if(destinationSet.find(deliveries[i].dropOff) != destinationSet.end()){
            duplicates++;
        }

            destinationSet.insert(deliveries[i].pickUp);
            indexToIntersectionId.push_back(deliveries[i].pickUp);
            intersectionVectorIndices.insert(std::make_pair(deliveries[i].pickUp, indexTracker));
            indexTracker++;

            
        
        
        
            destinationSet.insert(deliveries[i].dropOff);
            indexToIntersectionId.push_back(deliveries[i].dropOff);
            intersectionVectorIndices.insert(std::make_pair(deliveries[i].dropOff, indexTracker));
            indexTracker++;
       
        
        intersectionToDeliveryId.insert(std::make_pair(deliveries[i].pickUp, i));
        intersectionToDeliveryId.insert(std::make_pair(deliveries[i].dropOff, i));
    }
    //Populates data structs for depot locations mapping interIds to indicies
    for(int i = 0; i < depots.size(); i++){
        depotSet.insert(depots[i]);
        indexToIntersectionId.push_back(depots[i]);
        intersectionVectorIndices.insert(std::make_pair(depots[i], indexTracker));
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
            depots,
            duplicates
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
            depots,
            duplicates
            );
        
    }

    //=====================================END OF MATRIX SETUP================================================

    //Greedy TEST

    std::vector<CourierSubPath> returnPath;
    

    for(int i = 0; i< 1; i++){
        std::unordered_set<int> legalIntersection;

        for(int j = 0; j < deliveries.size(); j++){
            legalIntersection.insert(deliveries[j].pickUp);
        }

        bool pathFound = false;
        int minIndex = -1;
        int currentNodeIndex = depots[i];

        while(!pathFound){

            double minTime = std::numeric_limits<double>::infinity();
            

            for(auto iterator = legalIntersection.begin(); iterator != legalIntersection.end(); ++iterator){
                //std::cout<< *iterator <<std::endl;

                if(travelTimeMatrix[intersectionVectorIndices.find(currentNodeIndex)->second][intersectionVectorIndices.find(*iterator)->second].travelTime < minTime){
                    minTime = travelTimeMatrix[intersectionVectorIndices.find(currentNodeIndex)->second][intersectionVectorIndices.find(*iterator)->second].travelTime ;
                    minIndex = *iterator;
                }

            }

            legalIntersection.erase(minIndex);

            //std::cout<< minTime<< std::endl;

            if(deliveries[intersectionToDeliveryId.find(minIndex)->second].pickUp == minIndex){
                legalIntersection.insert(deliveries[intersectionToDeliveryId.find(minIndex)->second].dropOff);
            }
            CourierSubPath data; 
            data.intersections = std::make_pair(currentNodeIndex, minIndex);
            data.subpath = travelTimeMatrix[intersectionVectorIndices.find(currentNodeIndex)->second][intersectionVectorIndices.find(minIndex)->second].path;
            returnPath.push_back(data);
            currentNodeIndex = minIndex; 


            if(legalIntersection.empty()){
                data.intersections = std::make_pair(currentNodeIndex, depots[i]);
                data.subpath = travelTimeMatrix[intersectionVectorIndices.find(currentNodeIndex)->second][intersectionVectorIndices.find(depots[i])->second].path;
                returnPath.push_back(data);

                pathFound = true;
            }
        }
    }
    // std::cout << "TOTAL COST: " << route_cost(turn_penalty, returnPath) << std::endl;    

    // MAKING MAP FOR 2-OPT
    std::unordered_map<IntersectionIdx, std::unordered_set<IntersectionIdx>> deliveryRequirements;
    std::unordered_map<IntersectionIdx, std::vector<IntersectionIdx>> legalityChecking;

    // use vecotr instead of unordered set .insdert -> .push_back()
    for (int i = 0; i < deliveries.size(); i++) {
        auto dropOffIterator = deliveryRequirements.find(deliveries[i].dropOff);
        if (dropOffIterator != deliveryRequirements.end()) {
            deliveryRequirements[deliveries[i].dropOff].insert(deliveries[i].pickUp);
            legalityChecking[deliveries[i].dropOff].push_back(deliveries[i].pickUp);
        } else {
            deliveryRequirements[deliveries[i].dropOff] = {deliveries[i].pickUp};
            legalityChecking[deliveries[i].dropOff] = {deliveries[i].pickUp};
        }
    }





    //Min spanning Tree
    
    
    




    /*
    // //DRAWS OUT MATRIX
    for(int i = 0; i< travelTimeMatrix.size(); i++){
        for(int j = 0; j < travelTimeMatrix[i].size(); j++){
            if(travelTimeMatrix[i][j].legal)
            std::cout<<travelTimeMatrix[i][j].travelTime << "       ";
            else
            std::cout<<"false         ";
            if(travelTimeMatrix[i][j].travelTime == 0)
                std::cout<<"      ";
        }
        std::cout<<std::endl;
    }


    std::cout<< "finding" <<travelTimeMatrix[intersectionVectorIndices.find(23166)->second][intersectionVectorIndices.find(2432)->second].travelTime<<std::endl;

    std::vector<CourierSubPath> startPath;
    */


    // int counter = 0;
    std::cout << "Initial cost: " << route_cost(turn_penalty, returnPath) << std::endl;
    while (twoOpt(&returnPath, deliveryRequirements, turn_penalty)) {
        // std::cout << "Counter number: " << counter << std::endl;
        // counter++;
        auto currTime = std::chrono::high_resolution_clock::now();
        auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
        if (wallClock.count() >= 50*0.9) {
            break;
        }
    }
    std::cout << "Final cost: " << route_cost(turn_penalty, returnPath) << std::endl;
    return returnPath;


    /*

    // Parameters for annealing
    int perturbationSize = 10;
    int numberOfPerturbations = 50;
    double temperature = 1000;
    double tempMultiplier = 0.9;
    struct twoOptData perturbationData;
    while (1) {
        for (int i = 0; i < numberOfPerturbations; i++) {
            auto currTime = std::chrono::high_resolution_clock::now();
            auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
            if (wallClock.count() >= 50*0.9) {
                break;
            }
            twoOptAnneal(&startPath, deliveryRequirements, turn_penalty, (unsigned int) wallClock.count(), perturbationSize, &perturbationData);
            std::mt19937 generator(wallClock.count());
            std::uniform_real_distribution<double> distribution(0.0, 1.0);
            double randomNumber = distribution(generator);
            double exponential = std::exp((-1)*perturbationData.timeDifference/temperature);
            if (perturbationData.timeDifference < 0 || randomNumber < exponential) {
                for (int j = 0; j < perturbationData.swappedSection.size()+1; j++) {
                    std::pair<IntersectionIdx, IntersectionIdx> newPair;
                    newPair.first = perturbationData.swappedSection[j];
                    newPair.second = perturbationData.swappedSection[j+1];
                    startPath[perturbationData.swapStartIndex+j].intersections.first = newPair.first;
                    startPath[perturbationData.swapStartIndex+j].intersections.second = newPair.second;
                    int firstIndex = intersectionVectorIndices.find(newPair.first)->second;
                    int secondIndex = intersectionVectorIndices.find(newPair.second)->second;
                    startPath[perturbationData.swapStartIndex+j].subpath = travelTimeMatrix[firstIndex][secondIndex].path;
                }
            }
        }
        temperature *= tempMultiplier;
    }

    //DRAWS OUT MATRIX
    for(int i = 0; i< 6; i++){
        for(int j = 0; j < travelTimeMatrix[i].size(); j++){
            if(travelTimeMatrix[i][j].legal)
            std::cout<<travelTimeMatrix[i][j].travelTime << "       ";
            else
            std::cout<<"false         ";
            if(travelTimeMatrix[i][j].travelTime == 0)
                std::cout<<"      ";

        }
        std::cout<<std::endl;
    }

    // Empty return
    CourierSubPath data;
    std::vector<CourierSubPath> temp;
    temp.push_back(data);

    //std::vector<CourierSubPath> path = get_greedy_route(turn_penalty, deliveries, depots);


    //timing analysis
    auto currTime = std::chrono::high_resolution_clock::now();
    auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
    std::cout << "findPath took " << wallClock.count() <<" seconds" << std::endl;
    return temp;*/
}


std::vector<TravelMatrixElem> findPathBetweenIntersectionsM4(
            const double turn_penalty,
            IntersectionIdx intersect_Start, 
            int combinations, 
            bool includeDepots,
            const std::vector<DeliveryInf>& deliveries,
            const std::vector<IntersectionIdx>& depots,
            int duplicates
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

            if(pathsFound > combinations-duplicates){
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
                        
                        // if(j < deliveries.size()*2){
                        //     if(intersectionToDeliveryId.find(data.from)->second == intersectionToDeliveryId.find(data.to)->second)
                        //     {
                        //         if(deliveries[intersectionToDeliveryId.find(data.from)->second].pickUp == data.to){
                        //             data.path = {-1};
                        //             data.travelTime = std::numeric_limits<double>::infinity();
                        //             data.legal = false;
                                    
                        //             tempData.push_back(data);
                        //             continue;
                        //         }
                        //     }
                            
                        // }
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

bool twoOpt(std::vector<CourierSubPath>* initialPath, std::unordered_map<IntersectionIdx, std::unordered_set<IntersectionIdx>> deliveryInfos, double turnPenalty) {
    // std::cout << "IN FUNCTION" << std::endl;
    std::vector<IntersectionIdx> pathIntersections;
    int pathIntersectionsSize = initialPath->size()+1;
    int initialSolutionSize = initialPath->size();
    pathIntersections.resize(pathIntersectionsSize);
    for (int i =  0; i < pathIntersectionsSize; i++) {
        pathIntersections[i] = (*initialPath)[i].intersections.first;
    }
    // std::cout << "Past first for loop" << std::endl;
    pathIntersections[pathIntersectionsSize-1] = (*initialPath)[initialSolutionSize-1].intersections.second;

    for (int i = 1; i < initialSolutionSize-2; i++) {
        // std::cout << "In main for loop" << std::endl;
        auto intersectionIterator = deliveryInfos.find(pathIntersections[i]);
        if (intersectionIterator != deliveryInfos.end()) {
            auto pickUpIterator = deliveryInfos[pathIntersections[i+1]].find(pathIntersections[i]);
            if (pickUpIterator != deliveryInfos[pathIntersections[i+1]].end()) {
                continue; // Swap illegal
            }
        }
        // Swap legal
        // std::cout << "Found a legal swap, index " << i << std::endl;
        std::vector<IntersectionIdx> swappedIntersections(4);
        for (int j = 0; j < 4; j++) {
            swappedIntersections[j] = pathIntersections[i-1+j];
        }
        IntersectionIdx temp = swappedIntersections[2];
        swappedIntersections[2] = swappedIntersections[1];
        swappedIntersections[1] = temp;
        // std::cout << "Swapped Intersections set up" << std::endl;
        int currentTravelTime = 0;
        int tempTravelTime = 0;
        for (int j = 0; j < 3; j++) {
            int firstOne = intersectionVectorIndices.find(pathIntersections[i-1+j])->second;
            int secondOne = intersectionVectorIndices.find(pathIntersections[i+j])->second;
            currentTravelTime += travelTimeMatrix[firstOne][secondOne].travelTime;
        }
        // std::cout << "First time calculated" << std::endl;
        for (int j = 0; j < 3; j++) {
            // std::cout << "Loop number " << j << std::endl;
            int firstOne = intersectionVectorIndices.find(swappedIntersections[j])->second;
            int secondOne = intersectionVectorIndices.find(swappedIntersections[j+1])->second;
            // std::cout << "Indices calculated" << std::endl;
            tempTravelTime += travelTimeMatrix[firstOne][secondOne].travelTime;
        }
        // std::cout << "Travel times calculated" << std::endl;

        if (tempTravelTime < currentTravelTime) { // Swap made path faster
            // std::cout << "Found a fast swap, index " << i << std::endl;
            pathIntersections[i] = swappedIntersections[1];
            pathIntersections[i+1] = swappedIntersections[2];

            for (int j = 0; j < 3; j++) {                
                int firstIndex = intersectionVectorIndices.find(pathIntersections[i-1+j])->second;
                int secondIndex = intersectionVectorIndices.find(pathIntersections[i+j])->second;
                (*initialPath)[i-1+j].subpath = travelTimeMatrix[firstIndex][secondIndex].path;
                std::pair<IntersectionIdx, IntersectionIdx> newPair (pathIntersections[i-1+j], pathIntersections[i+j]);
                (*initialPath)[i-1+j].intersections = newPair;
            }
            return true;
        }
    }
    return false;
}

void twoOptAnneal(std::vector<CourierSubPath>* initialPath, std::unordered_map<IntersectionIdx, std::unordered_set<IntersectionIdx>> deliveryInfos, double turnPenalty, unsigned int seed, int perturbationSize, struct twoOptData* returnStruct) {
    std::vector<IntersectionIdx> pathIntersections;
    int pathIntersectionsSize = initialPath->size()+1;
    int initialSolutionSize = initialPath->size();
    pathIntersections.resize(pathIntersectionsSize);
    for (int i =  0; i < pathIntersectionsSize; i++) {
        pathIntersections[i] = (*initialPath)[i].intersections.first;
    }
    pathIntersections[pathIntersectionsSize-1] = (*initialPath)[initialSolutionSize-1].intersections.second;

    std::mt19937 generator(seed);
    std::uniform_int_distribution<int> distribution(1, pathIntersectionsSize-1);

    int randomNumber1 = distribution(generator);
    int randomNumber2 = distribution(generator);

    int firstIndex = std::min(randomNumber1, randomNumber2);
    int secondIndex = std::max(randomNumber1, randomNumber2);

    if (firstIndex == secondIndex) {
        if (secondIndex < pathIntersectionsSize-1) {
            secondIndex++;
        } else if (firstIndex > 1) {
            firstIndex--;
        }
    }
    if ((secondIndex-firstIndex) > perturbationSize) {
        secondIndex = firstIndex + perturbationSize;
    }

    int difference = secondIndex - firstIndex;

    std::vector<IntersectionIdx> clippedSection;
    clippedSection.resize(difference + 1);
    for (int i = 0; i < difference+1; i++) {
        clippedSection[i] = pathIntersections[secondIndex-i];
    }
    
    for (int i = 0; i < difference; i++) {
        IntersectionIdx suspiciousIntersection = clippedSection[i];
        auto dropOffIterator = deliveryInfos.find(suspiciousIntersection);
        if (dropOffIterator == deliveryInfos.end()) {
            continue;
        }
        int numberOfPickupChecks = difference-i;
        for (int j = 0; j < numberOfPickupChecks; j++) {
            IntersectionIdx pickupId = clippedSection[i+1+j];
            auto pickUpIterator = deliveryInfos[suspiciousIntersection].find(pickupId);
            if (pickUpIterator != deliveryInfos[suspiciousIntersection].end()) {
                returnStruct->legalPathFound = false;
                return;
            }
        }
    }

    returnStruct->legalPathFound = true;

    double currentTravelTime = 0.0;
    double tempTravelTime = 0.0;
    for (int i = 0; i < difference+2; i++) {
        int firstOne = intersectionVectorIndices.find(pathIntersections[firstIndex+i-1])->second;
        int secondOne = intersectionVectorIndices.find(pathIntersections[firstIndex+i])->second;
        currentTravelTime += travelTimeMatrix[firstOne][secondOne].travelTime;
    }
    for (int i = 0; i < difference; i++) {
        int firstOne = intersectionVectorIndices.find(clippedSection[i])->second;
        int secondOne = intersectionVectorIndices.find(clippedSection[i+1])->second;
        tempTravelTime += travelTimeMatrix[firstOne][secondOne].travelTime;
    }
    int firstOne = intersectionVectorIndices.find(pathIntersections[firstIndex-1])->second;
    int secondOne = intersectionVectorIndices.find(clippedSection[0])->second;
    int thirdOne = intersectionVectorIndices.find(clippedSection[difference])->second;
    int fourthOne = intersectionVectorIndices.find(pathIntersections[secondIndex+1])->second;

    tempTravelTime += travelTimeMatrix[firstOne][secondOne].travelTime;
    tempTravelTime += travelTimeMatrix[thirdOne][fourthOne].travelTime;

    returnStruct->newTime = tempTravelTime;
    returnStruct->timeDifference = tempTravelTime - currentTravelTime;
    returnStruct->swappedSection.resize(clippedSection.size());
    returnStruct->swappedSection = clippedSection;
    returnStruct->swapStartIndex = firstIndex-1;
}

bool checkLegal(std::unordered_map<IntersectionIdx, std::vector<IntersectionIdx>>* legalChecker, std::unordered_set<IntersectionIdx>* previousIntersections, IntersectionIdx nextIntersection) {
    auto currentIterator = legalChecker->find(nextIntersection);
    auto otherCurrentIterator = previousIntersections->find(nextIntersection);
    if (otherCurrentIterator != previousIntersections->end()) {
        return false;
    }
    if (currentIterator == legalChecker->end()) {
        return true;
    }
    for (int i = 0; i < currentIterator->second.size(); i++) {
        auto pickupIterator = previousIntersections->find(currentIterator->second[i]);
        if (pickupIterator == previousIntersections->end()) {
            return false;
        }   
    }
    return true;

}


/*
IntersectionIdx findFastest(
    IntersectionIdx currentIntersection,
    std::unordered_map<IntersectionIdx, std::vector<IntersectionIdx>>* legalChecker,
    std::unordered_set<IntersectionIdx>* previousIntersections) {

    int currentIntersectionIndex = intersectionVectorIndices.find(currentIntersection)->second;
    double fastestTravelTime = travelTimeMatrix[currentIntersectionIndex][0].travelTime;
    double tempTravelTime;
    IntersectionIdx currentFastest = travelTimeMatrix[currentIntersectionIndex][0].to;
    for (int i = 0; i < travelTimeMatrix[currentIntersectionIndex].size(); i++) {
        tempTravelTime = travelTimeMatrix[currentIntersectionIndex][i].travelTime;
        if (tempTravelTime < fastestTravelTime) {
            if (checkLegal(legalChecker, previousIntersections, travelTimeMatrix[currentIntersectionIndex][i].to)) {
                fastestTravelTime = tempTravelTime;
                currentFastest = travelTimeMatrix[currentIntersectionIndex][i].to;
            }
        }
    }
    return currentFastest;
}


std::vector<CourierSubPath> get_greedy_route(
    std::unordered_map<IntersectionIdx, std::vector<IntersectionIdx>>* legalChecker
    ) {

    std::unordered_set<IntersectionIdx> previousIntersections;
    IntersectionIdx currInterIdx;


    while (1) {

        IntersectionIdx nextInterIdx = findFastest(currInterIdx, legalChecker, &previousIntersections);

        
    }    



}
*/

double route_cost(const double turn_penalty, std::vector<CourierSubPath> route) {
    double total_cost = 0.0;
    for (int i = 0; i < route.size(); i++) {
        total_cost += computePathTravelTime(turn_penalty, route[i].subpath);
    }
    return total_cost;
}
