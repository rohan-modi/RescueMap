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
#include <list>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>

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
   double bestTime = 0;
};

struct WaveElem {
    IntersectionIdx nodeID;
    StreetSegmentIdx edgeID;
    IntersectionIdx reachingNodeID;
    double travelTime;
    int primaryStreet;
    WaveElem(int intersection, int segment, int inter2, float time, int s) {nodeID = intersection; edgeID = segment; reachingNodeID = inter2; travelTime = time;primaryStreet = s;}
    bool operator<(const WaveElem other) const {
        return travelTime > other.travelTime;
    }
};


// ==================================== Declare Globals ====================================
extern std::vector<std::vector<connected_intersection_data>> connectedIntersections;
extern std::vector<Intersection_data> intersections;
extern float cos_latavg;




// ==================================== Declare Helper Functions ====================================
std::string getTravelDirections(const std::vector<StreetSegmentIdx>& path, IntersectionIdx inter_start, IntersectionIdx inter_finish);
std::string getSegmentTravelDirection(IntersectionIdx inter1, IntersectionIdx inter2);
std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2);
std::vector<LatLon>findAngleReferencePoints(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id);
std::string getRoundedDistance(double distance);
void replaceUnknown(std::string &input);
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
    std::cout<<travelTime<<std::endl;
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

    openHeap.push(WaveElem(intersect_ids.first, NO_EDGE, NO_EDGE, STARTING_TIME, NO_EDGE));
    nodesToReset.push_back(intersect_ids.first);

    ezgl::point2d destination= latlon_to_pointm3(getIntersectionPosition(intersect_ids.second));


    while(!openHeap.empty()){
        WaveElem node = openHeap.top();
        openHeap.pop();

        IntersectionIdx nodeId = node.nodeID;

        if( node.travelTime < intersections[nodeId].bestTime){
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

            if(intersections[newData.intersectionId].processed){
                continue;
            }
            nodesToReset.push_back(newData.intersectionId);
            double delay;

            if(newData.primaryStreet != node.primaryStreet){
                //std::cout<< "added delay" <<std::endl;
                delay = turn_penalty;
            }else{
                //std::cout<< "NOI delay" <<std::endl;
                delay = 0.0;
            }

            
            double time = newData.travel_time + node.travelTime + findDistanceBetweenTwoPointsxym3(newData.position, destination) + delay;
            intersections[newData.intersectionId].processed = true;
            intersections[newData.intersectionId].reachingEdge = newData.streetId;
            intersections[newData.intersectionId].reachingNode = nodeId;
            intersections[nodeId].bestTime = time;

            openHeap.push(WaveElem(newData.intersectionId, newData.streetId, nodeId,time, newData.primaryStreet));

        }
    }
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

    std::list<StreetSegmentIdx> path;
    int index = nodeId;
    while(index != startingNode &&index != 0){
        path.push_front(intersections[index].reachingEdge);
        index = intersections[index].reachingNode;
    }
    return std::vector<StreetSegmentIdx>(path.begin(), path.end());
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

// This function returns a string containing detailed travel directions for a driver.
// This function calls several helper functions that return strings.
// Instructions are of the following general forms:
// -- Start at starting intersection
// -- Turn onto a new street, continue in a direction for a given distance
// -- Arrive at destination intersection
// Input receives a vector of street segments to form a path, starting intersection,
// destination intersection. Assume a valid path and valid intersection endpoints.
// Written by Jonathan
std::string getTravelDirections(const std::vector<StreetSegmentIdx>& path, IntersectionIdx inter_start, IntersectionIdx inter_finish) {

    // Declare stringstream to contain all travel directions
    std::stringstream directions;
    
    // Starting intersection
    directions << "Start at " << getIntersectionName(inter_start) << "\n";

    // Declare previous street segment and current street segment
    StreetSegmentInfo prevSegment = getStreetSegmentInfo(path[0]);
    StreetSegmentInfo currSegment = getStreetSegmentInfo(path[0]);

    // Declare previous intersection and next intersection (relative to current street segment)
    IntersectionIdx prevInter = inter_start;
    IntersectionIdx nextInter;
    if (currSegment.from == inter_start) {
        nextInter = currSegment.to;
        directions << "Head " << getSegmentTravelDirection(currSegment.from, currSegment.to);
    } else {
        nextInter = currSegment.from;
        directions << "Head " << getSegmentTravelDirection(currSegment.to, currSegment.from);
    }
    directions << " on " << getStreetName(currSegment.streetID) << "\n";
    
    // Declare and initialize total trip distance
    double totalDistance = findStreetSegmentLength(path[0]);

    // Loop through all street segments in the given path
    for (int pathIdx = 1; pathIdx < path.size(); pathIdx++) {
        
        prevSegment = currSegment;
        currSegment = getStreetSegmentInfo(path[pathIdx]);

        prevInter = nextInter;
        if (currSegment.from == prevInter) {
            nextInter = currSegment.to;
        } else {
            nextInter = currSegment.from;
        }

        // Declare distance variable to record distance travelled along one street
        // Initialize to the distance of the first street segment
        double distance = findStreetSegmentLength(path[pathIdx]);
        
        // Determine turning action based on angle between street segments
        double angle = findAngleBetweenStreetSegments(path[pathIdx - 1], path[pathIdx]);

        // Continue straight if angle is less than 8 degrees
        if (angle < 8 * kDegreeToRadian) {
            directions << "Continue straight on ";
        }
        // Slight turn if angle is between 8 degrees and 70 degrees
        else if ((angle > 8 * kDegreeToRadian) && (angle < 70 * kDegreeToRadian)) {
            directions << "Take a slight " << getIntersectionTurningDirection(path[pathIdx - 1], path[pathIdx]) << " onto ";
        }
        // Sharp turn if angle is greater than 110 degrees
        else if (angle > 110 * kDegreeToRadian) {
            directions << "Take a sharp " << getIntersectionTurningDirection(path[pathIdx - 1], path[pathIdx]) << " onto ";
        }
        // Or else, apply a regular turn
        else {
            directions << "Turn " << getIntersectionTurningDirection(path[pathIdx - 1], path[pathIdx]) << " onto ";
        }
        directions << getStreetName(currSegment.streetID) << ", ";
        directions << "continue " << getSegmentTravelDirection(prevInter, nextInter) << " for ";

        // Look ahead to the next street segment to determine its streetID.
        // While it is the same streetID, keep skipping the next iteration of street segment.
        // In other words, if consecutive street segments are part of the same street, only
        // output 1 instruction for the travel directions.
        while ((pathIdx + 1 < path.size()) && (getStreetSegmentInfo(path[pathIdx + 1]).streetID == currSegment.streetID)) {
            pathIdx = pathIdx + 1;

            prevSegment = currSegment;
            currSegment = getStreetSegmentInfo(path[pathIdx]);

            prevInter = nextInter;
            if (currSegment.from == prevInter) {
                nextInter = currSegment.to;
            } else {
                nextInter = currSegment.from;
            }

            // Accumulate the total distance across all street segments
            distance += findStreetSegmentLength(path[pathIdx]);
        }
        
        // Output the distance travelled along one street
        directions << getRoundedDistance(distance) << "\n";
        
        // Accumulate the total distance for the trip
        totalDistance += distance;
    }    

    // Destination intersection
    directions << "Arrive at " << getIntersectionName(inter_finish) << "\n";

    // Report total trip distance
    directions << "Total trip distance: " << getRoundedDistance(totalDistance) << "\n";
    
    // Report total trip time
    directions << "Total trip time: " << "<XXX[PLACEHOLDER]XXX>" << "\n";

    // Replace <unknown> and return travel directions as a string
    std::string output = directions.str();
    replaceUnknown(output);
    return output;
}

// Determines the direction of travel given 2 intersection endpoints of a street segment.
// Returns north, south, east, or west.
// Direction of travel is assumed to be inter1 -> inter2.
// Written by Jonathan
std::string getSegmentTravelDirection(IntersectionIdx inter1, IntersectionIdx inter2) {
    LatLon src = getIntersectionPosition(inter1);
    LatLon dest = getIntersectionPosition(inter2);

    // To find the distance between two points (lon1, lat1) and (lon2, lat2),
    // it is accurate to compute lat_avg = (lat1 + lat2) / 2 [rad]
    double lat_avg = kDegreeToRadian * (src.latitude() + dest.latitude()) / 2;

    // Compute x-coordinates for src and dest
    double x1 = kEarthRadiusInMeters * kDegreeToRadian * src.longitude() * cos(lat_avg);
    double x2 = kEarthRadiusInMeters * kDegreeToRadian * dest.longitude() * cos(lat_avg);

    // Compute y-coordinates for src and dest
    double y1 = kEarthRadiusInMeters * kDegreeToRadian * src.latitude();
    double y2 = kEarthRadiusInMeters * kDegreeToRadian * dest.latitude();
    
    // Compute inverse tangent to determine angle
    double angle = atan((y2 - y1) / (x2 - x1));

    // If x-component is negative, add pi
    if ((x2 - x1) < 0) {
        angle += M_PI;
    }

    // Declare direction string
    std::string direction;

    // Determine direction (North, South, East, West)
    // Direction boundaries are split at 90 degree intervals along 45 degree diagonals
    if ((angle > -(kDegreeToRadian * 45)) && (angle < kDegreeToRadian * 45)) {
        direction = "east";
    } else if ((angle > kDegreeToRadian * 45) && (angle < kDegreeToRadian * 135)) {
        direction = "north";
    } else if ((angle > kDegreeToRadian * 135) && (angle < kDegreeToRadian * 225)) {
        direction = "west";
    } else {
        direction = "south";
    }

    return direction;
}


// Determines the direction of turn given 2 street segments directly connected at
// an intersection. Returns left or right.
// This function assumes that segment1 and segment2 are connected such that one
// can drive legally by exiting segment1 and entering segment2.
// Written by Jonathan
// Determines the direction of turn given 2 street segments directly connected at
// an intersection. Returns left or right.
// This function assumes that segment1 and segment2 are connected such that one
// can drive legally by exiting segment1 and entering segment2.
// Written by Jonathan
std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2) {
    
    std::vector<LatLon> referencePoints = findAngleReferencePoints(segment1, segment2);
    LatLon shared = referencePoints[0];
    LatLon point1 = referencePoints[1];
    LatLon point2 = referencePoints[2];
    
    double lat_avg1 = kDegreeToRadian * (shared.latitude() + point1.latitude()) / 2;
    double lat_avg2 = kDegreeToRadian * (shared.latitude() + point2.latitude()) / 2;

    // Compute x-coordinates
    double x_shared1 = kEarthRadiusInMeters * kDegreeToRadian * shared.longitude() * cos(lat_avg1);
    double x_1 = kEarthRadiusInMeters * kDegreeToRadian * point1.longitude() * cos(lat_avg1);

    double x_shared2 = kEarthRadiusInMeters * kDegreeToRadian * shared.longitude() * cos(lat_avg2);
    double x_2 = kEarthRadiusInMeters * kDegreeToRadian * point2.longitude() * cos(lat_avg2);

    // Compute y-coordinates for src and dest
    double y_shared = kEarthRadiusInMeters * kDegreeToRadian * shared.latitude();
    double y_1 = kEarthRadiusInMeters * kDegreeToRadian * point1.latitude();
    double y_2 = kEarthRadiusInMeters * kDegreeToRadian * point2.latitude();
    
    // Compute cross product in z-direction: (Ax * By) - (Ay * Bx)
    double Ax = x_shared1 - x_1;
    double Ay = y_shared - y_1;

    double Bx = x_2 - x_shared2;
    double By = y_2 - y_shared;

    double crossProduct = (Ax * By) - (Ay * Bx);
    
    // By the right-hand rule:
    // If the z-component is positive, then the direction is left
    // If the z-component is negative, then the direction is right
    std::string direction;

    if (crossProduct < 0) {
        direction = "right";
    } else {
        direction = "left";
    }

    return direction;
}


// Determines the 3 sets of reference point coordinates used to determine the
// intersection turning direction.
// Returns the 3 sets of reference points as a vector of 3 LatLon objects.
// This function assumes that src and dst are connected such that one
// can drive legally by exiting src and entering dst.
// Written by Jonathan
std::vector<LatLon>findAngleReferencePoints(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id) {

    // Initialize StreetSegmentInfo struct for src and dst
    StreetSegmentInfo src_segment = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo dst_segment = getStreetSegmentInfo(dst_street_segment_id);

    // Declare 3 reference points to represent 2 connected street segments
    LatLon shared_point, point_1, point_2;

    // Determine 3 reference points depending on orientation for src and dst
    if (src_segment.from == dst_segment.from) {

        // The shared intersection LatLon is src_segment.from    
        shared_point = getIntersectionPosition(src_segment.from);

        // If the src segment has 0 curve points, then ref point 1 is src_segment.to
        if (src_segment.numCurvePoints == 0) {
            point_1 = getIntersectionPosition(src_segment.to);
        }
        // Else, ref point 1 is first curve point
        else {
            point_1 = getStreetSegmentCurvePoint(0, src_street_segment_id);
        }

        // If the dst segment has 0 curve points, then the ref point 2 is dst_segment.to
        if (dst_segment.numCurvePoints == 0) {
            point_2 = getIntersectionPosition(dst_segment.to);
        }
        // Else, the ref point 2 is first curve point
        else {
            point_2 = getStreetSegmentCurvePoint(0, dst_street_segment_id);
        }
    }

    else if (src_segment.to == dst_segment.to) {

        // The shared intersection LatLon is src_segment.to
        shared_point = getIntersectionPosition(src_segment.to);

        // If the src segment has 0 curve points, then ref point 1 is src_segment.from
        if (src_segment.numCurvePoints == 0) {
            point_1 = getIntersectionPosition(src_segment.from);
        }
        // Else, ref point 1 is last curve point
        else { 
            point_1 = getStreetSegmentCurvePoint((src_segment.numCurvePoints - 1), src_street_segment_id);
        }

        // If the dst segment has 0 curve points, then the ref point 2 is dst_segment.from
        if (dst_segment.numCurvePoints == 0) {
            point_2 = getIntersectionPosition(dst_segment.from);
        }
        // Else, the ref point 2 is last curve point
        else {
            point_2 = getStreetSegmentCurvePoint((dst_segment.numCurvePoints - 1), dst_street_segment_id);
        }
    }

    else if (src_segment.from == dst_segment.to) {

        // The shared intersection LatLon is src_segment.from    
        shared_point = getIntersectionPosition(src_segment.from);

        // If the src segment has 0 curve points, then ref point 1 is src_segment.to
        if (src_segment.numCurvePoints == 0) {
            point_1 = getIntersectionPosition(src_segment.to);
        }
        // Else, ref point 1 is first curve point
        else { 
            point_1 = getStreetSegmentCurvePoint(0, src_street_segment_id);
        }

        // If the dst segment has 0 curve points, then the ref point 2 is dst_segment.from
        if (dst_segment.numCurvePoints == 0) {
            point_2 = getIntersectionPosition(dst_segment.from);
        }
        // Else, the ref point 2 is last curve point
        else {
            point_2 = getStreetSegmentCurvePoint((dst_segment.numCurvePoints - 1), dst_street_segment_id);
        }
    }

    else if (src_segment.to == dst_segment.from) {
               
        // The shared intersection LatLon is src_segment.to    
        shared_point = getIntersectionPosition(src_segment.to);

        // If the src segment has 0 curve points, then ref point 1 is src_segment.from
        if (src_segment.numCurvePoints == 0) {
            point_1 = getIntersectionPosition(src_segment.from);
        }
        // Else, ref point 1 is last curve point
        else { 
            point_1 = getStreetSegmentCurvePoint((src_segment.numCurvePoints - 1), src_street_segment_id);
        }

        // If the dst segment has 0 curve points, then the ref point 2 is dst_segment.to
        if (dst_segment.numCurvePoints == 0) {
            point_2 = getIntersectionPosition(dst_segment.to);
        }
        // Else, the ref point 2 is first curve point
        else {
            point_2 = getStreetSegmentCurvePoint(0, dst_street_segment_id);
        }
    }

    return {shared_point, point_1, point_2};
}


// Distance is given in [m]
// Round the given distance depending on magnitude, return in string form
// Written by Jonathan
std::string getRoundedDistance(double distance) {
    const int KILOMETER_TO_METER = 1000;

    std::stringstream roundedDistance;

    // Any value below 10 m is rounded to 10 m
    if (distance < 0.01 * KILOMETER_TO_METER) {
        distance = 10;
        roundedDistance << distance << " m";        
    }
    // Any value less than 1 km is rounded to the nearest 10 m
    else if (distance < KILOMETER_TO_METER) {
        distance = std::round(distance / 10) * 10;
        roundedDistance << distance << " m";
    }
    // Any value between 1 km and 100 km is rounded to the nearest 0.1 km
    else if (distance < 100 * KILOMETER_TO_METER) {
        distance = std::round(distance / KILOMETER_TO_METER * 10) / 10;
        roundedDistance << distance << " km";
    }
    // Any value greater than 1 km is rounded to the nearest 1 km
    else {
        distance = std::round(distance / KILOMETER_TO_METER);
        roundedDistance << distance << " km";
    }
    
    // Return the rounded distance as a string
    return roundedDistance.str();
}


// Takes in a string input passed by reference.
// Replaces all instances of "<unknown>" with "Unnamed Road" for usability considerations.
// Written by Jonathan
void replaceUnknown(std::string &input) {
    
    // Find substring
    std::string str_find = "<unknown>";

    // Replacement substring
    std::string str_replace = "Unnamed Road";

    // Search for the first instance of "<unknown>"
    std::size_t pos = input.find(str_find);

    // Traverse the entire input string and replace all instances
    while (pos != std::string::npos) {
        
        // Replace first instance of "<unknown>" with "Unnamed Road"
        input.replace(pos, str_find.size(), str_replace);

        // Search for the next instance of "<unknown>"
        pos = input.find(str_find, pos + str_replace.size());
    }
}
