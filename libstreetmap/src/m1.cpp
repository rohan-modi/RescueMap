/* 
 * Copyright 2024 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include <cmath>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include <unordered_set>
#include <unordered_map>

// Declare typedefs
typedef int StreetSegmentIdx;
typedef int IntersectionIdx;
typedef int POIIdx;
typedef int StreetIdx;

/*
Cities:
beijing_china
boston_usa
cape-town_south-africa
golden-horseshoe_canada
hamilton_canada
hong-kong_china
iceland
interlaken_switzerland
kyiv_ukraine
london_england
new-delhi_india
new-york_usa
rio-de-janeiro_brazil
saint-helena
singapore
sydney_australia
tehran_iran
tokyo_japan
toronto_canada
*/

// ==================================== Declare global variables ====================================
std::vector<std::vector<StreetSegmentIdx>> streetSegmentsOfIntersections;
std::vector<std::vector<IntersectionIdx>> intersectionsOfStreets_;
std::vector<std::pair<std::string, int>> streetNamesAndIDs;
std::vector<double> segmentTravelTimes;
std::vector<std::vector<int>> streetSegments;
std::unordered_map<OSMID, const OSMNode*> OSMNodeByID;
std::unordered_map<OSMID, double> OSMWaylengths;


// ==================================== Declare functions to populate globals ====================================
void populateSegmentTravelTimes();
void populateStreetSegmentsAndIntersectionsVectors();
void populateStreetNamesVector();
void populateOSMNodeByID();
void populateOSMWaylengths();
void populateSegmentsOfStreets();

// ==================================== Declare helper functions ====================================
inline bool streetPairComparer(const std::pair<std::string, int>& pair1, const std::pair<std::string, int>& pair2);
std::pair<double, double> latLontoCartesian(LatLon point_1, double latavg);


// ==================================== Implementation of m1 functions ====================================

// Returns the distance between two (lattitude,longitude) coordinates in meters.
// Speed Requirement --> moderate
// Written by Jonathan
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2) {
    // Use the formula: (x,y) = (R * lon * cos(lat_avg), R * lat)
    // where:   R = kEarthRadiusInMeters
    // and      lat, lon expressed in [rad]; multiply [degree] values by kDegreeToRadian

    // To find the distance between two points (lon1, lat1) and (lon2, lat2),
    // it is accurate to compute lat_avg = (lat1 + lat2) / 2 [rad]
    double lat_avg = kDegreeToRadian * (point_1.latitude() + point_2.latitude()) / 2;

    // Compute x-coordinates for point_1 and point_2
    double x1 = kEarthRadiusInMeters * kDegreeToRadian * point_1.longitude() * cos(lat_avg);
    double x2 = kEarthRadiusInMeters * kDegreeToRadian * point_2.longitude() * cos(lat_avg);

    // Compute y-coordinates for point_1 and point_2
    double y1 = kEarthRadiusInMeters * kDegreeToRadian * point_1.latitude();
    double y2 = kEarthRadiusInMeters * kDegreeToRadian * point_2.latitude();

    // Return the distance by the Pythagoras theorem: d = sqrt((y2 - y1)^2, (x2 - x1)^2) [m]
    return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
}

// Returns the length of the given street segment in meters.
// Speed Requirement --> moderate
// Written by Jonathan
double findStreetSegmentLength(StreetSegmentIdx street_segment_id) {
    // Get the StreetSegmentInfo struct associated with street_segment_id
    StreetSegmentInfo segment = getStreetSegmentInfo(street_segment_id);

    // If segment contains 0 curve points: it is a straight line
    if (segment.numCurvePoints == 0) {

        // Retrieve LatLon position for segment endpoints "from" and "to"
        LatLon from_latlon = getIntersectionPosition(segment.from);
        LatLon to_latlon = getIntersectionPosition(segment.to);

        // Return straight-line distance between "from" and "to"
        return findDistanceBetweenTwoPoints(from_latlon, to_latlon);
    }

    // Else, segment contains curve points
    else {
        // APPROACH: 
        // Within the segment endpoints, add the sub-segment distances between curve points

        // Initialize curvePointIdx = 0
        int curvePointIdx = 0;

        // Initialize first 2 points to calculate distance: endpoint "from" and curvePoint 0
        LatLon latlon1 = getIntersectionPosition(segment.from);
        LatLon latlon2 = getStreetSegmentCurvePoint(curvePointIdx, street_segment_id);

        // Compute the first sub-segment distance between endpoint "from" and curvePoint 0
        double distance = findDistanceBetweenTwoPoints(latlon1, latlon2);
        
        // Loop through all curve points to traverse the entire street segment
        while (curvePointIdx < segment.numCurvePoints) {
            
            // Set the next curve point 1
            latlon1 = latlon2;

            // If current curve point is not the last: then set the next curve point 2
            if (curvePointIdx + 1 != segment.numCurvePoints) {
                latlon2 = getStreetSegmentCurvePoint(curvePointIdx + 1, street_segment_id);
            }
            // Else, final point becomes the overall segment endpoint "to"
            else {
                latlon2 = getIntersectionPosition(segment.to);
            }

            // Accumulate the total distance
            distance += findDistanceBetweenTwoPoints(latlon1, latlon2);

            // Increment curvePointIdx to calculate next sub-segment distance
            ++curvePointIdx;
        }

        return distance;
    }
}

// Returns the travel time to drive from one end of a street segment
// to the other, in seconds, when driving at the speed limit.
// Note: (time = distance/speed_limit)
// Speed Requirement --> high
// Written by Jonathan
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    // Look up travel time stored in global vector segmentTravelTimes
    return segmentTravelTimes[street_segment_id];
}

// Returns the angle (in radians) that would result as you exit
// src_street_segment_id and enter dst_street_segment_id, if they share an
// intersection.
// If a street segment is not completely straight, use the last piece of the
// segment closest to the shared intersection.
// If the two street segments do not share an intersection, return a constant
// NO_ANGLE, which is defined above.
// Speed Requirement --> none
// Written by Jonathan
double findAngleBetweenStreetSegments(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id) {

    // Initialize StreetSegmentInfo struct for src and dst
    StreetSegmentInfo src_segment = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo dst_segment = getStreetSegmentInfo(dst_street_segment_id);

    // Declare 3 reference points to represent 2 connected street segments
    LatLon shared_point, point_1, point_2;

    // Determine 3 reference points depending on orientation for src and dst
    if (src_segment.from == dst_segment.from) {

        // If src_segment is one-way, then ILLEGAL -- not possible
        if (src_segment.oneWay) {
            return NO_ANGLE;
        }

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

        // If dst_segment is one-way, then ILLEGAL -- not possible
        if (dst_segment.oneWay) {
            return NO_ANGLE;
        }

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
        
        // If src_segment OR dst_segment is one-way, then ILLEGAL -- not possible
        if (src_segment.oneWay || dst_segment.oneWay) {
            return NO_ANGLE;
        }

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
        
        // No need to check for one-way streets -- both segments are forward direction
        
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

    else {
        // The src and dst street segments do not share an intersection
        return NO_ANGLE;
    }

    // COMPUTE THE ANGLE //
    
    // Determine the triangle side lengths between 3 reference points
    double srclength = findDistanceBetweenTwoPoints(shared_point, point_1);
    double dstlength = findDistanceBetweenTwoPoints(shared_point, point_2);
    double side3 = findDistanceBetweenTwoPoints(point_1, point_2);

    // Solve using the cosine law: side3^2 = srclength^2 + dstlength^2 - 2 * srclength * dstlength * cos(pi - angle)
    double angle = M_PI - acos((pow(side3, 2) - pow(srclength, 2) - pow(dstlength, 2)) / (-2 * srclength * dstlength));

    return angle;
}

// Returns true if the two intersections are directly connected, meaning you can
// legally drive from the first intersection to the second using only one
// streetSegment.
// Speed Requirement --> moderate
// Written by Jonathan
bool intersectionsAreDirectlyConnected(std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {

    // Get the number of street segments attached to IntersectionIdx 1
    int point1_numSegments = getNumIntersectionStreetSegment(intersection_ids.first);

    // Loop to see if any segments connect to IntersectionIdx 2
    for (int segmentNum = 0; segmentNum < point1_numSegments; ++segmentNum) {
        
        // Get StreetSegmentIdx for the segmentNum'th segment attached to IntersectionIdx 1
        StreetSegmentIdx segmentID_1 = getIntersectionStreetSegment(segmentNum, intersection_ids.first);
        
        // Get StreetSegmentInfo struct for segmentID 1
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentID_1);

        // If segment "to" matches IntersectionIdx 2,
        // then the intersections are directly connected (regardless of one-way status)
        if (intersection_ids.second == segmentInfo.to) {
            return true;
        }
        // If segment "from" matches IntersectionIdx 2 && the street is NOT one-way,
        // then the intersections are directly connected
        else if ((intersection_ids.second == segmentInfo.from) && (!segmentInfo.oneWay)) {
            return true;
        }
    }

    // The intersections are not directly connected; IntersectionIdx 2 not found
    return false;
}

// Loops through all intersections and calls findDistanceBetweenTwoPoints to check its distance
// Tracks the closest intersection by IntersectionIdx and tracks the smallest distance
// Writen by Rohan
IntersectionIdx findClosestIntersection(LatLon my_position) {
    IntersectionIdx closestIntersection = 0;
    double minDistance = findDistanceBetweenTwoPoints(my_position, getIntersectionPosition(0));

    for (int i = 0; i < getNumIntersections(); i++) {
        if (findDistanceBetweenTwoPoints(my_position, getIntersectionPosition(i)) < minDistance) {
            minDistance = findDistanceBetweenTwoPoints(my_position, getIntersectionPosition(i));
            closestIntersection = i;
        }
    }
    return closestIntersection;
}

// Accesses the index of the global variable that stores all street segments attached to any given intersection
// Writen by Rohan
std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id) {
    return streetSegmentsOfIntersections[intersection_id];
}

// Accesses the index of the global variable that stores all the intersections along a street
// Written by Rohan
std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
    return intersectionsOfStreets_[street_id];
}

// Initializes an unordered set as a copy of one of the vectors that contain all the intersections along a street
// Loops through the second vector and searches in the set for each intersection in the vector, pushing it to a return vector if it is found
// Written by Rohan
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids) {
    std::vector<IntersectionIdx> intersectionsOfStreet1 = findIntersectionsOfStreet(street_ids.first);
    std::vector<IntersectionIdx> intersectionsOfStreet2 = findIntersectionsOfStreet(street_ids.second);

    std::unordered_set<IntersectionIdx> vec1AsSet(intersectionsOfStreet1.begin(), intersectionsOfStreet1.end());

    std::vector<IntersectionIdx> returnVector;

    for (int i = 0; i < intersectionsOfStreet2.size(); i++) {
        if (vec1AsSet.find(intersectionsOfStreet2[i]) != vec1AsSet.end()) {
            if (returnVector.size() == 0) {
                returnVector.push_back(intersectionsOfStreet2[i]);
            } else if (returnVector.back() != intersectionsOfStreet2[i]) {
                returnVector.push_back(intersectionsOfStreet2[i]);
            }
        }
    }
    return returnVector;
}

// Process string to remove spaces, and convert to lowercase
// Then return vector containing 0 if the processed string is empty
// Create a string that is 1 value higher than the input string by incrementing the ASCII value of the last character of the input string
// Use this string as an upper bound in a binary search and use the input string as the lower bound
// Loop until the iterator of the lower bound points to the higher bound, and push all associated street ids to the return vector
// The street id associated with any given street name is stored with the name in a global variable
// The global variable is a vector of pairs, a custom comparator helper function was written for the binary search, implemented below (streetPairComparer)
// Written by Rohan
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {
    std::cout << "Function entered" << std::endl;
    std::string startString = street_prefix;

    std::string::iterator endPosition = std::remove(startString.begin(), startString.end(), ' ');
    startString.erase(endPosition, startString.end());

    if (startString == "") {
        std::vector<StreetIdx> returnVector;
        return returnVector;
    }

    for (int i = 0; i < startString.size(); i++) {
        startString[i] = std::tolower(startString[i]);
    }
    std::string endString = startString;
    endString[endString.size() - 1] = static_cast<unsigned char>(endString[endString.size() - 1] + 1);

    std::pair<std::string, int> prefixPair = {startString, 0};
    std::pair<std::string, int> endPair = {endString, 0};
    
    auto lowerBound = std::lower_bound(streetNamesAndIDs.begin(), streetNamesAndIDs.end(), prefixPair, streetPairComparer);
    auto upperBound = std::upper_bound(streetNamesAndIDs.begin(), streetNamesAndIDs.end(), endPair, streetPairComparer);

    std::vector<StreetIdx> returnVector;

    if (lowerBound != streetNamesAndIDs.end()) {
        for (int i = 0; i < startString.size(); i++) {
            if (((*lowerBound).first)[i] != startString[i]) {
                return returnVector;
            }
        }
    }

    auto pointer = lowerBound;

    while(1) {
        returnVector.push_back(pointer->second);
        std::advance(pointer, 1);
        if (pointer == upperBound) {
            break;
        }
    }

    std::cout << "Function exiting" << std::endl;
    return returnVector;
}

// Written by Kevin
// 2D vector used in load maps to associate street segments with respective street ID
// Street IDs are matched with rows of 2D vector and all entries are summed for street length
double findStreetLength(StreetIdx street_id) {

    double result = 0.0;
    for(int segID : streetSegments[street_id]){
        result = result + findStreetSegmentLength(segID);
    }

    return result;
}

// Written by Kevin
// Loops through all POI's looking for names matching input poi_name.
// running minimum distance used to determine closest poi location
POIIdx findClosestPOI(LatLon my_position, std::string poi_name) {

    int max = getNumPointsOfInterest();
    int current = 0;
    bool foundFirst = false;

    while(!foundFirst){
        if(poi_name == getPOIName(current)){
            foundFirst = true;
        }
        current++;
    }

    POIIdx result = current-1;

    double leastDistance  = findDistanceBetweenTwoPoints(my_position, getPOIPosition(result));
    double tempDistance;

    for(int i = current; i < max; i++){
        if(poi_name == getPOIName(i)){
            tempDistance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(i));
            if(tempDistance < leastDistance){
                result = i;
                leastDistance = tempDistance;
            }
        }
    }
    return result;
}

// Written by Kevin
// Utilizes Shoelace method for polygon area calculation
// depends on latLontoCartesian to convert LatLon coords to XY coordinates
double findFeatureArea(FeatureIdx feature_id) {

    double result = 0.0;
    int points = getNumFeaturePoints(feature_id);

    int first = 0; 
    int second = 1;

    if(points > 1 && getFeaturePoint(0, feature_id) == getFeaturePoint(points-1, feature_id)){

        double latavg = 0.0;

        for(int i = 0; i < points; i++){
            latavg += getFeaturePoint(i, feature_id).latitude();
        }

        latavg = latavg/points;

        while(second != points){

            std::pair<double, double> point_1 = latLontoCartesian(getFeaturePoint(first, feature_id), latavg);
            std::pair<double, double> point_2 = latLontoCartesian(getFeaturePoint(second, feature_id), latavg);

            result += (point_1.first * point_2.second);
            result -= (point_1.second * point_2.first);

            first++;
            second++;
        }

    }

    return std::abs(result/2);
}

// Written by Kevin
// Way lengths are calculated in load map function to meet function speed requirements
// Unordered Map used to pair OSMIDs to calculated way Lengths
// lengths calculated in load maps to increase function speed
double findWayLength(OSMID way_id) {
    return OSMWaylengths[way_id];
}

// Find the OSMNode associated with the input OSMID by searching in the global variable
// Return an empty string if it was not in the global unordered set
// If the OSMID was found, get the associated node and loop through its tags
// If the key in the tag matches the key parameter passed in, return the associated value
// Return an empty string if all tags are looped through and the OSMNode does not contain the input key
// Written by Rohan
std::string getOSMNodeTagValue(OSMID osm_id, std::string key) {
    const OSMNode* node;
    std::pair<std::string, std::string> tagPair;

    auto iterator = OSMNodeByID.find(osm_id);

    if (iterator == OSMNodeByID.end()) {
        return "";
    }

    node = iterator->second;

    for (int j = 0; j < getTagCount(node); j++) {
        tagPair = getTagPair(node, j);
        if (tagPair.first == key) {
            return tagPair.second;
        }
    }
    return "";
}



// ==================================== LoadMap and closeMap ====================================


// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection 
// data that is higher-level than the raw OSM data). 
// This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.

// Call functions to preload gloabl variables, clearing them first to ensure there is no extra data
// Open both databases and check if they were both successful
// Written by all members of group
bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    if (map_streets_database_filename.find("streets") == std::string::npos) {
        return false;
    }

    //
    // Load your map related data structures here.
    //
    std::string mapName = map_streets_database_filename;

    bool check1 = loadStreetsDatabaseBIN(mapName);  

    std::string osmMapName = mapName.replace(mapName.find(".streets"), 8, ".osm");

    bool check2 = loadOSMDatabaseBIN(osmMapName);
    populateSegmentsOfStreets();

    for (int i = 0; i < streetSegmentsOfIntersections.size(); i++) {
        streetSegmentsOfIntersections[i].clear();
    }
    for (int i = 0; i < intersectionsOfStreets_.size(); i++) {
        intersectionsOfStreets_[i].clear();
    }
    streetNamesAndIDs.clear();
    segmentTravelTimes.clear();
    OSMNodeByID.clear();

    populateStreetSegmentsAndIntersectionsVectors();
    
    populateStreetNamesVector();
    
    populateSegmentTravelTimes();
    populateOSMNodeByID();
    populateOSMWaylengths();

    load_successful = check1 && check2; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

// Clear all global variables and close the two databases
// Written by all members of group
void closeMap() {
    //Clean-up your map related data structures here
    for (int i = 0; i < streetSegmentsOfIntersections.size(); i++) {
        streetSegmentsOfIntersections[i].clear();
    }
    for (int i = 0; i < intersectionsOfStreets_.size(); i++) {
        intersectionsOfStreets_[i].clear();
    }

    for (int i = 0; i < streetSegments.size(); i++) {
        streetSegments[i].clear();
    }

    streetSegments.clear();
    streetNamesAndIDs.clear();
    segmentTravelTimes.clear();

    closeOSMDatabase();
    closeStreetDatabase();
}

// ==================================== Implementation of helper functions to populate global variables ====================================

// Loop up to the number of nodes and set the key value pairs of the global unordered set
// Set the key to the OSMID of OSMNode with index i
// Set the value to the OSMNode with index i
// Enables you to search by OSMID and find an OSMNode with a given OSMID
// Written by Rohan
void populateOSMNodeByID() {
    for (int i = 0; i < getNumberOfNodes(); i++) {
        OSMNodeByID[(getNodeByIndex(i)->id())] = getNodeByIndex(i);
    }
}

// Written by Jonathan
void populateSegmentTravelTimes() {
    // Initialize the vector size to the number of street segments
    segmentTravelTimes.resize(getNumStreetSegments());

    // Compute the travel time for each street segment and populate the vector
    for (StreetSegmentIdx idx = 0; idx < getNumStreetSegments(); ++idx) {
        
        // Get the StreetSegmentInfo struct associated with street_segment_id
        StreetSegmentInfo segment = getStreetSegmentInfo(idx);

        // Compute time [s] = distance [m] / speed_limit [m/s]
        segmentTravelTimes[idx] = (findStreetSegmentLength(idx) / segment.speedLimit);
    }
}

// Written by Kevin
// loops through all street segments and pushes to vector of respective streetID associating the two together
void populateSegmentsOfStreets() {
    for(int i = 0; i < getNumStreets(); i++){
        std::vector<int> row;
        streetSegments.push_back(row);
    }

    int max = getNumStreetSegments();
    for(int i = 0; i< max; i++){
        streetSegments[getStreetSegmentInfo(i).streetID].push_back(i);
    }
}

// Written by Kevin
// calculates way distances through summing distance of way nodes
// Unordered map used to access length data by OSMID
void populateOSMWaylengths() {
    for (int i = 0; i < getNumberOfWays(); i++) {

        std::vector<OSMID> nodesID = getWayMembers(getWayByIndex(i));

        double sum = 0.0;

        for(int j = 1; j < nodesID.size(); j++){
            sum += findDistanceBetweenTwoPoints(getNodeCoords(OSMNodeByID[nodesID[j-1]]),getNodeCoords(OSMNodeByID[nodesID[j]]));
        }

        OSMWaylengths[(getWayByIndex(i)->id())] = sum;
    }
}

// Process the street names by removing spaces and converting to lowercase
// Put the name of the street as the first value in the pair and the ith value (street id) as the second
// Sort the vector using the name (because it's the first value) so it can be used for a binary search
// Written by Rohan
void populateStreetNamesVector() {
    streetNamesAndIDs.resize(getNumStreets());
    // std::vector<int> ids = {180, 193, 763, 20800, 22256, 665, 1798, 10494, 13672, 3251, 6730, 19168, 3160, 310, 2579, 6428, 20611, 225, 10438, 20803, 21567, 21008};
    for (int i = 0; i < getNumStreets(); i++) {
        std::string streetName = getStreetName(i);

        std::string::iterator endPosition = std::remove(streetName.begin(), streetName.end(), ' ');
        streetName.erase(endPosition, streetName.end());

        for (int j = 0; j < streetName.size(); j++) {
            streetName[j] = std::tolower(streetName[j]);
        }

        streetNamesAndIDs[i].first = streetName;
        streetNamesAndIDs[i].second = i;
        // for (int k = 0; k < ids.size(); k++) {
        //     if (i == ids[k]) {
        //         std::cout << "Street: " << streetName << " ID: " << i << std::endl;
        //     }
        // }
    }
    std::sort(streetNamesAndIDs.begin(), streetNamesAndIDs.end());
}

// Comparator for pair, checks if first value is lower then checks if first values are equal and second is lower
// Used for binary searches lower_bound and upper_bound
// Written by Rohan
inline bool streetPairComparer(const std::pair<std::string, int>& pair1, const std::pair<std::string, int>& pair2) {
    return (pair1.first < pair2.first || (pair1.first == pair2.first && pair1.second < pair2.second));
}

// Resizes two global vectors, first to store all street segments attached to an intersection, second to store all intersections along a street
// Loops through intersections and adds all street segments tangent to it to the corresponding vector
// Then adds that intersection id to the indices of the second vector to populate both simultaneously
// Written by Rohan
void populateStreetSegmentsAndIntersectionsVectors() {
    int numberOfStreetSegments;
    StreetSegmentIdx streetSegmentID;
    StreetIdx streetID;
    streetSegmentsOfIntersections.resize(getNumIntersections());
    intersectionsOfStreets_.resize(getNumStreets());
    for (int i = 0; i < getNumIntersections(); i++) {
        numberOfStreetSegments = getNumIntersectionStreetSegment(i);
        for (int j = 0; j < numberOfStreetSegments; j++) {
            streetSegmentID = getIntersectionStreetSegment(j, i);
            streetID = (getStreetSegmentInfo(streetSegmentID)).streetID;
            streetSegmentsOfIntersections[i].push_back(streetSegmentID);
            if (intersectionsOfStreets_[streetID].size() == 0) {
                intersectionsOfStreets_[streetID].push_back(i);
            } else if (intersectionsOfStreets_[streetID].back() != i) {
                intersectionsOfStreets_[streetID].push_back(i);
            }
        }
    }
}

// Converts LatLon to cartesian coordinates
// Written by Kevin and Jonathan
std::pair<double, double> latLontoCartesian(LatLon point_1, double latavg){ 

    double x1 = kEarthRadiusInMeters * kDegreeToRadian * point_1.longitude() * cos(latavg*kDegreeToRadian);
    double y1 = kEarthRadiusInMeters * kDegreeToRadian * point_1.latitude();

    return {x1,y1};
}

