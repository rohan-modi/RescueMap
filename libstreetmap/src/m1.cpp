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
void populateStreetSegmentsOfIntersections();
void populateStreetNamesVector();
void populateOSMNodeByID();
void populateOSMWaylengths();
void populateSegmentsOfStreets();

// Declare helper functions
double getDistanceBetweenPoints(LatLon point1, LatLon point2);
inline bool streetPairComparer(const std::pair<std::string, int>& pair1, const std::pair<std::string, int>& pair2);
std::pair<double, double> latLontoCartesian(LatLon point_1, double latavg);



void populateOSMNodeByID() {
    for (int i = 0; i < getNumberOfNodes(); i++) {
        OSMNodeByID[(getNodeByIndex(i)->id())] = getNodeByIndex(i);
    }
}

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

void populateStreetNamesVector() {
    streetNamesAndIDs.resize(getNumStreets());
    for (int i = 0; i < getNumStreets(); i++) {
        std::string streetName = getStreetName(i);

        std::string::iterator endPosition = std::remove(streetName.begin(), streetName.end(), ' ');
        streetName.erase(endPosition, streetName.end());

        for (int j = 0; j < streetName.size(); j++) {
            streetName[j] = std::tolower(streetName[j]);
        }

        streetNamesAndIDs[i].first = streetName;
        streetNamesAndIDs[i].second = i;
    }
    std::sort(streetNamesAndIDs.begin(), streetNamesAndIDs.end());
}

inline bool streetPairComparer(const std::pair<std::string, int>& pair1, const std::pair<std::string, int>& pair2) {
    return (pair1.first < pair2.first || (pair1.first == pair2.first && pair1.second < pair2.second));
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {
    std::string startString = street_prefix;

    std::string::iterator endPosition = std::remove(startString.begin(), startString.end(), ' ');
    startString.erase(endPosition, startString.end());

    if (startString == "") {
        std::vector<StreetIdx> returnVector;
        returnVector.push_back(0);
        return returnVector;
    }

    for (int i = 0; i < startString.size(); i++) {
        startString[i] = std::tolower(startString[i]);
    }
    std::string endString = startString;
    endString[endString.size() - 1] = static_cast<char>(endString[endString.size() - 1] + 1);

    std::pair<std::string, int> prefixPair = {startString, 0};
    std::pair<std::string, int> endPair = {endString, 0};
    
    auto lowerBound = std::lower_bound(streetNamesAndIDs.begin(), streetNamesAndIDs.end(), prefixPair, streetPairComparer);
    auto upperBound = std::upper_bound(streetNamesAndIDs.begin(), streetNamesAndIDs.end(), endPair, streetPairComparer);

    auto pointer = lowerBound;

    std::vector<StreetIdx> returnVector;

    while(1) {
        returnVector.push_back(pointer->second);
        std::advance(pointer, 1);
        if (pointer == upperBound) {
            break;
        }
    }

    return returnVector;
}

void populateStreetSegmentsOfIntersections() {
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

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id) {
  return streetSegmentsOfIntersections[intersection_id];
}

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
    return intersectionsOfStreets_[street_id];
}

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

double getDistanceBetweenPoints(LatLon point1, LatLon point2) {
    double lat1 = point1.latitude()*kDegreeToRadian;
    double lon1 = point1.longitude()*kDegreeToRadian;

    double lat2 = point2.latitude()*kDegreeToRadian;
    double lon2 = point2.longitude()*kDegreeToRadian;

    double lat_avg = (lat1 + lat2)/2;

    double x1 = kEarthRadiusInMeters*lon1*cos(lat_avg);
    double y1 = kEarthRadiusInMeters*lat1;

    double x2 = kEarthRadiusInMeters*lon2*cos(lat_avg);
    double y2 = kEarthRadiusInMeters*lat2;

    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

    return distance;
}

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

    populateStreetSegmentsOfIntersections();
    
    populateStreetNamesVector();
    
    populateSegmentTravelTimes();
    populateOSMNodeByID();
    populateOSMWaylengths();

    load_successful = check1 && check2; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

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


double findStreetLength(StreetIdx street_id) {

    double result = 0.0;
    for(int segID : streetSegments[street_id]){
        result = result + findStreetSegmentLength(segID);
    }

    return result;
}

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

std::pair<double, double> latLontoCartesian(LatLon point_1, double latavg){ 

            double x1 = kEarthRadiusInMeters * kDegreeToRadian * point_1.longitude() * cos(latavg*kDegreeToRadian);

            double y1 = kEarthRadiusInMeters * kDegreeToRadian * point_1.latitude();

            return {x1,y1};
}

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


    return fabs(result/2);
}

double findWayLength(OSMID way_id) {
    
    return OSMWaylengths[way_id];
}

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

// Returns the travel time to drive from one end of a street segment
// to the other, in seconds, when driving at the speed limit.
// Note: (time = distance/speed_limit)
// Speed Requirement --> high
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    // Look up travel time stored in global vector segmentTravelTimes
    return segmentTravelTimes[street_segment_id];
}
