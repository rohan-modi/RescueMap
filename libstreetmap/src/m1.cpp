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
#include <unordered_set>

typedef int StreetSegmentIdx;
typedef int IntersectionIdx;
typedef int POIIdx;
typedef int StreetIdx;

std::vector<std::vector<StreetSegmentIdx>> streetSegmentsOfIntersections;
std::vector<std::vector<IntersectionIdx>> intersectionsOfStreets_;

void populateStreetSegmentsOfIntersections();

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

double getDistanceBetweenPoints(LatLon point1, LatLon point2);

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
    double minDistance = getDistanceBetweenPoints(my_position, getIntersectionPosition(0));

    for (int i = 0; i < getNumIntersections(); i++) {
        if (getDistanceBetweenPoints(my_position, getIntersectionPosition(i)) < minDistance) {
            minDistance = getDistanceBetweenPoints(my_position, getIntersectionPosition(i));
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

    //
    // Load your map related data structures here.
    //
    loadStreetsDatabaseBIN(map_streets_database_filename);

    for (int i = 0; i < streetSegmentsOfIntersections.size(); i++) {
        streetSegmentsOfIntersections[i].clear();
    }
    for (int i = 0; i < intersectionsOfStreets_.size(); i++) {
        intersectionsOfStreets_[i].clear();
    }

    populateStreetSegmentsOfIntersections();
    

    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    
}

double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2) {
    if (point_1.latitude() == point_2.latitude()) {
        return 0.0;
    }
    return 0.0;
}

<<<<<<< HEAD
=======
double findStreetSegmentLength(StreetSegmentIdx street_segment_id) {
    if (street_segment_id) {
        return 0.0;
    }
    return 0.0;
}
>>>>>>> master

double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    if (street_segment_id) {
        return 0.0;
    }
    return 0.0;
}

double findAngleBetweenStreetSegments(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id) {
    int x = src_street_segment_id;
    int y = dst_street_segment_id;
    if (x == y) {
        return 0.0;
    }
    return 0.0;
}

bool intersectionsAreDirectlyConnected(std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {
    if (intersection_ids.first == intersection_ids.second) {
        return true;
    }
    return true;
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {
    std::vector<int> returnVector;
    if (street_prefix == "hello") {
        return returnVector;
    }
    return returnVector;
}

double findStreetLength(StreetIdx street_id) {
    if (street_id == 1) {
        return 0.0;
    }
    return 0.0;
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_name) {
    if (my_position.latitude() == 1 && poi_name == "hello") {
        return 0.0;
    }
    return 0;
}

double findFeatureArea(FeatureIdx feature_id) {
    if (feature_id == 1) {
        return 0.0;
    }
    return 0.0;
}

double findWayLength(OSMID way_id) {
    std::vector<OSMID> aVector;
    aVector.push_back(way_id);
    return 0.0;
}

std::string getOSMNodeTagValue(OSMID osm_id, std::string key) {
    if (key == "hello") {
        return  "hello";
    }
    std::vector<OSMID> aVector;
    aVector.push_back(osm_id);
    return "Hello";
}
