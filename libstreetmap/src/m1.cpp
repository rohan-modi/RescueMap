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

typedef int StreetSegmentIdx;
typedef int IntersectionIdx;
typedef int POIIdx;
typedef int StreetIdx;

std::vector<std::vector<StreetSegmentIdx>> streetSegmentsOfIntersections;


void populateStreetSegmentsOfIntersections() {
   int numberOfStreetSegments;
   int b = 6;
   int numberOfInstersections = getNumIntersections();
   int a = 5;
   streetSegmentsOfIntersections.resize(getNumIntersections());
   for (int i = 0; i < getNumIntersections(); i++) {
       numberOfStreetSegments = getNumIntersectionStreetSegment(i);
       for (int j = 0; j < numberOfStreetSegments; j++) {
           streetSegmentsOfIntersections[i].push_back(getIntersectionStreetSegment(j, i));
       }
   }
}


//std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id) {
//   return streetSegmentsOfIntersections[intersection_id];
//}



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

    populateStreetSegmentsOfIntersections();
    

    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    
}



double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    return 0.0;
}


bool intersectionsAreDirectlyConnected(std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {
    return 0.0;
}

IntersectionIdx findClosestIntersection(LatLon my_position){
    return 0;
}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
    std::vector<int> returnVector;
    return returnVector;
} 

std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    std::vector<int> returnVector;
    return returnVector;
}

std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids) {
    std::vector<int> returnVector;
    return returnVector;
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix) {
    std::vector<int> returnVector;
    return returnVector;
}

double findStreetLength(StreetIdx street_id) {
    return 0.0;
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_name) {
    return 0;
}

double findFeatureArea(FeatureIdx feature_id) {
    return 0.0;
}

double findWayLength(OSMID way_id) {
    return 0.0;
}

std::string getOSMNodeTagValue(OSMID osm_id, std::string key) {
    return "Hello";
}
