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
#include <chrono>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>


// ==================================== Declare Helper Functions ====================================
std::string getSegmentTravelDirection(IntersectionIdx inter1, IntersectionIdx inter2);

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

    return {0};
}


// Determines the direction of travel given 2 intersection endpoints of a street segment.
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

