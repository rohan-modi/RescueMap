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
std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2);
std::vector<LatLon>findAngleReferencePoints(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id);

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
std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2) {
    
    std::vector<LatLon> referencePoints = findAngleReferencePoints(segment1, segment2);
    LatLon shared = referencePoints[0];
    LatLon point1 = referencePoints[1];
    LatLon point2 = referencePoints[2];
    
    float lat_avg = kDegreeToRadian * (shared.latitude() + point1.latitude() + point2.latitude()) / 3;

    // Compute x-coordinates
    float x_shared = kEarthRadiusInMeters * kDegreeToRadian * shared.longitude() * cos(lat_avg);
    float x_1 = kEarthRadiusInMeters * kDegreeToRadian * point1.longitude() * cos(lat_avg);
    float x_2 = kEarthRadiusInMeters * kDegreeToRadian * point2.longitude() * cos(lat_avg);
    
    // Compute y-coordinates for src and dest
    float y_shared = kEarthRadiusInMeters * kDegreeToRadian * shared.latitude();
    float y_1 = kEarthRadiusInMeters * kDegreeToRadian * point1.latitude();
    float y_2 = kEarthRadiusInMeters * kDegreeToRadian * point2.latitude();

    // Compute cross product in z-direction
    float crossProduct = (x_shared - x_1) * (y_2 - y_shared) - (x_2 - x_shared) * (y_shared - y_1);

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