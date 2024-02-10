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


// Returns the distance between two (lattitude,longitude) coordinates in meters.
// Speed Requirement --> moderate
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


// Returns the angle (in radians) that would result as you exit
// src_street_segment_id and enter dst_street_segment_id, if they share an
// intersection.
// If a street segment is not completely straight, use the last piece of the
// segment closest to the shared intersection.
// If the two street segments do not share an intersection, return a constant
// NO_ANGLE, which is defined above.
// Speed Requirement --> none
double findAngleBetweenStreetSegments(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id) {

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






