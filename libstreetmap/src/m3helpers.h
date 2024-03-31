#pragma once

#include "StreetsDatabaseAPI.h"

#include <vector>
#include <string>

std::string getTravelDirections(const std::vector<StreetSegmentIdx>& path, IntersectionIdx inter_start, IntersectionIdx inter_finish);

std::string getSegmentTravelDirection(IntersectionIdx inter1, IntersectionIdx inter2);

std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2);

std::vector<LatLon>findAngleReferencePoints(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id);

std::string getRoundedDistance(double distance);

void replaceUnknown(std::string &input);