#pragma once

#include "StreetsDatabaseAPI.h"

#include <vector>
#include <string>

std::string getTravelDirections(const std::vector<StreetSegmentIdx>& path, IntersectionIdx inter_start, IntersectionIdx inter_finish);