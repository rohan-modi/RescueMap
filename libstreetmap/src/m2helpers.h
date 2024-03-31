#pragma once

#include <cmath>
#include <sstream>
#include <chrono>
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m3helpers.h"
#include "m2helpers.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>

ezgl::point2d latlon_to_point(LatLon position);

int findDistanceBetweenTwoPointsxy(ezgl::point2d point_1, ezgl::point2d point_2);