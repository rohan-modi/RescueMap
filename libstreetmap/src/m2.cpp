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

#include <cmath>
#include <chrono>
#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include <thread>

// Declare global variables
struct Intersection_data {
   LatLon position;
   std::string name;
};
std::vector<Intersection_data> intersections;
extern std::vector<std::vector<int>> streetSegments;

struct Map_bounds {
   double max_lat;
   double min_lat;
   double max_lon;
   double min_lon;
} mapBounds;

float cos_latavg;


// Declare helper functions
void draw_main_canvas (ezgl::renderer *g);
void initializeIntersections();
void draw_intersections(ezgl::renderer *g);
void draw_streets(ezgl::renderer *g);
void draw_POI(ezgl::renderer *g);
ezgl::point2d latlon_to_point(LatLon position);
void draw_features(ezgl::renderer *g);
void set_feature_color(ezgl::renderer *g, int feature_id);

float x_from_lon(float lon);
float y_from_lat(float lat);

float lon_from_x(float x);
float lat_from_y(float y);


// void drawMap()
void drawMap() {
   initializeIntersections();
   // Set up the ezgl graphics window and hand control to it, as shown in the 
   // ezgl example program. 
   // This function will be called by both the unit tests (ece297exercise) 
   // and your main() function in main/src/main.cpp.
   // The unit tests always call loadMap() before calling this function
   // and call closeMap() after this function returns.

   // Setup EZGL Configuration
   // main.ui file defines window layout
   ezgl::application::settings settings;
   settings.main_ui_resource = "libstreetmap/resources/main.ui";
   settings.window_identifier = "MainWindow";
   settings.canvas_identifier = "MainCanvas";

   // Create the EZGL application
   ezgl::application application(settings);

   // Define a Canvas (area to draw upon)
   ezgl::rectangle initial_world({x_from_lon(mapBounds.min_lon), y_from_lat(mapBounds.min_lat)},
                                 {x_from_lon(mapBounds.max_lon), y_from_lat(mapBounds.max_lat)});
   application.add_canvas("MainCanvas", draw_main_canvas, initial_world);

   

   // Run the application
   application.run(nullptr, nullptr, nullptr, nullptr);

}


void draw_main_canvas (ezgl::renderer *g) {
   draw_intersections(g);
   draw_streets(g);
   draw_features(g);
}

void draw_intersections(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();
   g->set_color(ezgl::RED);
   for (IntersectionIdx inter_id = 0; inter_id < intersections.size(); inter_id++) {
      float x = x_from_lon(intersections[inter_id].position.longitude());
      float y = y_from_lat(intersections[inter_id].position.latitude());

      float width = 5;
      float height = width;

      g->fill_rectangle({x, y}, {x + width, y + height});

   }
   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   std::cout << "draw_intersections took " << wallClock.count() <<" seconds" << std::endl;
}

void draw_streets(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();

   g->set_line_width(1);
   g->set_color(187, 187, 187);


   for(int segment_id = 0; segment_id < getNumStreetSegments(); segment_id++){
      if(getStreetSegmentInfo(segment_id).numCurvePoints > 0){
         LatLon point1 = getIntersectionPosition(getStreetSegmentInfo(segment_id).from);
         LatLon point2;
         for(int part_id = 0; part_id < getStreetSegmentInfo(segment_id).numCurvePoints; part_id++){
            point2 = getStreetSegmentCurvePoint(part_id, segment_id);
            
            g->draw_line(latlon_to_point(point1), latlon_to_point(point2));
            point1 = point2;
         }
         point2 = getIntersectionPosition(getStreetSegmentInfo(segment_id).to);
         g->draw_line(latlon_to_point(point1), latlon_to_point(point2));
      } else{
         LatLon point1 = getIntersectionPosition(getStreetSegmentInfo(segment_id).from);
         LatLon point2 = getIntersectionPosition(getStreetSegmentInfo(segment_id).to);

         g->draw_line(latlon_to_point(point1), latlon_to_point(point2));
      }
   }

   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   std::cout << "draw_streets took " << wallClock.count() <<" seconds" << std::endl;
}


void initializeIntersections() {
   mapBounds.max_lat = getIntersectionPosition(0).latitude();
   mapBounds.min_lat = mapBounds.max_lat;
   mapBounds.max_lon = getIntersectionPosition(0).longitude();
   mapBounds.min_lon = mapBounds.max_lon;
   
   intersections.resize(getNumIntersections());

   for (int inter_id = 0; inter_id < getNumIntersections(); inter_id++) {
      intersections[inter_id].position = getIntersectionPosition(inter_id);
      intersections[inter_id].name = getIntersectionName(inter_id);

      mapBounds.max_lat = std::max(mapBounds.max_lat, intersections[inter_id].position.latitude());
      mapBounds.min_lat = std::min(mapBounds.min_lat, intersections[inter_id].position.latitude());
      mapBounds.max_lon = std::max(mapBounds.max_lon, intersections[inter_id].position.longitude());
      mapBounds.min_lon = std::min(mapBounds.min_lon, intersections[inter_id].position.longitude());
   }

   cos_latavg = cos((mapBounds.min_lat + mapBounds.max_lat) * kDegreeToRadian / 2);

}

float x_from_lon(float lon) {
   return kEarthRadiusInMeters * kDegreeToRadian * lon * cos_latavg;
}

float y_from_lat(float lat) {
   return kEarthRadiusInMeters * kDegreeToRadian * lat;
}

float lon_from_x(float x) {
   return x / kEarthRadiusInMeters / kDegreeToRadian / cos_latavg;
}

float lat_from_y(float y) {
   return y / kEarthRadiusInMeters / kDegreeToRadian;
}

ezgl::point2d latlon_to_point(LatLon position){
   float x = kEarthRadiusInMeters * kDegreeToRadian * position.longitude() * cos_latavg;
   float y = kEarthRadiusInMeters * kDegreeToRadian * position.latitude();

   return(ezgl::point2d(x,y));
}