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
#include "m3helpers.h"
#include "m2helpers.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>
#include <limits>

// ==================================== Declare strucs ====================================

struct Intersection_data {
   ezgl::point2d position;
   std::string name;
   bool highlight = false;
   bool processed = false;
   int reachingEdge = 0;
   int reachingNode = 0;
   double bestTime = std::numeric_limits<double>::infinity();
};


struct Map_bounds {
   double max_lat;
   double min_lat;
   double max_lon;
   double min_lon;
};

struct closed_feature_data {
    std::vector<ezgl::point2d> bounds;
    std::string name;
    double area;
    int index;
    int type;
    double minx;
    double maxx;
    double miny;
    double maxy;

    bool operator < (const closed_feature_data& struc) const{
        return (area > struc.area);
    }
};

struct line_feature_data {
   std::vector<ezgl::point2d> bounds;
   std::string name;
   int size;
   int type;
   int index;
};

struct feature_data {
   ezgl::point2d position;
   int index;
   int type;
};

struct segment_data{
   std::vector<ezgl::point2d> points;
   bool oneWay;        // if true, then can only travel in from->to direction
   float speedLimit;        // in m/s
   StreetIdx streetID;     // index of street this segment belongs to
   std::string name;
   std::string OSMtag; //type of segment
};
struct name_data{
   ezgl::point2d position;
   std::string name;
   double angle;
   std::string type;
};

struct POI_data{
    ezgl::point2d position;
    std::string name;

};

struct building_data{
    ezgl::point2d position;
    std::string levels;
    std::string type;
};
// ==================================== Declare global variables ====================================
extern std::vector<Intersection_data> intersections;
extern std::vector<std::vector<int>> streetSegments;
extern std::vector<std::pair<std::string, int>> streetNamesAndIDs;
extern std::vector<std::vector<int>> streetSegments;
extern Map_bounds mapBounds;
extern std::vector<segment_data> street_segments;
extern float cos_latavg;
extern std::vector<name_data> streetNames;
extern std::vector<closed_feature_data> closedFeatures;
extern std::vector<line_feature_data> lineFeatures;
extern std::vector<feature_data> features;
extern std::vector<std::string> mapNames;
extern std::vector<POI_data> fire_hydrants;
extern std::vector<POI_data> FIREfacilities;
extern std::vector<POI_data> EMTfacilities;
extern std::vector<building_data> buildings;
int line_width;
bool setupComplete = false;
double viewPortArea;
bool darkMode;
ezgl::rectangle world;
bool userMode = 0;

//FOR M3
int intersection1 = -1, intersection2 = -1;
bool togglenav = true;
std::vector<IntersectionIdx> firstIntersections;
std::vector<IntersectionIdx> secondIntersections;
std::vector<StreetSegmentIdx> pathSegments;

// ==================================== Declare Helper Functions ====================================
void draw_main_canvas (ezgl::renderer *g);
bool checkContains(double maxx, double minx, double maxy, double miny);
void initializeIntersections();
void draw_intersections(ezgl::renderer *g);
void draw_streets(ezgl::renderer *g);
void draw_features(ezgl::renderer *g);
void set_feature_color(ezgl::renderer *g, int feature_id);
bool set_segment_color(ezgl::renderer *g, std::string streetType);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
gboolean change_dark_switch(GtkSwitch* /*switch*/, gboolean switch_state, ezgl::application* application);
void fillMapDropDown(ezgl::application* application);
double findAngle360(ezgl::point2d point_1, ezgl::point2d point_2);
void drawPOIs(ezgl::renderer *g);
void setWorldScale(ezgl::renderer *g);
gboolean changeUserMode(GtkSwitch* /*switch*/, gboolean switch_state, ezgl::application* application);
LatLon point_to_latlon(ezgl::point2d point);
float x_from_lon(float lon);
float y_from_lat(float lat);
float lon_from_x(float x);
float lat_from_y(float y);
POIIdx findClickablePOI(LatLon my_position);
void initial_setup(ezgl::application *application, bool new_window);
void firstTextEntered(GtkEntry* textBox, ezgl::application* application);
void secondTextEntered(GtkEntry* textBox, ezgl::application* application);
void thirdTextEntered(GtkEntry* textBox, ezgl::application* application);
void fourthTextEntered(GtkEntry* textBox, ezgl::application* application);
void findIntersections(GtkButton* button, ezgl::application* application);
std::string processString(std::string inputString);
void menuCallBack1(GtkComboBoxText* /*box*/, ezgl::application* application);
void menuCallBack2(GtkComboBoxText* /*box*/, ezgl::application* application);
void menuCallBack3(GtkComboBoxText* /*box*/, ezgl::application* application);
void menuCallBack4(GtkComboBoxText* /*box*/, ezgl::application* application);
void map_selection_changed(GtkComboBoxText* /*box*/, ezgl::application* application);
void updateOptions(std::string boxName, std::string streetName, ezgl::application* application);
void drawScaleBar(ezgl::renderer* g);
std::pair<double, std::string> findClosestFireStation(LatLon my_position);
std::pair<double, std::string> findClosestHydrant(LatLon my_position);
std::pair<double, std::string> findClosestHospital(LatLon my_position);
void createHelpPopup(GtkButton* /*button*/, ezgl::application* application);
void updateStreetTextBoxes(ezgl::application *application, IntersectionIdx intersection);
void swapStreetNames(GtkButton* button, ezgl::application* application);
void highlightRoute(ezgl::renderer *g, std::vector<StreetSegmentIdx> highlightedStreetSegments);
void displayDirections(ezgl::application* application, std::string directions);
void clearHighlights(GtkButton* /*button*/, ezgl::application* application);

void drawMap() {
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
   application.run(initial_setup, act_on_mouse_click, nullptr, nullptr);
}

void draw_main_canvas(ezgl::renderer *g) {

   const int MAX_VIEW_PORT_AREA = 200000;

   viewPortArea = g->get_visible_world().area();
   world = g->get_visible_world();
   setWorldScale(g);
   
   if (darkMode) {
      // Draw a dark rectangle that covers the entire main drawing area
      ezgl::rectangle visible_world = g->get_visible_world();
      g->set_color(60, 60, 70); 
      g->fill_rectangle(visible_world);
   }
   
   draw_features(g);
   draw_streets(g);
   draw_intersections(g);
   highlightRoute(g, pathSegments);
   drawScaleBar(g);
   if (viewPortArea < MAX_VIEW_PORT_AREA) {
      drawPOIs(g);
   }
   if (intersection1 != -1 && intersection2 != -1) {
      std::vector<int> temp = findPathBetweenIntersections(0.0,std::pair<IntersectionIdx, IntersectionIdx>(intersection1, intersection2));

      //highlightRoute(g, temp);

      intersection1 = -1;
      intersection2 = -1;
   }
   drawScaleBar(g);
   if(viewPortArea < 200000)
      drawPOIs(g);

      
}

// ==================================== Draw Functions ====================================
// Get screen dimensions and calculate placement of scale bar
// Draw scale bar and write text box beneath for size
// Change colour from black to white if in dark mode
void drawScaleBar(ezgl::renderer* g) {
   // Get ratio of pixels and metres
   double factor = g->get_visible_screen().width()/g->get_visible_world().width();

   // Calculate scale string to display
   int distance = 37/factor;
   std::string distanceString = std::to_string(distance);
   std::string printString = distanceString + "m";

   // Calculate points to draw at
   double x = g->get_visible_world().left() + 30/factor;
   double y = g->get_visible_world().bottom() + 15/factor;
   double barX1 = x - 18/factor;
   double barX2 = x + 18/factor;
   double barY1 = y + 7/factor;
   double barY2 = y + 22/factor;
   ezgl::point2d barPoint1 = {barX1, barY2};
   ezgl::point2d barPoint2 = {barX1, barY1};
   ezgl::point2d barPoint3 = {barX2, barY1};
   ezgl::point2d barPoint4 = {barX2, barY2};
   ezgl::point2d drawPoint = {x, y};

   // Set renderer settings
   darkMode ? g->set_color(ezgl::WHITE) : g->set_color(ezgl::BLACK);
   g->set_font_size(12);
   g->set_text_rotation(0);
   g->set_line_width(line_width/5);

   // Draw scale bar and text
   g->draw_text(drawPoint, printString);
   g->draw_line(barPoint1, barPoint2);
   g->draw_line(barPoint2, barPoint3);
   g->draw_line(barPoint3, barPoint4);
}

//draws poi from loaded data struc
//written by kevin
void drawPOIs(ezgl::renderer *g){
   g->set_color(ezgl::RED);
   if (userMode) {
      for(int POI_index = 0; POI_index < FIREfacilities.size(); POI_index++){
         g->set_font_size(15);
         g->draw_text(ezgl::point2d(FIREfacilities[POI_index].position.x,FIREfacilities[POI_index].position.y+5), FIREfacilities[POI_index].name);
         g->fill_arc(FIREfacilities[POI_index].position, 1, 0, 360);
      }

      for(int POI_index = 0; POI_index < fire_hydrants.size(); POI_index++){
         g->set_font_size(15);
         g->draw_text(ezgl::point2d(fire_hydrants[POI_index].position.x,fire_hydrants[POI_index].position.y+5), fire_hydrants[POI_index].name);
         g->fill_arc(fire_hydrants[POI_index].position, 1, 0, 360);
         
      }
   } else {
      for(int POI_index = 0; POI_index < EMTfacilities.size(); POI_index++){
         g->set_font_size(15);
         g->draw_text(ezgl::point2d(EMTfacilities[POI_index].position.x,EMTfacilities[POI_index].position.y+5), EMTfacilities[POI_index].name);
         g->fill_arc(EMTfacilities[POI_index].position, 1, 0, 360);
      }
   }
}

//draws all highlighted intersections
void draw_intersections(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();
   g->set_color(ezgl::RED);
   for (IntersectionIdx inter_id = 0; inter_id < intersections.size(); inter_id++) {
      float intersectionRadius = 10;
      
      if (intersections[inter_id].highlight) {
         g->fill_arc(intersections[inter_id].position, intersectionRadius, 0, 360);
      }
   }
   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   //std::cout << "draw_intersections took " << wallClock.count() << " seconds" << std::endl;
}

//draws streets and street names only if visible on screen
//written by Kevin
void draw_streets(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();

   for(int segment_id = 0; segment_id < street_segments.size(); segment_id++){
      
      if(set_segment_color(g, street_segments[segment_id].OSMtag)){

         int points = street_segments[segment_id].points.size();

            ezgl::point2d prev = street_segments[segment_id].points[0];
            ezgl::point2d curr = street_segments[segment_id].points[points-1];
            double minx = prev.x;
            double maxx = curr.x;
            double miny = prev.y;
            double maxy = curr.y;
            if(prev.x > curr.x){
               minx = curr.x;
               maxx = prev.x;
            }
            if(prev.y > curr.y){
               miny = curr.y;
               maxy = prev.y;
            }
         if(checkContains(maxx, minx, maxy, miny))
         for(int point_index = 1; point_index < street_segments[segment_id].points.size();point_index++){
            set_segment_color(g, street_segments[segment_id].OSMtag);
            g->draw_line(street_segments[segment_id].points[point_index-1],street_segments[segment_id].points[point_index]);
         }
      }
   }
   if(viewPortArea < 250000)
   for(int streetNameIndex = 0; streetNameIndex < streetNames.size(); streetNameIndex++){
      if(world.contains(streetNames[streetNameIndex].position)){
         g->set_color(ezgl::BLACK);
         g->set_font_size(9);
         g->set_text_rotation(streetNames[streetNameIndex].angle);
         g->draw_text(streetNames[streetNameIndex].position, streetNames[streetNameIndex].name);
      }
   }   

   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   //std::cout << "draw_streets took " << wallClock.count() <<" seconds" << std::endl;
}

// Draw highlighted streets blue using same logic as draw streets, while changing the renderer colour
void highlightRoute(ezgl::renderer *g, std::vector<StreetSegmentIdx> highlightedStreetSegments) {
   for(int segmentIndex = 0; segmentIndex < highlightedStreetSegments.size(); segmentIndex++){
      int segment_id = highlightedStreetSegments[segmentIndex];
      int points = street_segments[segment_id].points.size();

      ezgl::point2d prev = street_segments[segment_id].points[0];
      ezgl::point2d curr = street_segments[segment_id].points[points-1];
      double minx = prev.x;
      double maxx = curr.x;
      double miny = prev.y;
      double maxy = curr.y;
      if(prev.x > curr.x){
         minx = curr.x;
         maxx = prev.x;
      }
      if(prev.y > curr.y){
         miny = curr.y;
         maxy = prev.y;
      }
      if(checkContains(maxx, minx, maxy, miny))
      for(int point_index = 1; point_index < street_segments[segment_id].points.size();point_index++){
         g->set_color(ezgl::BLUE);
         if (darkMode) {
            g->set_color(ezgl::YELLOW);
         }
         g->draw_line(street_segments[segment_id].points[point_index-1],street_segments[segment_id].points[point_index]);
      }
   } 
}

//draws features from preloaded data strucs with closed and line features
//written by Kevin
void draw_features(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();

    for(int feature_index = 0; feature_index < closedFeatures.size(); feature_index++){
      if(checkContains(closedFeatures[feature_index].maxx, closedFeatures[feature_index].minx, closedFeatures[feature_index].maxy, closedFeatures[feature_index].miny)){
         if(closedFeatures[feature_index].area < 100000 && viewPortArea < 1000000){ //REPLACE WITH SCALE 
            set_feature_color(g,closedFeatures[feature_index].type);
            g->fill_poly(closedFeatures[feature_index].bounds);
         } 
         if(closedFeatures[feature_index].area > 100000){
            set_feature_color(g,closedFeatures[feature_index].type);
            g->fill_poly(closedFeatures[feature_index].bounds);
         }
      }
    }

   if(viewPortArea < 10000000)
    for(int feature_index = 0; feature_index < lineFeatures.size(); feature_index++){
      for(int point_index = 1; point_index < lineFeatures[feature_index].bounds.size();point_index++){
         set_feature_color(g,lineFeatures[feature_index].type);
         g->draw_line(lineFeatures[feature_index].bounds[point_index-1],lineFeatures[feature_index].bounds[point_index]);
      }
    }
    
   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   //std::cout << "draw_features took " << wallClock.count() <<" seconds" << std::endl;
}

// ==================================== Helper Functions ====================================

//sets world scale based on screen and world width
//written by kevin
void setWorldScale(ezgl::renderer *g){
   double factor = g->get_visible_screen().width()/g->get_visible_world().width();
   line_width = ((factor+0.13)*10);
   if(line_width > 11){
      line_width = 11;
   }
}

//finds Angle from 0-360 degrees based on tow point2d
//written by Kevin
double findAngle360(ezgl::point2d point_1, ezgl::point2d point_2) {
   double x, y;
      x = point_2.x - point_1.x;
      y = point_2.y - point_1.y;
   double radian = std::atan2(y, x);

   double degrees = radian *(180/M_PI);
   if(degrees< 0)
      degrees += 360;

   return degrees;
}

//checks if part of a feature of road will pass through the visible viewport
//written by Kevin
bool checkContains(double maxx, double minx, double maxy, double miny) {
   ezgl::point2d bottom_right = world.bottom_right();
   ezgl::point2d top_left = world.top_left();
   if(world.contains(maxx, miny)||world.contains(minx, maxy)||world.contains(maxx, miny)||world.contains(minx, maxy)) {
      return true;
   }
   if(maxx > bottom_right.x && minx < top_left.x ) {
      return true;
   } else if (miny < bottom_right.y && maxy > top_left.y ) {
      return true;
   }

   if (maxx < bottom_right.x && maxx > top_left.x ){
      return true;
   } else if (miny > bottom_right.y && miny < top_left.y ) {
      return true;
   }

   if (minx < bottom_right.x && minx > top_left.x ){
      return true;
   } else if (maxy > bottom_right.y && maxy < top_left.y ) {
      return true;
   }
   
   return false;
}

//sets street segment color and line width based on street type
//written by Kevin and Jonathan
bool set_segment_color(ezgl::renderer *g, std::string streetType){
   if(streetType == "motorway"||streetType == "motorway_link"||streetType == "trunk"||streetType == "trunk_link"){
      darkMode ? g->set_color(205, 145, 137) : g->set_color(255, 195, 187);
      g->set_line_width(line_width);
   }else if (streetType == "primary"||streetType == "primary_link"){
      darkMode ? g->set_color(107, 107, 107) : g->set_color(157, 157, 157);
      g->set_line_width(line_width-1);
   }else if (streetType == "secondary"||streetType == "secondary_link"){
      if(viewPortArea > 100000000)
         return false;
      darkMode ? g->set_color(107, 107, 107) : g->set_color(157, 157, 157);
      g->set_line_width(line_width-1);
   }else {
      if(viewPortArea > 10000000)
         return false;
      darkMode ? g->set_color(137, 137, 137) : g->set_color(187, 187, 187);
      g->set_line_width(line_width-1);
   }
   return true;
}

//sets feature color based on feature enum
//written by Kevin
void set_feature_color(ezgl::renderer *g, int feature_id){
   g->set_line_width(1);
   switch(feature_id){
      case UNKNOWN:
      case BUILDING:
         darkMode ? g->set_color(167, 167, 167) : g->set_color(217, 217, 217);
         break;
      case PARK:
      case GREENSPACE:
      case GOLFCOURSE:
         darkMode ? g->set_color(142, 200, 168) : g->set_color(192, 250, 218);
         break;
      case BEACH:
         darkMode ? g->set_color(197, 186, 136) : g->set_color(247, 236, 186);
         break;
      case LAKE:
      case RIVER:
      case STREAM:
      case GLACIER:
         darkMode ? g->set_color(108, 176, 205) : g->set_color(158, 226, 255);
         break;
      case ISLAND:
         darkMode ? g->set_color(60, 60, 70) : g->set_color(255, 255, 255);
         break;
      default:
         darkMode ? g->set_color(167, 167, 167) : g->set_color(217, 217, 217);
      break;
   }
}

// Helper function to convert longitude to x-coordinate projection
// Written by Jonathan
float x_from_lon(float lon) {
   return kEarthRadiusInMeters * kDegreeToRadian * lon * cos_latavg;
}

// Helper function to convert latitude to y-coordinate projection
// Written by Jonathan
float y_from_lat(float lat) {
   return kEarthRadiusInMeters * kDegreeToRadian * lat;
}

// Helper function to convert x-coordinate projection to longitude
// Written by Jonathan
float lon_from_x(float x) {
   return x / kEarthRadiusInMeters / kDegreeToRadian / cos_latavg;
}

// Helper function to convert y-coordinate projection to latitude
// Written by Jonathan
float lat_from_y(float y) {
   return y / kEarthRadiusInMeters / kDegreeToRadian;
}
//converts latlon coordinates to point2d
//written by Kevin
ezgl::point2d latlon_to_point(LatLon position) {
   float x = kEarthRadiusInMeters * kDegreeToRadian * position.longitude() * cos_latavg;
   float y = kEarthRadiusInMeters * kDegreeToRadian * position.latitude();

   return(ezgl::point2d(x,y));
}

// Convert point2d to LatLon
LatLon point_to_latlon(ezgl::point2d point) {
   float longitude = point.x / (kEarthRadiusInMeters * kDegreeToRadian * cos_latavg);
   float latitude = point.y / (kEarthRadiusInMeters * kDegreeToRadian);

   return LatLon(latitude, longitude);
}

// Update options displayed as suggested streets based on prefixes
void updateOptions(std::string boxName, std::string streetName, ezgl::application* application) {
   // Get street names and initialize vector of options
   std::vector<StreetIdx> ids = findStreetIdsFromPartialStreetName(streetName);
   std::vector<std::string> options;

   // Check if there are any options to display, populate options vector and remove duplicates if there are any
   if (ids.size() == 0) {
      options.push_back("No Matches");
   } else {
      options.resize(ids.size());
      for (int nameOptionsIdx = 0; nameOptionsIdx < ids.size(); nameOptionsIdx++) {
         options[nameOptionsIdx] = getStreetName(ids[nameOptionsIdx]);
      }
      auto lastUniqueElement = std::unique(options.begin(), options.end());
      options.erase(lastUniqueElement, options.end());
   }

   // Clear current text box options
   const char* charVersion = boxName.c_str();
   GtkComboBoxText* textBoxText = (GtkComboBoxText*) application->find_widget(charVersion);
   gtk_combo_box_text_remove_all(textBoxText);

   if (options.size() > 8) {
      return;
   }

   // Add new options and open popup menu
   for (int nameOptionsIdx = 0; nameOptionsIdx < options.size(); nameOptionsIdx++) {
      const char* optionCharVersion = (options[nameOptionsIdx]).c_str();
      gtk_combo_box_text_append_text(textBoxText, optionCharVersion);
   }
   GtkComboBox* textBox = (GtkComboBox*) application->find_widget(charVersion);
   gtk_combo_box_popup(textBox);
}

// Get entered text from first text box, call function to update coresponding dropdown
void firstTextEntered(GtkEntry* textBox, ezgl::application* application) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = text;
   updateOptions("Street1Options", streetString, application);
}

// Get entered text from second text box, call function to update coresponding dropdown
void secondTextEntered(GtkEntry* textBox, ezgl::application* application) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = text;
   updateOptions("Street2Options", streetString, application);
}

// Get entered text from third text box, call function to update coresponding dropdown
void thirdTextEntered(GtkEntry* textBox, ezgl::application* application) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = text;
   updateOptions("Street3Options", streetString, application);
}

// Get entered text from fourth text box, call function to update coresponding dropdown
void fourthTextEntered(GtkEntry* textBox, ezgl::application* application) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = text;
   updateOptions("Street4Options", streetString, application);
}

// If user selected option from menu, display it on text box and remove suggestions from screen (first menu)
void menuCallBack1(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {
      GtkComboBoxText* textBox = (GtkComboBoxText*) application->find_widget("Street1Options");
      if (gtk_combo_box_text_get_active_text(textBox)) {
         const gchar* myString = gtk_combo_box_text_get_active_text(textBox);
         if (strcmp(myString, "No Matches") != 0) {
            GtkEntry* labelBox = (GtkEntry*) application->find_widget("Street1");
            gtk_entry_set_text(labelBox, myString);
            gtk_combo_box_text_remove_all(textBox);
         }
      }
   }
}

// If user selected option from menu, display it on text box and remove suggestions from screen (second menu)
void menuCallBack2(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {
      GtkComboBoxText* textBox = (GtkComboBoxText*) application->find_widget("Street2Options");
      if (gtk_combo_box_text_get_active_text(textBox)) {
         const gchar* myString = gtk_combo_box_text_get_active_text(textBox);
         if ((strcmp(myString, "Name Complete") != 0) && (strcmp(myString, "No Matches") != 0)) {
            GtkEntry* labelBox = (GtkEntry*) application->find_widget("Street2");
            gtk_entry_set_text(labelBox, myString);
            gtk_combo_box_text_remove_all(textBox);
         }
      }
   }
}

// If user selected option from menu, display it on text box and remove suggestions from screen (third menu)
void menuCallBack3(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {
      GtkComboBoxText* textBox = (GtkComboBoxText*) application->find_widget("Street3Options");
      if (gtk_combo_box_text_get_active_text(textBox)) {
         const gchar* myString = gtk_combo_box_text_get_active_text(textBox);
         if ((strcmp(myString, "Name Complete") != 0) && (strcmp(myString, "No Matches") != 0)) {
            GtkEntry* labelBox = (GtkEntry*) application->find_widget("Street3");
            gtk_entry_set_text(labelBox, myString);
            gtk_combo_box_text_remove_all(textBox);
         }
      }
   }
}

// If user selected option from menu, display it on text box and remove suggestions from screen (fourth menu)
void menuCallBack4(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {
      GtkComboBoxText* textBox = (GtkComboBoxText*) application->find_widget("Street4Options");
      if (gtk_combo_box_text_get_active_text(textBox)) {
         const gchar* myString = gtk_combo_box_text_get_active_text(textBox);
         if ((strcmp(myString, "Name Complete") != 0) && (strcmp(myString, "No Matches") != 0)) {
            GtkEntry* labelBox = (GtkEntry*) application->find_widget("Street4");
            gtk_entry_set_text(labelBox, myString);
            gtk_combo_box_text_remove_all(textBox);
         }
      }
   }
}

// When find button is pressed, find intersections of the streets currently entered
// Highlight the intersections, pan and zoom to them
void findIntersections(GtkButton* /*button*/, ezgl::application* application) {
   // Get strings from text boxes
   GtkEntry* streetNameBox1 = (GtkEntry*) application->find_widget("Street1");
   GtkEntry* streetNameBox2 = (GtkEntry*) application->find_widget("Street2");
   GtkEntry* streetNameBox3 = (GtkEntry*) application->find_widget("Street3");
   GtkEntry* streetNameBox4 = (GtkEntry*) application->find_widget("Street4");
   std::string street1 = gtk_entry_get_text(streetNameBox1);
   std::string street2 = gtk_entry_get_text(streetNameBox2);
   std::string street3 = gtk_entry_get_text(streetNameBox3);
   std::string street4 = gtk_entry_get_text(streetNameBox4);

   // Check if user entered duplicate street names
   if (street1 == street2) {
      application->create_popup_message("Repeated Street", "The provided streets 1 and 2 are the same");
      return;
   } else if (street3 == street4) {
      application->create_popup_message("Repeated Street", "The provided streets 3 and 4 are the same");
      return;
   }

   // Make a pair of street ids
   std::pair<StreetIdx, StreetIdx> streetPair1;
   std::pair<StreetIdx, StreetIdx> streetPair2;

   // Get street names from partials
   std::vector<StreetIdx> firstResults = findStreetIdsFromPartialStreetName(street1);
   std::vector<StreetIdx> secondResults = findStreetIdsFromPartialStreetName(street2);
   std::vector<StreetIdx> thirdResults = findStreetIdsFromPartialStreetName(street3);
   std::vector<StreetIdx> fourthResults = findStreetIdsFromPartialStreetName(street4);

   // Create temporary vectors to hold intersections of the first two and latter two streets
   std::vector<IntersectionIdx> tempFirstIntersections;
   std::vector<IntersectionIdx> tempSecondIntersections;

   // Booleans to check if the pairs of streets actually intersect
   bool intersection1Exists = false;
   bool intersection2Exists = false;

   // Floats to track where to pan the window to
   float x1 = 0;
   float x2 = 0;
   float y1 = 0;
   float y2 = 0;
   // Check if the current street names are valid
   if (firstResults.size() == 0) {
      application->create_popup_message("Incorrect Street Names", "There were no streets found matching the name provided for Street 1");
      return;
   } else if (secondResults.size() == 0) {
      application->create_popup_message("Incorrect Street Names", "There were no streets found matching the name provided for Street 2");
      return;
   } else if (thirdResults.size() == 0) {
      application->create_popup_message("Incorrect Street Names", "There were no streets found matching the name provided for Street 3");
      return;
   } else if (fourthResults.size() == 0) {
      application->create_popup_message("Incorrect Street Names", "There were no streets found matching the name provided for Street 4");
      return;
   }
   std::cout << "Second" << std::endl;

   // Loop through all combinations of street names based on prefix and check for intersections
   for (int street1StringsIdx = 0; street1StringsIdx < firstResults.size(); street1StringsIdx++) {
      for (int street2StringsIdx = 0; street2StringsIdx < secondResults.size(); street2StringsIdx++) {
         streetPair1.first = firstResults[street1StringsIdx];
         streetPair1.second = secondResults[street2StringsIdx];
         std::vector<IntersectionIdx> intersectionsOfStreets = findIntersectionsOfTwoStreets(streetPair1);
         if (intersectionsOfStreets.size() != 0) {
            intersection1Exists = true;
            std::string foundStreet1 = getStreetName(streetPair1.first);
            std::string foundStreet2 = getStreetName(streetPair1.second);
            const gchar* charVersionStreet1 = foundStreet1.c_str();
            const gchar* charVersionStreet2 = foundStreet2.c_str();
            for (int intersectingStreetsId = 0; intersectingStreetsId < intersectionsOfStreets.size(); intersectingStreetsId++) {
               // Set intersection to be highlighted when redrawn
               intersections[intersectionsOfStreets[intersectingStreetsId]].highlight = true;
               tempFirstIntersections.push_back(intersectionsOfStreets[intersectingStreetsId]);
               // Output intersection name information
               std::stringstream closestIntersection;
               closestIntersection << "Selected: " << intersections[intersectionsOfStreets[intersectingStreetsId]].name;
               application->update_message(closestIntersection.str());
            }

            // Pan to intersections
            x1 = intersections[intersectionsOfStreets[0]].position.x;
            y1 = intersections[intersectionsOfStreets[0]].position.y;

            gtk_entry_set_text(streetNameBox1, charVersionStreet1);
            gtk_entry_set_text(streetNameBox2, charVersionStreet2);
         }
      }
   }

   // Repeat the process for the second pair of street names
   for (int street3StringsIdx = 0; street3StringsIdx < thirdResults.size(); street3StringsIdx++) {
      for (int street4StringsIdx = 0; street4StringsIdx < fourthResults.size(); street4StringsIdx++) {
         streetPair2.first = thirdResults[street3StringsIdx];
         streetPair2.second = fourthResults[street4StringsIdx];
         std::vector<IntersectionIdx> intersectionsOfStreets = findIntersectionsOfTwoStreets(streetPair2);
         if (intersectionsOfStreets.size() != 0) {
            intersection2Exists = true;
            std::string foundStreet1 = getStreetName(streetPair2.first);
            std::string foundStreet2 = getStreetName(streetPair2.second);
            const gchar* charVersionStreet1 = foundStreet1.c_str();
            const gchar* charVersionStreet2 = foundStreet2.c_str();
            for (int intersectingStreetsId = 0; intersectingStreetsId < intersectionsOfStreets.size(); intersectingStreetsId++) {
               // Set intersection to be highlighted when redrawn
               intersections[intersectionsOfStreets[intersectingStreetsId]].highlight = true;
               tempSecondIntersections.push_back(intersectionsOfStreets[intersectingStreetsId]);
               // Output intersection name information
               std::stringstream closestIntersection;
               closestIntersection << "Selected: " << intersections[intersectionsOfStreets[intersectingStreetsId]].name;
               application->update_message(closestIntersection.str());
            }
            x2 = intersections[intersectionsOfStreets[0]].position.x;
            y2 = intersections[intersectionsOfStreets[0]].position.y; 

            gtk_entry_set_text(streetNameBox3, charVersionStreet1);
            gtk_entry_set_text(streetNameBox4, charVersionStreet2);
         }
      }
   }

   // Check if both starting and ending intersection are the same
   if (tempFirstIntersections.size() > 0 && tempSecondIntersections.size() > 0) {
      if (tempFirstIntersections[0] == tempSecondIntersections[0]) {
         application->create_popup_message("Invalid Route", "The provided intersections are at the same location");
         return;
      }
   }
   

   // If both pairs of streets generated unique intersections, update global vectors using the temporary ones
   // Set the x and y coordinates and pan to put the route on screen
   if (intersection1Exists && intersection2Exists) {
      float xMin = std::min(x1, x2);
      float xMax = std::max(x1, x2);
      float yMin = std::min(y1, y2);
      float yMax = std::max(y1, y2);

      float x = (xMin + xMax)/2;
      float y = (yMin + yMax)/2;

      float xDiff = xMax - xMin;
      float yDiff = yMax - yMin;

      int zoomBoxSize = (std::max(xDiff, yDiff))*1.1;
      ezgl::renderer* g = application->get_renderer();
      g->set_visible_world(ezgl::rectangle({x-zoomBoxSize, y-zoomBoxSize}, {x+zoomBoxSize, y+zoomBoxSize}));            
      application->refresh_drawing();

      bool changeFirstVector = true;
      bool changeSecondVector = true;
      if (firstIntersections.size() == 1) {
         for (int intersectionIndex = 0; intersectionIndex < tempFirstIntersections.size(); intersectionIndex++) {
            if (tempFirstIntersections[intersectionIndex] == firstIntersections[0]) {
               changeFirstVector = false;
               break;
            }
         }
      }
      if (secondIntersections.size() == 1) {
         for (int intersectionIndex = 0; intersectionIndex < tempSecondIntersections.size(); intersectionIndex++) {
            if (tempSecondIntersections[intersectionIndex] == secondIntersections[0]) {
               changeSecondVector = false;
               break;
            }
         }
      }
      if (changeFirstVector) {
         firstIntersections.clear();
         firstIntersections.resize(tempFirstIntersections.size());
         for (int intersectionIndex = 0; intersectionIndex < tempFirstIntersections.size(); intersectionIndex++) {
            firstIntersections[intersectionIndex] = tempFirstIntersections[intersectionIndex];
         }
      }
      if (changeSecondVector) {
         secondIntersections.clear();
         secondIntersections.resize(tempSecondIntersections.size());
         for (int intersectionIndex = 0; intersectionIndex < tempSecondIntersections.size(); intersectionIndex++) {
            secondIntersections[intersectionIndex] = tempSecondIntersections[intersectionIndex];
         }
      }
   } else {
      // The intersections were not unique
      if (intersection1Exists == false) {
         application->create_popup_message("No Intersections Found", "The provided streets 1 and 2 do not intersect.");
      } else if (intersection2Exists == false) {
         application->create_popup_message("No Intersections Found", "The provided streets 3 and 4 do not intersect.");
      }
      return;
   }
   // Pass the intersections to the findPath function and store the return in a temporary vector
   std::pair<IntersectionIdx, IntersectionIdx> intersectionPair(firstIntersections[0], secondIntersections[0]);
   std::vector<StreetSegmentIdx> tempPath = findPathBetweenIntersections(15, intersectionPair);

   // Copy to global vector
   pathSegments.resize(tempPath.size());
   for (int segmentIndex = 0; segmentIndex < tempPath.size(); segmentIndex++) {
      pathSegments[segmentIndex] = tempPath[segmentIndex];
   }

   // Display the driving directions in the UI
   std::string directions = getTravelDirections(pathSegments, intersectionPair.first, intersectionPair.second);
   displayDirections(application, directions);
   application->refresh_drawing();
}

// Initialize GObjects and connect callbacks to signals
void initial_setup(ezgl::application* application, bool /*new_window*/) {
   // Display map title
   application->update_message("WELCOME TO MAPPER CD-031");

   // Connect glade objects
   GObject* firstBox = application->get_object("Street1");
   GObject* secondBox = application->get_object("Street2");
   GObject* thirdBox = application->get_object("Street3");
   GObject* fourthBox = application->get_object("Street4");
   GObject* findButton = application->get_object("FindIntersections");
   GObject* dropDown1 = application->get_object("Street1Options");
   GObject* dropDown2 = application->get_object("Street2Options");
   GObject* dropDown4 = application->get_object("Street3Options");
   GObject* dropDown5 = application->get_object("Street4Options");
   GObject* dropDown3 = application->get_object("MapSelection");
   GObject* darkSwitch = application->get_object("DarkMode");
   GObject* modeTypeSwitch = application->get_object("ModeSwitch");
   GObject* helpButton = application->get_object("HelpButton");
   GObject* swapIntersectionsButton = application->get_object("SwitchIntersections");
   GObject* clearButton = application->get_object("ClearButton");

   // Connect signals
   g_signal_connect(firstBox, "activate", G_CALLBACK(firstTextEntered), application);
   g_signal_connect(secondBox, "activate", G_CALLBACK(secondTextEntered), application);
   g_signal_connect(thirdBox, "activate", G_CALLBACK(thirdTextEntered), application);
   g_signal_connect(fourthBox, "activate", G_CALLBACK(fourthTextEntered), application);
   g_signal_connect(findButton, "clicked", G_CALLBACK(findIntersections), application);
   g_signal_connect(darkSwitch, "state-set", G_CALLBACK(change_dark_switch), application);
   g_signal_connect(dropDown1, "changed", G_CALLBACK(menuCallBack1), application);
   g_signal_connect(dropDown2, "changed", G_CALLBACK(menuCallBack2), application);
   g_signal_connect(dropDown4, "changed", G_CALLBACK(menuCallBack3), application);
   g_signal_connect(dropDown5, "changed", G_CALLBACK(menuCallBack4), application);
   g_signal_connect(dropDown3, "changed", G_CALLBACK(map_selection_changed), application);
   g_signal_connect(modeTypeSwitch, "state-set", G_CALLBACK(changeUserMode), application);
   g_signal_connect(helpButton, "clicked", G_CALLBACK(createHelpPopup), application);
   g_signal_connect(swapIntersectionsButton, "clicked", G_CALLBACK(swapStreetNames), application);
   g_signal_connect(clearButton, "clicked", G_CALLBACK(clearHighlights), application);

   // Populate dropdown menu of map names
   fillMapDropDown(application);

   darkMode = false;
   setupComplete = true;
}


// The function act_on_mouse_click determines how to respond to user input upon mouse click.
// There are two possible responses to mouse clicks: clicking on intersection, or POI.
// Upon clicking on screen, the function determines if the click was within proximity
// of an intersection. If yes, it will highlight the intersection and display its name.
// Upon clicking on screen, the function also determines if the click was within proximity
// of a POI. If yes, it will create a pop-up message to display the POI information (name).
// Written by Jonathan
void act_on_mouse_click(ezgl::application* app, GdkEventButton* /*event*/, double x, double y) {
   const float INTERSECTION_CLICK_PROXIMITY = 6;
   const float POI_CLICK_PROXIMITY = 10;

   std::cout << "Mouse 2 clicked at (" << x << "," << y << ")\n";

   // Convert mouse click coordinates (xy) to LatLon to determine closest intersection
   LatLon position = LatLon(lat_from_y(y), lon_from_x(x));
   IntersectionIdx inter_id = findClosestIntersection(position);
   POIIdx poi_id = findClickablePOI(position);

   // Only highlight and display intersection if clicked within close proximity
   if (findDistanceBetweenTwoPoints(position, getIntersectionPosition(inter_id)) < INTERSECTION_CLICK_PROXIMITY) {
      // Change the visibility of the selected intersection
      intersections[inter_id].highlight = !intersections[inter_id].highlight;


      //TEMP FOR TESTING
      if(togglenav){
         intersection1 = inter_id;
         togglenav = !togglenav;
      }else{
         intersection2 = inter_id;
         togglenav = !togglenav;
      }



      // Output intersection name information
      std::stringstream closestIntersection;
      closestIntersection << "intersection "<< inter_id << " : " << intersections[inter_id].name;

      app->update_message(closestIntersection.str());

      // Update streets text boxes
      updateStreetTextBoxes(app, inter_id);

      // Refresh map drawing
      app->refresh_drawing();
   }

   // Only create POI pop-up if clicked within close proximity
   else if (findDistanceBetweenTwoPointsxy(latlon_to_point(position), buildings[poi_id].position) < POI_CLICK_PROXIMITY) {
      // Output intersection name information
      std::stringstream closestPOI;
      std::string type = buildings[poi_id].type;
      std::string levels = buildings[poi_id].levels; 

      if(type == "yes"){
         type = "Unknown";
      }
      if(levels == ""){
         levels = "Unknown";
      }
      

      if(userMode){
         std::pair<double, std::string > fire_station = findClosestFireStation(position);
         std::pair<double, std::string > hydrant = findClosestHydrant(position);
         closestPOI << "Type: " << type << "\n" <<"Floors: " << levels << "\n" << "Closest Fire Station: " << fire_station.second << "\n" 
                    << "Distance: "<< fire_station.first << " m" << "\n" << "Closest Fire Hydrant: " << hydrant.first << " m" << "\n"
                    << "Located near: " << hydrant.second;
      }else{
         std::pair<double, std::string > hospital = findClosestHospital(position);
         closestPOI << "Type: " << type << "\n" <<"Floors: " << levels << "\n" << "Closest Hospital: " << hospital.second << "\n" 
                    << "Distance: "<< hospital.first << " m";
      }

      app->create_popup_message("Building Information", closestPOI.str().c_str());

      // Refresh map drawing
      app->refresh_drawing();
   }
}

// The function map_selection_changed is a callback function that responds to a change
// in city map selection. The overall steps are as follows: close the current map, load
// the new map, change the canvas coordinate system to adjust for the new map, and
// refresh the drawing.
// Global data structures are cleared in loadMap() and closeMap() functions in order to
// prevent multiple city maps drawn at the same time.
// Written by Jonathan
void map_selection_changed(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {

      // STEPS:
      // closeMap
      // loadMap
      // change coordinates
      // refresh

      // Cast to GtkComboBox
      GtkComboBoxText* gtk_combo_box_text = GTK_COMBO_BOX_TEXT(application->find_widget("MapSelection"));
      
      if (gtk_combo_box_text_get_active_text(gtk_combo_box_text)) {      
         std::string new_map_name = gtk_combo_box_text_get_active_text(gtk_combo_box_text);
         std::string new_path = "/cad2/ece297s/public/maps/" + new_map_name;
         std::cout << new_path << std::endl;

         new_path = new_path + ".streets.bin";
         std::cout << new_path << std::endl;

         std::cout << "Closing map\n";         
         closeMap();

         bool load_success = loadMap(new_path);
         load_success ? std::cout << "Successfully loaded map" << std::endl : std::cerr << "Failed to load map" << std::endl;

         ezgl::rectangle new_world({x_from_lon(mapBounds.min_lon), y_from_lat(mapBounds.min_lat)},
                                 {x_from_lon(mapBounds.max_lon), y_from_lat(mapBounds.max_lat)});
         application->change_canvas_world_coordinates("MainCanvas", new_world);
         application->refresh_drawing();
         application->create_popup_message("Load Complete", "Your new map is finished loading!");
      }
   }
}

// The function change_dark_switch is a callback function that responds to a change in the dark mode switch.
// Upon changing the switch, the global boolean variable darkMode is updated to reflect the new state.
// All colour settings are already dependent on the global variable, so the only step required is to
// refresh the drawing.
// The function returns false to indicate that GTK should update the switch value visually.
// Written by Jonathan
gboolean change_dark_switch(GtkSwitch* /*switch*/, gboolean switch_state, ezgl::application* application) {
   
   // Update darkMode global variable based on the switch being on or off
   darkMode = switch_state;

   //Force a redraw to reflect the new dark mode state
   application->refresh_drawing();

   // GTK's usual callback is now called to update the switch visually
   return false;
}

// Fill dropdown for map menu
void fillMapDropDown(ezgl::application* application) {
   const char* charVersion = "MapSelection";
   GtkComboBoxText* menu = (GtkComboBoxText*) application->find_widget(charVersion);
   for (int mapIndex = 0; mapIndex < mapNames.size(); mapIndex++) {
      const char* mapNameCharVersion = (mapNames[mapIndex]).c_str();
      gtk_combo_box_text_append_text(menu, mapNameCharVersion);
   }
}

// Loops through all POIs and calls findDistanceBetweenTwoPoints to check its distance
// Tracks the closest POI by POIIdx and tracks the smallest distance
// Writen by Jonathan

POIIdx findClickablePOI(LatLon my_position) {
   POIIdx closestPOI = 0;
   double minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), buildings[0].position);

   for (int POI_idx = 0; POI_idx < buildings.size(); POI_idx++) {
      if (findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), buildings[POI_idx].position) < minDistance) {
         minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), buildings[POI_idx].position);
         closestPOI = POI_idx;
      }
   }
   return closestPOI;
}

std::pair<double, std::string> findClosestFireStation(LatLon my_position){
   POIIdx closestPOI = 0;
   double minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), FIREfacilities[0].position);

   for (int POI_idx = 0; POI_idx < FIREfacilities.size(); POI_idx++) {
      if (findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), FIREfacilities[POI_idx].position) < minDistance) {
         minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), FIREfacilities[POI_idx].position);
         closestPOI = POI_idx;
      }
   }
   return {minDistance, FIREfacilities[closestPOI].name};
}

std::pair<double, std::string> findClosestHydrant(LatLon my_position){
   POIIdx closestPOI = 0;
   double minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), fire_hydrants[0].position);

   for (int POI_idx = 0; POI_idx < fire_hydrants.size(); POI_idx++) {
      if (findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), fire_hydrants[POI_idx].position) < minDistance) {
         minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), fire_hydrants[POI_idx].position);
         closestPOI = POI_idx;
      }
   }
   return {minDistance, getIntersectionName(findClosestIntersection(point_to_latlon(fire_hydrants[closestPOI].position)))};
}

std::pair<double, std::string> findClosestHospital(LatLon my_position){
   POIIdx closestPOI = 0;
   double minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), EMTfacilities[0].position);

   for (int POI_idx = 0; POI_idx < EMTfacilities.size(); POI_idx++) {
      if (findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), EMTfacilities[POI_idx].position) < minDistance) {
         minDistance = findDistanceBetweenTwoPointsxy(latlon_to_point(my_position), EMTfacilities[POI_idx].position);
         closestPOI = POI_idx;
      }
   }
   return {minDistance, EMTfacilities[closestPOI].name};
}

gboolean changeUserMode(GtkSwitch* /*switch*/, gboolean switch_state, ezgl::application* application) {
   userMode = switch_state;
   std::cout << "User mode is " << userMode << std::endl;
   application->refresh_drawing();
   return false;
}

// Create popup for instructions for user
void createHelpPopup(GtkButton* /*button*/, ezgl::application* application) {
   application->create_popup_message("User Instructions", 
   "1. Type your streets into the provided search bars.\n2. Allow the amazing autocomplete to fix your spelling.\n3. Press find route and watch the amazing results.\n4. Clicking intersections also fills in the street names.\n5. To intersections using mouse clicks, first click any intersction. Then, press the \"Switch Intersections\" button before clicking a second intersection.\nTip: The map UI is optimal in full screen");
}

// Update the strings in the street text boxes, called when user clicks on an intersection
void updateStreetTextBoxes(ezgl::application* application, IntersectionIdx intersection) {
   std::string intersectionStreets = getIntersectionName(intersection);

   std::string street1;
   std::string street2;

   int splitIndex = intersectionStreets.find('&');
   street1 = intersectionStreets.substr(0, splitIndex-1);
   street2 = intersectionStreets.substr(splitIndex+2);

   int occurences = std::count(intersectionStreets.begin(), intersectionStreets.end(), '&');
   if (occurences > 1) {
      int secondIndex = street2.find('&');
      street2 = street2.substr(0, secondIndex-1);
   }

   GtkEntry* streetNameBox1 = (GtkEntry*) application->find_widget("Street1");
   const gchar* charVersionStreet1 = street1.c_str();
   gtk_entry_set_text(streetNameBox1, charVersionStreet1);

   GtkEntry* streetNameBox2 = (GtkEntry*) application->find_widget("Street2");
   const gchar* charVersionStreet2 = street2.c_str();
   gtk_entry_set_text(streetNameBox2, charVersionStreet2);

   firstIntersections.clear();
   firstIntersections.push_back(intersection);
}

// Swap the names in the street boxes, also swaps the values in the associated vectors of intersection IDs
void swapStreetNames(GtkButton* /*button*/, ezgl::application* application) {
   GtkEntry* streetNameBox1 = (GtkEntry*) application->find_widget("Street1");
   GtkEntry* streetNameBox2 = (GtkEntry*) application->find_widget("Street2");
   GtkEntry* streetNameBox3 = (GtkEntry*) application->find_widget("Street3");
   GtkEntry* streetNameBox4 = (GtkEntry*) application->find_widget("Street4");

   std::string string1 = gtk_entry_get_text(streetNameBox1);
   std::string string2 = gtk_entry_get_text(streetNameBox2);
   std::string string3 = gtk_entry_get_text(streetNameBox3);
   std::string string4 = gtk_entry_get_text(streetNameBox4);

   const gchar* charVersionStreet1 = string1.c_str();
   const gchar* charVersionStreet2 = string2.c_str();
   const gchar* charVersionStreet3 = string3.c_str();
   const gchar* charVersionStreet4 = string4.c_str();

   gtk_entry_set_text(streetNameBox1, charVersionStreet3);
   gtk_entry_set_text(streetNameBox2, charVersionStreet4);
   gtk_entry_set_text(streetNameBox3, charVersionStreet1);
   gtk_entry_set_text(streetNameBox4, charVersionStreet2);

   firstIntersections.swap(secondIntersections);
}

// Display directions in the UI
void displayDirections(ezgl::application* application, std::string directions) {
   GtkTextBuffer* textDisplay = (GtkTextBuffer*) application->find_widget("Directions");
   std::string outputDirections = "Directions\n" + directions;
   const gchar* charVersion = outputDirections.c_str();
   gtk_text_buffer_set_text(textDisplay, charVersion, -1);
}

// Unhighlight intersections and streets
void clearHighlights(GtkButton* /*button*/, ezgl::application* application) {
   for (int intersectionID = 0; intersectionID < getNumIntersections(); intersectionID++) {
      intersections[intersectionID].highlight = false;
   }
   pathSegments.clear();
   application->refresh_drawing();
}