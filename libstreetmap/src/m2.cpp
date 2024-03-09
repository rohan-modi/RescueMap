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
#include <sstream>
#include <chrono>
#include "m1.h"
#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include <thread>

// Declare global variables

struct Intersection_data {
   ezgl::point2d position;
   std::string name;
   bool highlight = false;
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

extern std::vector<Intersection_data> intersections;
extern std::vector<std::vector<int>> streetSegments;
extern std::vector<std::pair<std::string, int>> streetNamesAndIDs;
extern std::vector<std::vector<int>> streetSegments;
extern Map_bounds mapBounds;
extern std::vector<segment_data> street_segments;
double viewPortArea;
extern float cos_latavg;

extern std::vector<closed_feature_data> closedFeatures;
extern std::vector<line_feature_data> lineFeatures;
extern std::vector<feature_data> features;


// Declare helper functions
void draw_main_canvas (ezgl::renderer *g);
void initializeIntersections();
void draw_intersections(ezgl::renderer *g);
void draw_streets(ezgl::renderer *g);
void draw_POI(ezgl::renderer *g);
ezgl::point2d latlon_to_point(LatLon position);
void draw_features(ezgl::renderer *g);
void set_feature_color(ezgl::renderer *g, int feature_id);
bool set_segment_color(ezgl::renderer *g, std::string streetType);
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);

float x_from_lon(float lon);
float y_from_lat(float lat);

float lon_from_x(float x);
float lat_from_y(float y);

struct buttonData {
   std::string string1;
   std::string string2;
   ezgl::application* application;
};

void initial_setup(ezgl::application *application, bool new_window);
void firstTextEntered(GtkEntry* textBox, buttonData* myStruct);
void secondTextEntered(GtkEntry* textBox, buttonData* myStruct);
void findIntersections(GtkButton* button, buttonData* myStruct);
std::string processString(std::string inputString);
void menuCallBack1(GtkComboBoxText* /*box*/, ezgl::application* application);
void menuCallBack2(GtkComboBoxText* /*box*/, ezgl::application* application);
void updateOptions(std::string boxName, std::string streetName, ezgl::application* application);

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


void draw_main_canvas (ezgl::renderer *g) {
   std::cout << g->get_visible_world().area() << std::endl;
   viewPortArea = g->get_visible_world().area();
   draw_features(g);
   draw_intersections(g);
   draw_streets(g);
   
}

void draw_intersections(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();
   g->set_color(ezgl::RED);
   for (IntersectionIdx inter_id = 0; inter_id < intersections.size(); inter_id++) {
      float intersectionRadius = 5;
      
      if (intersections[inter_id].highlight) {
         g->fill_arc(intersections[inter_id].position, intersectionRadius, 0, 360);
      }
   }
   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   std::cout << "draw_intersections took " << wallClock.count() << " seconds" << std::endl;
}

void draw_streets(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();

   for(int segment_id = 0; segment_id < street_segments.size(); segment_id++){
      if(set_segment_color(g, street_segments[segment_id].OSMtag))
      for(int point_index = 1; point_index < street_segments[segment_id].points.size();point_index++){
            g->draw_line(street_segments[segment_id].points[point_index-1],street_segments[segment_id].points[point_index]);
         
      }
   }

   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   std::cout << "draw_streets took " << wallClock.count() <<" seconds" << std::endl;
}

void draw_features(ezgl::renderer *g){
   auto startTime = std::chrono::high_resolution_clock::now();
    
    
    for(int feature_index = 0; feature_index < closedFeatures.size(); feature_index++){
      if(closedFeatures[feature_index].area > 100000){ //REPLACE WITH SCALE 
         set_feature_color(g,closedFeatures[feature_index].type);
         g->fill_poly(closedFeatures[feature_index].bounds);
       
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
   std::cout << "draw_features took " << wallClock.count() <<" seconds" << std::endl;
}


bool set_segment_color(ezgl::renderer *g, std::string streetType){

   if(streetType == "motorway"||streetType == "motorway_link"||streetType == "trunk"||streetType == "trunk_link"){
      g->set_color(255, 195, 187);
      g->set_line_width(1);
   }else if (streetType == "primary"||streetType == "primary_link"){
      g->set_color(157, 157, 157);
      g->set_line_width(1);
   }else {
      if(viewPortArea > 1000000)
         return false;
      g->set_color(187, 187, 187);
      g->set_line_width(2);
   }
   return true;
}

void set_feature_color(ezgl::renderer *g, int feature_id){
   g->set_line_width(1);
   switch(feature_id){
      case UNKNOWN:
      case BUILDING:
         g->set_color(217, 217, 217);
         break;
      case PARK:
      case GREENSPACE:
      case GOLFCOURSE:
         g->set_color(192, 250, 218);
         break;
      case BEACH:
         g->set_color(247, 236, 186);
         break;
      case LAKE:
      case RIVER:
      case STREAM:
      case GLACIER:
         g->set_color(158, 226, 255);
         break;
      default:
         g->set_color(217, 217, 217);
      break;
   }
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

ezgl::point2d latlon_to_point(LatLon position) {
   float x = kEarthRadiusInMeters * kDegreeToRadian * position.longitude() * cos_latavg;
   float y = kEarthRadiusInMeters * kDegreeToRadian * position.latitude();

   return(ezgl::point2d(x,y));
}

struct buttonData findButtonData;
bool setupComplete = false;

std::string processString(std::string inputString) {
   std::string::iterator endPosition = std::remove(inputString.begin(), inputString.end(), ' ');
   inputString.erase(endPosition, inputString.end());
   for (int j = 0; j < inputString.size(); j++) {
      inputString[j] = std::tolower(inputString[j]);
   }
   return inputString;
}

void updateOptions(std::string boxName, std::string streetName, ezgl::application* application) {
   std::vector<StreetIdx> ids = findStreetIdsFromPartialStreetName(streetName);
   std::vector<std::string> options;
   if (ids.size() == 0) {
      options.push_back("No Matches");
   } else if (ids.size() == 1) {
      options.push_back("Name Complete");
   } else {
      options.resize(ids.size());
      for (int i = 0; i < ids.size(); i++) {
         options[i] = getStreetName(ids[i]);
      }
   }
   const char* charVersion = boxName.c_str();
   application->change_combo_box_text_options(charVersion, options);
}

void firstTextEntered(GtkEntry* textBox, buttonData* myStruct) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = processString(text);
   myStruct->string1 = streetString;
   updateOptions("Street1Options", streetString, myStruct->application);
}

void secondTextEntered(GtkEntry* textBox, buttonData* myStruct) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = processString(text);
   myStruct->string2 = streetString;
   updateOptions("Street2Options", streetString, myStruct->application);
}

void menuCallBack1(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {
      GtkComboBoxText* textBox = (GtkComboBoxText*) application->find_widget("Street1Options");
      if (gtk_combo_box_text_get_active_text(textBox)) {
         const gchar* myString = gtk_combo_box_text_get_active_text(textBox);
         if ((strcmp(myString, "Name Complete") != 0) && (strcmp(myString, "No Matches") != 0)) {
            GtkEntry* labelBox = (GtkEntry*) application->find_widget("Street1");
            gtk_entry_set_text(labelBox, myString);
         }
      }
   }
}

void menuCallBack2(GtkComboBoxText* /*box*/, ezgl::application* application) {
   if (setupComplete) {
      GtkComboBoxText* textBox = (GtkComboBoxText*) application->find_widget("Street2Options");
      if (gtk_combo_box_text_get_active_text(textBox)) {
         const gchar* myString = gtk_combo_box_text_get_active_text(textBox);
         if ((strcmp(myString, "Name Complete") != 0) && (strcmp(myString, "No Matches") != 0)) {
            GtkEntry* labelBox = (GtkEntry*) application->find_widget("Street2");
            gtk_entry_set_text(labelBox, myString);
         }
      }
   }
}

void findIntersections(GtkButton* /*button*/, buttonData* myStruct) {
   GtkEntry* streetNameBox1 = (GtkEntry*) myStruct->application->find_widget("Street1");
   GtkEntry* streetNameBox2 = (GtkEntry*) myStruct->application->find_widget("Street2");
   std::string street1 = gtk_entry_get_text(streetNameBox1);
   std::string street2 = gtk_entry_get_text(streetNameBox2);

   // std::string street1 = myStruct->string1;
   // std::string street2 = myStruct->string2;
   myStruct->application->update_message(street1 + street2);

   std::pair<StreetIdx, StreetIdx> streetPair;

   std::vector<StreetIdx> firstResults = findStreetIdsFromPartialStreetName(street1);
   std::vector<StreetIdx> secondResults = findStreetIdsFromPartialStreetName(street2);

   bool anyResult = true;

   if (firstResults.size() == 0) {
      myStruct->application->create_popup_message("Incorrect Street Names", "There were no streets found matching the provided names");
      return;
   } else {
      streetPair.first = firstResults[0];
      anyResult = true;
   }

   if (secondResults.size() == 0) {
      myStruct->application->create_popup_message("Incorrect Street Names", "There were no streets found matching the provided names");
   } else {
      streetPair.second = secondResults[0];
      anyResult = true;
   }

   if (anyResult) {
      std::vector<IntersectionIdx> intersections_ = findIntersectionsOfTwoStreets(streetPair);
      if (intersections_.size() == 0) {
         myStruct->application->create_popup_message("No Intersections Found", "The street names provided do not intersect, check for spelling.");
      } else {
         for (int i = 0; i < intersections_.size(); i++) {
            intersections[intersections_[i]].highlight = true;
         }
         if (intersections_.size() != 0) {
            float x = intersections[intersections_[0]].position.x;
            float y = intersections[intersections_[0]].position.y;
            int zoomBoxSize = 250;
            ezgl::renderer* g = myStruct->application->get_renderer();
            g->set_visible_world(ezgl::rectangle({x-zoomBoxSize, y-zoomBoxSize}, {x+zoomBoxSize, y+zoomBoxSize}));
         }
      }
   }
   myStruct->application->refresh_drawing();
}

void initial_setup(ezgl::application* application, bool /*new_window*/) {
   application->update_message("MAP THING");
   GObject* firstBox = application->get_object("Street1");
   GObject* secondBox = application->get_object("Street2");
   GObject* findButton = application->get_object("FindIntersections");
   std::vector<std::string> startingChoice = {"No Current Options"};
   application->create_combo_box_text("Street1Options", 1, 11, 2, 1, menuCallBack1, startingChoice);
   application->create_combo_box_text("Street2Options", 1, 12, 2, 1, menuCallBack2, startingChoice);

   findButtonData.application = application;
   buttonData* findButtonPointer = &findButtonData;

   g_signal_connect(firstBox, "activate", G_CALLBACK(firstTextEntered), findButtonPointer);
   g_signal_connect(secondBox, "activate", G_CALLBACK(secondTextEntered), findButtonPointer);
   g_signal_connect(findButton, "clicked", G_CALLBACK(findIntersections), findButtonPointer);

   setupComplete = true;
}


void act_on_mouse_click(ezgl::application* app, GdkEventButton* /*event*/, double x, double y) {
   std::cout << "Mouse clicked at (" << x << "," << y << ")\n";

   LatLon position = LatLon(lat_from_y(y), lon_from_x(x));
   int inter_id = findClosestIntersection(position);

   std::stringstream closestIntersection;
   closestIntersection << "Selected: " << intersections[inter_id].name;
   std::cout << "That sucks" << std::endl;
   app->update_message(closestIntersection.str());
   app->refresh_drawing();
}