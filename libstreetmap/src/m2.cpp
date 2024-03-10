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

extern std::vector<Intersection_data> intersections;
extern std::vector<std::vector<int>> streetSegments;
extern std::vector<std::pair<std::string, int>> streetNamesAndIDs;
extern std::vector<std::vector<int>> streetSegments;
extern Map_bounds mapBounds;
extern std::vector<segment_data> street_segments;
double viewPortArea;
extern float cos_latavg;
extern std::vector<name_data> streetNames;

extern std::vector<closed_feature_data> closedFeatures;
extern std::vector<line_feature_data> lineFeatures;
extern std::vector<feature_data> features;
extern std::vector<ezgl::point2d> POIlocations;
int line_width; 

extern std::vector<std::string> mapNames;
bool darkMode;

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
int findDistanceBetweenTwoPointsxy(ezgl::point2d point_1, ezgl::point2d point_2);
gboolean change_dark_switch(GtkSwitch* /*switch*/, gboolean switch_state, ezgl::application* application);
void fillMapDropDown(ezgl::application* application);
double findAngle360(ezgl::point2d point_1, ezgl::point2d point_2);
void drawPOIs(ezgl::renderer *g);

float x_from_lon(float lon);
float y_from_lat(float lat);

float lon_from_x(float x);
float lat_from_y(float y);

POIIdx findClickablePOI(LatLon my_position);


ezgl::rectangle world;

struct buttonData {
   std::string string1;
   std::string string2;
   ezgl::application* application;
   // GObject* infoBox;
};

void initial_setup(ezgl::application *application, bool new_window);
void firstTextEntered(GtkEntry* textBox, ezgl::application* application);
void secondTextEntered(GtkEntry* textBox, ezgl::application* application);
void findIntersections(GtkButton* button, ezgl::application* application);
std::string processString(std::string inputString);
void menuCallBack1(GtkComboBoxText* /*box*/, ezgl::application* application);
void menuCallBack2(GtkComboBoxText* /*box*/, ezgl::application* application);
void map_selection_changed(GtkComboBoxText* /*box*/, ezgl::application* application);
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

void setWorldScale(ezgl::renderer *g);
void draw_main_canvas (ezgl::renderer *g) {

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
   if(viewPortArea < 200000)
      drawPOIs(g);
}

void setWorldScale(ezgl::renderer *g){
   double factor = g->get_visible_screen().width()/g->get_visible_world().width();
   line_width = ((factor+0.13)*10);
   if(line_width > 11){
      line_width = 11;
   }
}

void drawPOIs(ezgl::renderer *g){
   g->set_color(ezgl::RED);
   std::cout<<"dar"<<std::endl;
   for(int POI_index = 0; POI_index < POIlocations.size(); POI_index++){
      g->fill_arc(POIlocations[POI_index], 1, 0, 360);
      
   }
}

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
   std::cout << "draw_intersections took " << wallClock.count() << " seconds" << std::endl;
}

bool checkContains(double maxx, double minx, double maxy, double miny);

double findAngle360(ezgl::point2d point_1, ezgl::point2d point_2){
   double x, y;
      x = point_2.x - point_1.x;
      y = point_2.y - point_1.y;
   double radian = std::atan2(y, x);

   double degrees = radian *(180/M_PI);
   if(degrees< 0)
      degrees += 360;

   return degrees;
}


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
   for(int i = 0; i < streetNames.size(); i++){
      if(world.contains(streetNames[i].position)){
         g->set_color(ezgl::BLACK);
         g->set_font_size(9);
         g->set_text_rotation(streetNames[i].angle);
         g->draw_text(streetNames[i].position, streetNames[i].name);
      }
   }   

   // if(street_segments[segment_id].oneWay&&viewPortArea < 250000){
   //                g->set_color(ezgl::BLACK);
   //                g->set_font_size(9);
   //                g->set_text_rotation(findAngle360(street_segments[segment_id].points[point_index-1],street_segments[segment_id].points[point_index]));
   //                g->draw_text(street_segments[segment_id].points[point_index-1], ">");
   //             }

   auto currTime = std::chrono::high_resolution_clock::now();
   auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - startTime);
   std::cout << "draw_streets took " << wallClock.count() <<" seconds" << std::endl;
}

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
   std::cout << "draw_features took " << wallClock.count() <<" seconds" << std::endl;
}

bool checkContains(double maxx, double minx, double maxy, double miny){
   ezgl::point2d bottom_right = world.bottom_right();
   ezgl::point2d top_left = world.top_left();
   if(world.contains(maxx, miny)||world.contains(minx, maxy)||world.contains(maxx, miny)||world.contains(minx, maxy)){
      return true;
   }
   if(maxx > bottom_right.x && minx < top_left.x ){
      return true;
   }else if (miny < bottom_right.y && maxy > top_left.y ){
      return true;
   }

   if(maxx < bottom_right.x && maxx > top_left.x ){
      return true;
   }else if (miny > bottom_right.y && miny < top_left.y ){
      return true;
   }

   if(minx < bottom_right.x && minx > top_left.x ){
      return true;
   }else if (maxy > bottom_right.y && maxy < top_left.y ){
      return true;
   }
   
   return false;
}


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

bool setupComplete = false;

void updateOptions(std::string boxName, std::string streetName, ezgl::application* application) {
   std::vector<StreetIdx> ids = findStreetIdsFromPartialStreetName(streetName);
   std::vector<std::string> options;
   if (ids.size() == 0) {
      options.push_back("No Matches");
   } else {
      options.resize(ids.size());
      for (int i = 0; i < ids.size(); i++) {
         options[i] = getStreetName(ids[i]);
      }
      auto lastUniqueElement = std::unique(options.begin(), options.end());
      options.erase(lastUniqueElement, options.end());
   }
   const char* charVersion = boxName.c_str();
   GtkComboBoxText* textBoxText = (GtkComboBoxText*) application->find_widget(charVersion);
   gtk_combo_box_text_remove_all(textBoxText);
   for (int i = 0; i < options.size(); i++) {
      const char* optionCharVersion = (options[i]).c_str();
      gtk_combo_box_text_append_text(textBoxText, optionCharVersion);
   }
   GtkComboBox* textBox = (GtkComboBox*) application->find_widget(charVersion);
   gtk_combo_box_popup(textBox);
}

void firstTextEntered(GtkEntry* textBox, ezgl::application* application) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = text;
   updateOptions("Street1Options", streetString, application);
}

void secondTextEntered(GtkEntry* textBox, ezgl::application* application) {
   const gchar* text = gtk_entry_get_text(textBox);
   std::string streetString = text;
   updateOptions("Street2Options", streetString, application);
}

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

void findIntersections(GtkButton* /*button*/, ezgl::application* application) {
   GtkEntry* streetNameBox1 = (GtkEntry*) application->find_widget("Street1");
   GtkEntry* streetNameBox2 = (GtkEntry*) application->find_widget("Street2");
   std::string street1 = gtk_entry_get_text(streetNameBox1);
   std::string street2 = gtk_entry_get_text(streetNameBox2);

   application->update_message(street1 + street2);

   std::pair<StreetIdx, StreetIdx> streetPair;

   std::vector<StreetIdx> firstResults = findStreetIdsFromPartialStreetName(street1);
   std::vector<StreetIdx> secondResults = findStreetIdsFromPartialStreetName(street2);

   if (firstResults.size() == 0 || secondResults.size() == 0) {
      application->create_popup_message("Incorrect Street Names", "There were no streets found matching the provided names");
      return;
   }

   for (int i = 0; i < firstResults.size(); i++) {
      for (int j = 0; j < secondResults.size(); j++) {
         streetPair.first = firstResults[i];
         streetPair.second = secondResults[j];
         std::vector<IntersectionIdx> intersections_ = findIntersectionsOfTwoStreets(streetPair);
         if (intersections_.size() != 0) {
            std::string foundStreet1 = getStreetName(streetPair.first);
            std::string foundStreet2 = getStreetName(streetPair.second);
            const gchar* charVersionStreet1 = foundStreet1.c_str();
            const gchar* charVersionStreet2 = foundStreet2.c_str();
            for (int k = 0; k < intersections_.size(); k++) {
               intersections[intersections_[k]].highlight = true;

               // Output intersection name information
               std::stringstream closestIntersection;
               closestIntersection << "Selected: " << intersections[intersections_[k]].name;
               application->update_message(closestIntersection.str());
            }
            float x = intersections[intersections_[0]].position.x;
            float y = intersections[intersections_[0]].position.y;
            int zoomBoxSize = 250;
            ezgl::renderer* g = application->get_renderer();
            g->set_visible_world(ezgl::rectangle({x-zoomBoxSize, y-zoomBoxSize}, {x+zoomBoxSize, y+zoomBoxSize}));            
            gtk_entry_set_text(streetNameBox1, charVersionStreet1);
            gtk_entry_set_text(streetNameBox2, charVersionStreet2);
            application->refresh_drawing();
            return;
         }
      }
   }
   application->create_popup_message("No Intersections Found", "The provided streets do not intersect.");
}

void initial_setup(ezgl::application* application, bool /*new_window*/) {
   application->update_message("MAP THING");

   GObject* firstBox = application->get_object("Street1");
   GObject* secondBox = application->get_object("Street2");
   GObject* findButton = application->get_object("FindIntersections");
   GObject* dropDown1 = application->get_object("Street1Options");
   GObject* dropDown2 = application->get_object("Street2Options");
   GObject* dropDown3 = application->get_object("MapSelection");
   GObject* darkSwitch = application->get_object("DarkMode");

   g_signal_connect(firstBox, "activate", G_CALLBACK(firstTextEntered), application);
   g_signal_connect(secondBox, "activate", G_CALLBACK(secondTextEntered), application);
   g_signal_connect(findButton, "clicked", G_CALLBACK(findIntersections), application);
   g_signal_connect(darkSwitch, "state-set", G_CALLBACK(change_dark_switch), application);
   g_signal_connect(dropDown1, "changed", G_CALLBACK(menuCallBack1), application);
   g_signal_connect(dropDown2, "changed", G_CALLBACK(menuCallBack2), application);
   g_signal_connect(dropDown3, "changed", G_CALLBACK(map_selection_changed), application);

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
   const float POI_CLICK_PROXIMITY = 2;

   std::cout << "Mouse clicked at (" << x << "," << y << ")\n";

   // Convert mouse click coordinates (xy) to LatLon to determine closest intersection
   LatLon position = LatLon(lat_from_y(y), lon_from_x(x));
   int inter_id = findClosestIntersection(position);
   int poi_id = findClickablePOI(position);

   // Only highlight and display intersection if clicked within close proximity
   if (findDistanceBetweenTwoPoints(position, getIntersectionPosition(inter_id)) < INTERSECTION_CLICK_PROXIMITY) {
      // Change the visibility of the selected intersection
      intersections[inter_id].highlight = !intersections[inter_id].highlight;

      // Output intersection name information
      std::stringstream closestIntersection;
      closestIntersection << "Selected: " << intersections[inter_id].name;
      app->update_message(closestIntersection.str());

      // Refresh map drawing
      app->refresh_drawing();
   }

   // Only create POI pop-up if clicked within close proximity
   else if (findDistanceBetweenTwoPoints(position, getPOIPosition(poi_id)) < POI_CLICK_PROXIMITY) {
      // Output intersection name information
      std::stringstream closestPOI;
      closestPOI << getPOIName(poi_id);
      app->create_popup_message("POI Selected", closestPOI.str().c_str());

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

void fillMapDropDown(ezgl::application* application) {
   const char* charVersion = "MapSelection";
   GtkComboBoxText* menu = (GtkComboBoxText*) application->find_widget(charVersion);
   for (int i = 0; i < mapNames.size(); i++) {
      const char* mapNameCharVersion = (mapNames[i]).c_str();
      gtk_combo_box_text_append_text(menu, mapNameCharVersion);
   }
}

// Loops through all POIs and calls findDistanceBetweenTwoPoints to check its distance
// Tracks the closest POI by POIIdx and tracks the smallest distance
// Writen by Jonathan
POIIdx findClickablePOI(LatLon my_position) {
    POIIdx closestPOI = 0;
    double minDistance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(0));

    for (int POI_idx = 0; POI_idx < getNumPointsOfInterest(); POI_idx++) {
        if (findDistanceBetweenTwoPoints(my_position, getPOIPosition(POI_idx)) < minDistance) {
            minDistance = findDistanceBetweenTwoPoints(my_position, getPOIPosition(POI_idx));
            closestPOI = POI_idx;
        }
    }
    return closestPOI;
}


