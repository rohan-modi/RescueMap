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
#include <string>

#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m4.h"

//Program exit codes
constexpr int SUCCESS_EXIT_CODE = 0;        //Everyting went OK
constexpr int ERROR_EXIT_CODE = 1;          //An error occured
constexpr int BAD_ARGUMENTS_EXIT_CODE = 2;  //Invalid command-line usage

//The default map to load if none is specified
std::string default_map_path = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";

/*
Cities:
beijing_china
boston_usa
cape-town_south-africa
golden-horseshoe_canada
hamilton_canada
hong-kong_china
iceland
interlaken_switzerland
kyiv_ukraine
london_england
new-delhi_india
new-york_usa
rio-de-janeiro_brazil
saint-helena
singapore
sydney_australia
tehran_iran
tokyo_japan
toronto_canada
*/


// The start routine of your program (main) when you are running your standalone
// mapper program. This main routine is *never called* when you are running 
// ece297exercise (the unit tests) -- those tests have their own main routine
// and directly call your functions in /libstreetmap/src/ to test them.
// Don't write any code in this file that you want run by ece297exerise -- it 
// will not be called!
int main(int argc, char** argv) {

    std::string map_path;
    if(argc == 1) {
        //Use a default map
        map_path = default_map_path;
    } else if (argc == 2) {
        //Get the map from the command line
        map_path = argv[1];
    } else {
        //Invalid arguments
        std::cerr << "Usage: " << argv[0] << " [map_file_path]\n";
        std::cerr << "  If no map_file_path is provided a default map is loaded.\n";
        return BAD_ARGUMENTS_EXIT_CODE;
    }

    //Load the map and related data structures
    bool load_success = loadMap(map_path);
    if(!load_success) {
        std::cerr << "Failed to load map '" << map_path << "'\n";
        return ERROR_EXIT_CODE;
    }

    std::cout << "Successfully loaded map '" << map_path << "'\n";

    //You can now do something with the map data
    //drawMap();

    std::vector<DeliveryInf> deliveries;
    std::vector<IntersectionIdx> depots;

    deliveries.push_back(DeliveryInf(342,51666));
    deliveries.push_back(DeliveryInf(5323,23166));
    deliveries.push_back(DeliveryInf(23166,2432));
    depots.push_back(33424);
    depots.push_back(19274);
    int temp1 = 2; 
    int temp2 = 41423;
    
    // for(int i = 0; i < 100; i++){
    //     deliveries.push_back(DeliveryInf(temp1,temp2));
    //     temp1++;
    //     temp2++;
    // }

    travelingCourier(0.0,deliveries,depots);

    std::cout<<"TRAVEL TIME : " << computePathTravelTime(0.0,findPathBetweenIntersections(0.0,{5323,23166}))<<std::endl;

    
    //Clean-up the map data and related data structures
    std::cout << "Closing map\n";
    closeMap(); 

    return SUCCESS_EXIT_CODE;
}
