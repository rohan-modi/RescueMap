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

    deliveries = {DeliveryInf(65840, 60953), DeliveryInf(192916, 99534), DeliveryInf(36488, 138914), DeliveryInf(148548, 110367), DeliveryInf(77172, 47396), DeliveryInf(89630, 118327), DeliveryInf(126486, 90236), DeliveryInf(37154, 154609), DeliveryInf(67973, 69887), DeliveryInf(95910, 24968), DeliveryInf(176009, 8147), DeliveryInf(136111, 122118), DeliveryInf(120031, 190483), DeliveryInf(156388, 176440), DeliveryInf(75435, 37921), DeliveryInf(116111, 11725), DeliveryInf(143601, 111890), DeliveryInf(89878, 70968), DeliveryInf(191194, 176440), DeliveryInf(69451, 175514), DeliveryInf(91736, 154609), DeliveryInf(89630, 79802), DeliveryInf(66414, 4048), DeliveryInf(89630, 154151), DeliveryInf(152136, 147909), DeliveryInf(85741, 4866), DeliveryInf(88126, 8462), DeliveryInf(123063, 192084), DeliveryInf(146044, 184578), DeliveryInf(132458, 100808), DeliveryInf(97936, 79805), DeliveryInf(26853, 84051), DeliveryInf(92161, 58286), DeliveryInf(155451, 172102), DeliveryInf(94839, 92373), DeliveryInf(142821, 48233), DeliveryInf(1766, 167198), DeliveryInf(75435, 65559), DeliveryInf(154278, 33456), DeliveryInf(185509, 124535), DeliveryInf(26213, 176368), DeliveryInf(115384, 70036), DeliveryInf(88898, 84606), DeliveryInf(26853, 40586), DeliveryInf(192116, 140295), DeliveryInf(103046, 10973), DeliveryInf(8684, 154609), DeliveryInf(9164, 159614), DeliveryInf(109207, 154609), DeliveryInf(142589, 54022), DeliveryInf(178453, 150376), DeliveryInf(26853, 64787), DeliveryInf(121086, 98940), DeliveryInf(70437, 120154), DeliveryInf(4855, 152936), DeliveryInf(50504, 176088), DeliveryInf(107781, 37966), DeliveryInf(75435, 173157), DeliveryInf(51638, 59557), DeliveryInf(89630, 80888), DeliveryInf(108952, 39894), DeliveryInf(13661, 91105), DeliveryInf(28755, 188685), DeliveryInf(127699, 135015), DeliveryInf(13523, 168569), DeliveryInf(72427, 85304), DeliveryInf(77800, 177734), DeliveryInf(104445, 49692), DeliveryInf(73852, 132722), DeliveryInf(26853, 14119), DeliveryInf(43512, 36548), DeliveryInf(185420, 132666), DeliveryInf(6981, 74227), DeliveryInf(26853, 98107), DeliveryInf(83584, 178359), DeliveryInf(108143, 30243), DeliveryInf(171998, 103633), DeliveryInf(193659, 16084), DeliveryInf(122531, 46088), DeliveryInf(17118, 121966), DeliveryInf(150225, 65168), DeliveryInf(165379, 2186), DeliveryInf(84731, 176440), DeliveryInf(149528, 176440), DeliveryInf(107211, 42540), DeliveryInf(102297, 132740), DeliveryInf(188661, 157754), DeliveryInf(29383, 57698), DeliveryInf(28222, 9650), DeliveryInf(183669, 154609), DeliveryInf(73522, 83377), DeliveryInf(79547, 174075), DeliveryInf(17370, 47366), DeliveryInf(133019, 134143), DeliveryInf(43505, 134749), DeliveryInf(166674, 136427), DeliveryInf(108525, 148962), DeliveryInf(123845, 134401), DeliveryInf(132616, 176440), DeliveryInf(92308, 74041)};
        depots = {26, 99179, 118214};
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
