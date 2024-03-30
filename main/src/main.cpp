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
#include <cmath>
#include <sstream>


#include "m1.h"
#include "m2.h"
#include "m3.h"

std::string getSegmentTravelDirection(IntersectionIdx inter1, IntersectionIdx inter2);
std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2);
std::vector<LatLon>findAngleReferencePoints(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id);
std::string getTravelDirections(const std::vector<StreetSegmentIdx>& path, IntersectionIdx inter_start, IntersectionIdx inter_finish);
std::string getRoundedDistance(double distance);
void replaceUnknown(std::string &input);



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

    // TESTING PURPOSES
    std::cout << "Street 1: " << getStreetName(getStreetSegmentInfo(114846).streetID) << std::endl;
    std::cout << "Intersection from: " << getIntersectionName(getStreetSegmentInfo(114846).from) << std::endl;
    std::cout << "Intersection to: " << getIntersectionName(getStreetSegmentInfo(114846).to) << std::endl;

    std::cout << "Street 2: " << getStreetName(getStreetSegmentInfo(69910).streetID) << std::endl;
    std::cout << "Intersection from: " << getIntersectionName(getStreetSegmentInfo(69910).from) << std::endl;
    std::cout << "Intersection to: " << getIntersectionName(getStreetSegmentInfo(69910).to) << std::endl;

    std::cout << "Street A: " << getStreetName(getStreetSegmentInfo(108263).streetID) << std::endl;
    std::cout << "Intersection from: " << getIntersectionName(getStreetSegmentInfo(108263).from) << std::endl;
    std::cout << "Intersection to: " << getIntersectionName(getStreetSegmentInfo(108263).to) << std::endl;

    std::cout << "Street B: " << getStreetName(getStreetSegmentInfo(15).streetID) << std::endl;
    std::cout << "Intersection from: " << getIntersectionName(getStreetSegmentInfo(15).from) << std::endl;
    std::cout << "Intersection to: " << getIntersectionName(getStreetSegmentInfo(15).to) << std::endl;

    std::cout <<getSegmentTravelDirection(getStreetSegmentInfo(114846).from, getStreetSegmentInfo(114846).to) << std::endl;
    std::cout <<getSegmentTravelDirection(getStreetSegmentInfo(69910).from, getStreetSegmentInfo(69910).to) << std::endl;

    std::cout << getStreetSegmentInfo(247965).from << " " << getStreetSegmentInfo(247965).to << std::endl;
    std::cout << getStreetSegmentInfo(247964).from << " " << getStreetSegmentInfo(247964).to << std::endl;
    std::cout << getStreetSegmentInfo(120386).from << " " << getStreetSegmentInfo(120386).to << std::endl;
    std::cout << getStreetSegmentInfo(120387).from << " " << getStreetSegmentInfo(120387).to << std::endl;

    //std::vector<StreetSegmentIdx> path = {47882, 20830, 20829, 20828, 20827, 20826, 20825, 20824, 20823, 20822, 20821, 20820, 20819, 222552, 222553, 222554, 175404, 222557, 54698, 54699, 54697, 169406, 170144, 170145, 169282, 169285, 169286, 169284, 169408, 169409, 90311, 90310, 143591, 143590, 143586, 143597, 143594, 167285, 169377, 169378, 169382, 169383, 169379, 169380, 167284, 214201, 214202, 16};
    //std::vector<StreetSegmentIdx> path = {114846, 69910, 69924, 69912, 837, 836, 835, 68390, 68391, 186132, 68377, 155416, 68374, 68373, 68378, 186133, 186130, 68397, 189750, 189749, 189751, 186139, 186138, 186137, 186136, 186135, 186134, 16183, 16182, 16181, 16180, 16179, 16178, 16177, 185637, 185638, 16184, 185639, 132433, 185635, 16190, 116823, 23373, 23374, 116808, 106076, 249806, 119680, 9153, 116795, 116794, 116796, 106091, 116797, 28653, 28654, 240608, 28666, 28667, 6347, 6348, 145466, 145471, 145469, 85333, 145464, 85337, 85338, 85339, 85360, 85361, 118839, 118840, 39177, 39178, 39179, 39180, 39181, 267831, 267830, 39191, 15914, 9457, 9458, 9459, 128289, 0, 172130, 172131, 222122, 15926, 8187, 250651, 108263, 15};
    //std::vector<StreetSegmentIdx> path = {67341, 67342, 67320, 67331, 67327, 67325, 67324, 64697, 197040, 197039, 197041, 103842, 103843, 158290, 158289, 67526, 67525, 67524, 67514, 67515, 233266, 67522, 189755, 189754, 189753, 189752, 189747, 222771, 222770, 222769, 222768, 222767, 222766, 222765, 222764, 20019, 221624, 221629, 221628, 221627, 221626, 221625, 16176, 150013, 150016, 150017, 161792, 161793, 161794, 159071, 68814, 68813, 68815, 189363, 132431, 132432, 16188, 16191, 132426, 116822, 116823, 23373, 23374, 116808, 106076, 249806, 119680, 9153, 116795, 116794, 116796, 106091, 116797, 28653, 28654, 240608, 28666, 28667, 6347, 6348, 145466, 145471, 145469, 85333, 145464, 85337, 85338, 85339, 85360, 85361, 118839, 118840, 39177, 39178, 39179, 39180, 39181, 267831, 267830, 39191, 15914, 9457, 9458, 9459, 128289};
    std::vector<StreetSegmentIdx> path = {247965, 247964, 247963, 247962, 247961, 247960, 247959, 247958, 53180, 53179, 53178, 53177, 53176, 53175, 53174, 49442, 49846, 140299, 101772, 101771, 101767, 101764, 101765, 101760, 101761, 257443, 153886, 153885, 153884, 156987, 156986, 156985, 156984, 156983, 156982, 101766, 101776, 189798, 189799, 101884, 101883, 101881, 101879, 101878, 101854, 101869, 257440, 101852, 101853, 101839, 101870, 101873, 101874, 101847, 101845, 221305, 101871, 101857, 101876, 101875, 101856, 130559, 130563, 130562, 151615, 151614, 151613, 151612, 184683, 184682, 151616, 77199, 77198, 77194, 77196, 218811, 12762, 123537, 123480, 77318, 230872, 77317, 77321, 77322, 230873, 230874, 215522, 77310, 77309, 77311, 126517, 126518, 126519, 126520, 167463, 167464, 153856, 153857, 153858, 104108, 104109, 104110, 104111, 103547, 103548, 103549, 103550, 278000, 59405, 59408, 59406, 59421, 59423, 59425, 103553, 103545, 190705, 137472, 190708, 190726, 190727, 190725, 183691, 183692, 183693, 183694, 172962, 144069, 144070, 191394, 191393, 172959, 172960, 172961, 137457, 184051, 137461, 137462, 137458, 137459, 137460, 137463, 137464, 137465, 168774, 191389, 168775, 168772, 190734, 190735, 190736, 190737, 190733, 190740, 190741, 190742, 190739, 190738, 196139, 196140, 196141, 196142, 196143, 196144, 196145, 196146, 196147, 196148, 196149, 196150, 196151, 196152, 184066, 161845, 161842, 14302, 225891, 181021, 160119, 160120, 225830, 160121, 277999, 224466, 147709, 147712, 147713, 39762, 161987, 39763, 14291, 14292, 14293, 14294, 14295, 14296, 14297, 14298, 14299, 14300, 14301, 89582, 141287, 127947, 277995, 141290, 127945, 140298, 168645, 168644, 168643, 168642, 168641, 168640, 168639, 168646, 117182, 117181, 117186, 117187, 117188, 117183, 117184, 117185, 117189, 117190, 117191, 117192, 117193, 118475, 168653, 118472, 118474, 9495, 2694, 46231, 39753, 39754, 39755, 73641, 168546, 168547, 73644, 73645, 141458, 46246, 46241, 46242, 46243, 46244, 194141, 46251, 167888, 167889, 167890, 194139, 46252, 193422, 193423, 193424, 113704, 113705, 118816, 118815, 118814, 246467, 248837, 248836, 248835, 248834, 248833, 248832, 248838, 117146, 117145, 117144, 117143, 117142, 117141, 117140, 117139, 117138, 117137, 117136, 196258, 196257, 196256, 196255, 254457, 196262, 196261, 196260, 196259, 194074, 194073, 87124, 117135, 117134, 248510, 101298, 101297, 101296, 101295, 851, 20411, 189050, 150622, 150621, 150620, 150619, 87370, 87369, 87368, 87367, 13514, 13513, 16202, 138791, 138792, 221190, 221189, 188983, 185689, 115593, 115637, 185688, 115634, 246431, 120386, 120387};
    //std::vector<StreetSegmentIdx> path = {249145, 222067, 75119, 75118, 249534, 249441, 215609, 162987, 162986, 162985, 162954, 162949, 162950, 162951, 162952, 162953, 223528, 223529, 76832, 76834, 163948, 139085, 76823, 76824, 76825, 76826, 76827, 241739, 241737, 241738, 184637, 36195, 36196, 36197, 168674, 168675, 115948, 115949, 115945, 115947, 241734, 241735, 241736, 241733, 115939, 115940, 115941, 115942, 115943, 115944, 254231, 254232, 254233, 73715, 73712, 73716, 73717, 73718, 73719, 73720, 73705, 73706, 73710, 73709, 73708, 73700, 73702, 193100, 26270, 26269, 26268, 26267, 26266, 26265, 26264, 26263, 27235, 74251, 74249, 254104, 254105, 250195, 250193, 250194, 250192, 36218, 36219, 234172, 159845, 27233, 27234, 250191, 145026, 160933, 160932, 160931, 160930, 72956, 250256, 250257, 250254, 250253, 161811, 161810, 161809, 161808, 250252, 235063, 235062, 250251, 250255, 250249, 250250, 129704, 234289, 234290, 254278, 254277, 254276, 254275, 234287, 234286, 234285, 234283, 235059, 248326, 127363, 248328, 248329, 248323, 248321, 248322, 248318, 179801, 179802, 179803, 179804, 179805, 127360, 127361, 158670, 158671, 158669, 159778, 159780, 215682, 231305, 231306, 159777, 189829, 36233, 36234, 36235, 221430, 221429, 1977, 1978, 127274, 127273, 221424, 231300, 127272, 250171, 250174, 250175, 127297, 127298, 127299, 127301, 127300, 214271, 127304, 127306, 127307, 127308, 189697, 256446, 18613, 250168, 189685, 18611, 77413, 250166, 217782, 129326, 129327, 129328, 231295, 231291, 231288, 36251, 36252, 231284, 231285, 231286, 231287, 231279, 231280, 144296, 144297, 161213, 125599, 125600, 125601, 231276, 223051, 223052, 146414, 139140, 125602, 118199, 36272, 36273, 160898, 160900, 250164, 161387, 161388, 161389, 36275, 36276, 130330, 130331, 130329, 118200, 256443, 7748, 35543, 36283, 189663, 36285, 36288, 7753, 7754, 151602, 151604, 36292, 247473, 190643, 247478, 133226, 133227, 151452, 151453, 151450, 151451, 151577, 189527, 151576, 256389, 160896, 160897, 160894, 160895, 250450, 250451, 250452, 161804, 126799, 36317, 36318, 36319, 24380, 100059, 168870, 168869, 99922, 99923, 251889, 251890, 251891, 100028, 100029, 99899, 99900, 99901, 102709, 102710, 99971, 99905, 221466, 221467, 99966, 99967, 221470, 99870, 99871, 221468, 100022, 91642, 231318, 151440, 151441, 231315, 231316, 107469, 107470, 252036, 36346, 36347, 252033, 252032, 232395, 12547, 12548, 252029, 129359, 129360, 155021, 36369, 36370, 36371, 36372, 36373, 183454, 273006, 273007, 155022, 252025, 272998, 272996, 272997, 100979, 100980, 100981, 100982, 234208, 273111, 273107, 273108, 103364, 252023, 249829, 249830, 249831, 249832, 249833, 114671, 273079, 273078, 251886, 251887, 252019, 114662, 273034, 273035, 252016, 252017, 252018, 114672, 273036, 273037, 252015, 251885, 185580, 185581, 273077, 273080, 103361, 103362, 103363, 252020, 252021, 252022, 249834, 249835, 249836, 249837, 252024, 273106, 273112, 273113, 44913, 44914, 192054, 192055, 272992, 273002, 273003, 252026, 252027, 252028, 273005, 46017, 46018, 45691, 273021, 100144, 173326, 173325, 100131, 100111, 100158, 100138, 249784, 148256, 100108, 100153, 100133, 159534, 100151, 100113, 66575, 66574, 66677, 66428, 66067, 154874, 154875, 154876, 180863, 66702, 66347, 66369, 66368, 66367, 66133, 66132, 66131, 66221, 66071, 66070, 66069, 66099, 66098, 66732, 66558, 66016, 66076, 66695, 80957, 66771, 65697, 65696, 65695, 65694, 65693, 65692, 65691, 65688, 65687, 182522, 65669, 65668, 65667, 65666, 65665, 65690, 65689, 65703, 65702, 65701, 65700, 182519, 182520, 65699, 65698, 65673, 65672, 182518, 182517, 65686, 65685, 182516, 65677, 65676, 65684, 65682, 65679, 65678, 105700, 105699, 105698, 105697, 105696, 105695, 105694, 105693, 105692, 105691, 105690, 105689, 105688, 105687, 105686, 105685, 176245, 17642, 17638, 31296, 234924, 30151, 30152, 17639, 17636, 251694, 17630, 116831, 249854, 28295, 28296, 116826, 28285, 28286, 28281, 28282, 150725, 150726, 106075, 116814, 23392, 23393, 116816, 116815, 6332, 132425, 132426, 116822, 116823, 23373, 23374, 116808, 106076, 249806, 119680, 9153, 116795, 116794, 116796, 106091, 116797, 28653, 28654, 240608, 28666, 28667, 6347, 6348, 145466, 145471, 145469, 85333, 145464, 85337, 85338, 85339, 85360, 85361, 118839, 118840, 39177, 39178, 39179, 39180, 39181, 267831, 267830, 39191, 15914, 9457, 9458, 9459, 128289, 0, 172130, 172131, 15927};
    //std::vector<StreetSegmentIdx> path = {164543, 138342, 138341, 138340, 138339, 138338, 138337, 138336, 138335, 138334, 138333, 138332, 176899, 176898, 176901, 176902, 176903, 120086, 120085, 120084, 120083, 120082, 120081, 120078, 120077, 120076, 24050, 190305, 190297, 119479, 24051, 24052, 24053, 24054, 226126, 24055, 24056, 24057, 24058, 24059, 24049, 190304, 186876, 186877, 216621, 216622, 184103, 184102, 153146, 190309, 255994, 248808, 248809, 248810, 248811, 256334, 256333, 150865, 176056, 119481, 138732, 222973, 222974, 119590, 119591, 222987, 222988, 180864, 180865, 189382, 5705, 128641, 160890, 188661, 67694, 67693, 67692, 188660, 188652, 272442, 272439, 172808, 188715, 272448, 272447, 188717, 188716, 188658, 188659, 188657, 188656, 272426, 272425, 188654, 188653, 188655, 30181, 272431, 272432, 132589, 144147, 144146, 272429, 272428, 113179, 113178, 188694, 188701, 188699, 188702, 188703, 188700, 188698, 30182, 70141, 70138, 141328, 70128, 70134, 16171, 16170, 16169, 16168, 221631, 221632, 16162, 141326, 190264, 137383, 137384, 137385, 137382, 36559, 170056, 133107, 133109, 133110, 16166, 16165, 16163, 141336, 89979, 89978, 141338, 190268, 190267, 16449, 190259, 190263, 190262, 190261, 16447, 16448, 190260, 190265, 141339, 35217, 35216, 190273, 190272, 190271, 190270, 190269, 222348, 190266, 89071, 89070, 89069, 89068, 89067, 89066, 89065, 89064, 89063, 89062, 197110, 197115, 197114, 197113, 197112, 197111, 190555, 142448, 229053, 229055, 229054, 121737, 197101, 219239, 219238, 219237, 183805, 183804, 191826, 191825, 87135, 87134, 190545, 190551, 121736, 190554, 190548, 190547, 226603, 226602, 190556, 190552, 196780, 191204, 191203, 197059, 197058, 217791, 142254, 87683, 87685, 190550, 127996, 190544, 54962, 54961, 156094, 87669, 87675, 87674, 87673, 87672, 87671, 87670, 159588, 159587, 159586, 159585, 159584, 159583, 190553, 190549, 190546, 118282, 138792, 138791, 16202, 13513, 13514, 87367, 87368, 87369, 87370, 150619, 150620, 150621, 150622, 189050, 20411, 851, 101295, 101296, 101297, 101298, 248510, 117134, 117135, 87124, 194073, 194074, 196259, 196260, 196261, 196262, 254457, 196255, 196256, 196257, 196258, 117136, 117137, 117138, 117139, 117140, 117141, 117142, 117143, 117144, 117145, 117146, 248838, 248832, 248833, 248834, 248835, 248836, 248837, 246467, 118814, 118815, 118816, 246465, 246466, 142630, 142631, 142632, 142633, 118809, 118810, 118811, 118812, 118813, 117219, 117220, 117221, 29147, 142636, 142637, 101290, 101291, 101292, 101288, 101289, 228890, 233409, 29138, 170055, 150409, 150408, 103143, 103141, 22052, 22053, 157303, 157305, 157304, 42417, 142562, 183506, 183507, 142563, 136374, 136378, 187013, 166004, 166005, 146216, 146217, 117400, 15939, 15940, 131146, 3940, 15942, 131144, 43099, 43100, 43101, 131145, 131140, 131141, 131142, 131143, 137193, 137194, 138192, 138173, 138174, 172656, 172657, 143238, 138179, 138180, 138181, 278152, 145187, 138434, 138435, 138436, 138433, 158531, 158532, 158533, 158387, 29628, 158362, 158363, 158364, 158365, 158366, 157072, 157073, 157074, 157075, 157076, 104355, 104356, 153056, 153058, 153057, 153065, 153066, 153067, 168326, 168328, 247831, 247832, 72247, 72248, 72249, 72250, 72251, 72241, 170128, 72245, 72201, 72202, 72203, 72204, 72205, 72206, 72207, 72208, 72238, 105706, 72239, 276686, 88951, 72220, 72221, 72222, 72223, 72224, 72225, 72226, 72227, 147698, 145188, 72209, 72210, 72211, 72212, 72213, 72214, 72215, 72216, 72217, 72218, 72219, 72097, 72098, 72101, 72102, 72103, 72104, 72105, 72106, 72127, 72119, 72120, 168953, 168954, 72123, 72117, 72118, 72133, 72134, 72135, 72136, 72137, 72138, 72139, 72140, 72141, 72142, 72143, 72144, 72145, 72146, 72147, 72148, 72149, 72150, 72151, 88585, 146680, 146681, 72019, 72023, 72041, 72024, 72020, 72042, 72014, 72013, 72046, 72047, 72048, 72049, 72050, 72051, 72052, 72053, 72054, 72055, 72056, 72057, 72058, 72059, 72060, 72061, 72062, 72009, 72006, 72003, 72007, 71999, 161753, 161752, 161755, 161756, 161757, 161758, 161754, 17204, 17201, 17202, 17207, 17208, 88417, 88416, 88418, 36049, 36048, 157952, 157951, 157953, 60348, 60347, 60346, 60345, 60333, 60322, 60323, 60324, 260718, 260719, 260716, 260717, 167852, 60332, 60328, 3765, 3764, 3763, 3762, 3761, 3760, 3759, 3758, 3757, 3756, 3755, 3754, 3753, 3752, 3751, 3750, 3749, 3748, 3747, 3746, 3745, 255366, 255364, 255363, 255362, 255361, 255360, 255359, 255358, 255357, 255356, 255355, 3734, 277860, 116401, 116402, 185716, 225878, 72297, 72298, 72299, 183365, 183366, 255243, 255245, 108927, 108928, 108929, 183367, 183368, 183369, 72295, 277859, 72309, 72310, 153606, 221947, 221948, 221949, 13963, 157940, 144800, 154004, 72287, 3740, 141599, 141598, 131283, 86750, 6143, 141313};

    std::cout << "SEGMENTS: " << "\n";
    for (int i = 0; i < 30; i++) {
        std::cout << i << " : " << "Segment ID is " << path[i] << "\n";
        std::cout << getStreetSegmentInfo(path[i]).from << " " << getStreetName(getStreetSegmentInfo(path[i]).streetID);
        std::cout << ", " << getStreetSegmentInfo(path[i]).to << " " << getStreetName(getStreetSegmentInfo(path[i]).streetID) << std::endl;

    }

    std::cout << findStreetSegmentLength(101761) << "\n";;
    std::cout << findStreetSegmentLength(257443) << "\n";;
    

    /*
        WB 16th Ave: 101761
        from: 93043
        to: 93044

        SB Bayview Ave: 257443
        from: 93044
        to: 129391
    */

    std::cout << getIntersectionTurningDirection(101761, 257443) << "\n";
    
    std::cout << getTravelDirections(path, 72804, 32) << "\n";


    // END TESTING PURPOSES


    //You can now do something with the map data
    //drawMap();

    //Clean-up the map data and related data structures
    std::cout << "Closing map\n";
    closeMap(); 

    return SUCCESS_EXIT_CODE;
}


// This function returns a string containing detailed travel directions for a driver.
// This function calls several helper functions that return strings.
// Instructions are of the following general forms:
// -- Start at starting intersection
// -- Turn onto a new street, continue in a direction for a given distance
// -- Arrive at destination intersection
// Input receives a vector of street segments to form a path, starting intersection,
// destination intersection. Assume a valid path and valid intersection endpoints.
// Written by Jonathan
std::string getTravelDirections(const std::vector<StreetSegmentIdx>& path, IntersectionIdx inter_start, IntersectionIdx inter_finish) {

    // Declare stringstream to contain all travel directions
    std::stringstream directions;
    
    // Starting intersection
    directions << "Start at " << getIntersectionName(inter_start) << "\n";

    // Declare previous street segment and current street segment
    StreetSegmentInfo prevSegment = getStreetSegmentInfo(path[0]);
    StreetSegmentInfo currSegment = getStreetSegmentInfo(path[0]);

    // Declare previous intersection and next intersection (relative to current street segment)
    IntersectionIdx prevInter = inter_start;
    IntersectionIdx nextInter;
    if (currSegment.from == inter_start) {
        nextInter = currSegment.to;
        directions << "Head " << getSegmentTravelDirection(currSegment.from, currSegment.to);
    } else {
        nextInter = currSegment.from;
        directions << "Head " << getSegmentTravelDirection(currSegment.to, currSegment.from);
    }
    directions << " on " << getStreetName(currSegment.streetID) << "\n";
    
    // Declare and initialize total trip distance
    double totalDistance = 0.0;

    // Loop through all street segments in the given path
    for (int pathIdx = 1; pathIdx < path.size(); pathIdx++) {
        
        prevSegment = currSegment;
        currSegment = getStreetSegmentInfo(path[pathIdx]);

        prevInter = nextInter;
        if (currSegment.from == prevInter) {
            nextInter = currSegment.to;
        } else {
            nextInter = currSegment.from;
        }

        // Declare distance variable to record distance travelled along one street
        // Initialize to the distance of the first street segment
        double distance = findStreetSegmentLength(path[pathIdx]);
        
        // Determine turning action based on angle between street segments
        double angle = findAngleBetweenStreetSegments(path[pathIdx - 1], path[pathIdx]);

        // Continue straight if angle is less than 5 degrees
        if (angle < 5 * kDegreeToRadian) {
            directions << "Continue straight";
        }
        // Slight turn if angle is between 5 degrees and 70 degrees
        else if ((angle > 5 * kDegreeToRadian) && (angle < 70 * kDegreeToRadian)) {
            directions << "Take a slight " << getIntersectionTurningDirection(path[pathIdx - 1], path[pathIdx]);
        }
        // Sharp turn if angle is greater than 110 degrees
        else if (angle > 110 * kDegreeToRadian) {
            directions << "Take a sharp " << getIntersectionTurningDirection(path[pathIdx - 1], path[pathIdx]);
        }
        // Or else, apply a regular turn
        else {
            directions << "Turn " << getIntersectionTurningDirection(path[pathIdx - 1], path[pathIdx]);
        }
        directions << " onto " << getStreetName(currSegment.streetID) << ", ";
        directions << "continue " << getSegmentTravelDirection(prevInter, nextInter) << " for ";

        // Look ahead to the next street segment to determine its streetID.
        // While it is the same streetID, keep skipping the next iteration of street segment.
        // In other words, if consecutive street segments are part of the same street, only
        // output 1 instruction for the travel directions.
        while ((pathIdx + 1 < path.size()) && (getStreetSegmentInfo(path[pathIdx + 1]).streetID == currSegment.streetID)) {
            pathIdx = pathIdx + 1;

            prevSegment = currSegment;
            currSegment = getStreetSegmentInfo(path[pathIdx]);

            prevInter = nextInter;
            if (currSegment.from == prevInter) {
                nextInter = currSegment.to;
            } else {
                nextInter = currSegment.from;
            }

            // Accumulate the total distance across all street segments
            distance += findStreetSegmentLength(path[pathIdx]);
        }
        
        // Output the distance travelled along one street
        directions << getRoundedDistance(distance) << "\n";
        
        // Accumulate the total distance for the trip
        totalDistance += distance;
    }    

    // Destination intersection
    directions << "Arrive at " << getIntersectionName(inter_finish) << "\n";

    // Report total trip distance
    directions << "Total trip distance: " << getRoundedDistance(totalDistance) << "\n";
    
    // Report total trip time
    directions << "Total trip time: " << "<XXX[PLACEHOLDER]XXX>" << "\n";

    // Replace <unknown> and return travel directions as a string
    std::string output = directions.str();
    replaceUnknown(output);
    return output;
}

// Determines the direction of travel given 2 intersection endpoints of a street segment.
// Returns north, south, east, or west.
// Direction of travel is assumed to be inter1 -> inter2.
// Written by Jonathan
std::string getSegmentTravelDirection(IntersectionIdx inter1, IntersectionIdx inter2) {
    LatLon src = getIntersectionPosition(inter1);
    LatLon dest = getIntersectionPosition(inter2);

    // To find the distance between two points (lon1, lat1) and (lon2, lat2),
    // it is accurate to compute lat_avg = (lat1 + lat2) / 2 [rad]
    double lat_avg = kDegreeToRadian * (src.latitude() + dest.latitude()) / 2;

    // Compute x-coordinates for src and dest
    double x1 = kEarthRadiusInMeters * kDegreeToRadian * src.longitude() * cos(lat_avg);
    double x2 = kEarthRadiusInMeters * kDegreeToRadian * dest.longitude() * cos(lat_avg);

    // Compute y-coordinates for src and dest
    double y1 = kEarthRadiusInMeters * kDegreeToRadian * src.latitude();
    double y2 = kEarthRadiusInMeters * kDegreeToRadian * dest.latitude();
    
    // Compute inverse tangent to determine angle
    double angle = atan((y2 - y1) / (x2 - x1));

    // If x-component is negative, add pi
    if ((x2 - x1) < 0) {
        angle += M_PI;
    }

    // Declare direction string
    std::string direction;

    // Determine direction (North, South, East, West)
    // Direction boundaries are split at 90 degree intervals along 45 degree diagonals
    if ((angle > -(kDegreeToRadian * 45)) && (angle < kDegreeToRadian * 45)) {
        direction = "east";
    } else if ((angle > kDegreeToRadian * 45) && (angle < kDegreeToRadian * 135)) {
        direction = "north";
    } else if ((angle > kDegreeToRadian * 135) && (angle < kDegreeToRadian * 225)) {
        direction = "west";
    } else {
        direction = "south";
    }

    return direction;
}


// Determines the direction of turn given 2 street segments directly connected at
// an intersection. Returns left or right.
// This function assumes that segment1 and segment2 are connected such that one
// can drive legally by exiting segment1 and entering segment2.
// Written by Jonathan
std::string getIntersectionTurningDirection(StreetSegmentIdx segment1, StreetSegmentIdx segment2) {
    
    std::vector<LatLon> referencePoints = findAngleReferencePoints(segment1, segment2);
    LatLon shared = referencePoints[0];
    LatLon point1 = referencePoints[1];
    LatLon point2 = referencePoints[2];
    
    //double lat_avg = kDegreeToRadian * (shared.latitude() + point1.latitude() + point2.latitude()) / 3;
    double lat_avg1 = kDegreeToRadian * (shared.latitude() + point1.latitude()) / 2;
    double lat_avg2 = kDegreeToRadian * (shared.latitude() + point2.latitude()) / 2;

    //double lat_avg1 = kDegreeToRadian * (shared.latitude() + point1.latitude() + point2.latitude()) / 3;
    //double lat_avg2 = lat_avg1;

    // Compute x-coordinates
    double x_shared1 = kEarthRadiusInMeters * kDegreeToRadian * shared.longitude() * cos(lat_avg1);
    double x_1 = kEarthRadiusInMeters * kDegreeToRadian * point1.longitude() * cos(lat_avg1);

    double x_shared2 = kEarthRadiusInMeters * kDegreeToRadian * shared.longitude() * cos(lat_avg2);
    double x_2 = kEarthRadiusInMeters * kDegreeToRadian * point2.longitude() * cos(lat_avg2);

    // Compute y-coordinates for src and dest
    double y_shared = kEarthRadiusInMeters * kDegreeToRadian * shared.latitude();
    double y_1 = kEarthRadiusInMeters * kDegreeToRadian * point1.latitude();
    double y_2 = kEarthRadiusInMeters * kDegreeToRadian * point2.latitude();
    
    // Compute cross product in z-direction: (Ax * By) - (Ay * Bx)
    double Ax = x_shared1 - x_1;
    double Ay = y_shared - y_1;

    double Bx = x_2 - x_shared2;
    double By = y_2 - y_shared;

    double crossProduct = (Ax * By) - (Ay * Bx);
    
    // By the right-hand rule:
    // If the z-component is positive, then the direction is left
    // If the z-component is negative, then the direction is right
    std::string direction;

    if (crossProduct < 0) {
        direction = "right";
    } else {
        direction = "left";
    }

    return direction;
}


// Determines the 3 sets of reference point coordinates used to determine the
// intersection turning direction.
// Returns the 3 sets of reference points as a vector of 3 LatLon objects.
// This function assumes that src and dst are connected such that one
// can drive legally by exiting src and entering dst.
// Written by Jonathan
std::vector<LatLon>findAngleReferencePoints(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_segment_id) {

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

    return {shared_point, point_1, point_2};
}


// Distance is given in [m]
// Round the given distance depending on magnitude, return in string form
// Written by Jonathan
std::string getRoundedDistance(double distance) {
    const int KILOMETER_TO_METER = 1000;

    std::stringstream roundedDistance;

    // Any value below 10 m is rounded to 10 m
    if (distance < 0.01 * KILOMETER_TO_METER) {
        distance = 10;
        roundedDistance << distance << " m";        
    }
    // Any value less than 1 km is rounded to the nearest 10 m
    else if (distance < KILOMETER_TO_METER) {
        distance = std::round(distance / 10) * 10;
        roundedDistance << distance << " m";
    }
    // Any value between 1 km and 100 km is rounded to the nearest 0.1 km
    else if (distance < 100 * KILOMETER_TO_METER) {
        distance = std::round(distance / KILOMETER_TO_METER * 10) / 10;
        roundedDistance << distance << " km";
    }
    // Any value greater than 1 km is rounded to the nearest 1 km
    else {
        distance = std::round(distance / KILOMETER_TO_METER);
        roundedDistance << distance << " km";
    }
    
    // Return the rounded distance as a string
    return roundedDistance.str();
}


// Takes in a string input passed by reference.
// Replaces all instances of "<unknown>" with "Unnamed Road" for usability considerations.
// Written by Jonathan
void replaceUnknown(std::string &input) {
    
    // Find substring
    std::string str_find = "<unknown>";

    // Replacement substring
    std::string str_replace = "Unnamed Road";

    // Search for the first instance of "<unknown>"
    std::size_t pos = input.find(str_find);

    // Traverse the entire input string and replace all instances
    while (pos != std::string::npos) {
        
        // Replace first instance of "<unknown>" with "Unnamed Road"
        input.replace(pos, str_find.size(), str_replace);

        // Search for the next instance of "<unknown>"
        pos = input.find(str_find, pos + str_replace.size());
    }
}
