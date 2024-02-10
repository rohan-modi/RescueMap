
#include "m1.h"
#include "unit_test_util.h"

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

#include <UnitTest++/UnitTest++.h>

#include <random>
#include <algorithm>
#include <set>

using ece297test::relative_error;
using ece297test::sorted;

SUITE(street_queries_public_toronto_canada) {

    struct BaseMapFixture {
        BaseMapFixture() {
            //Load the map
            try {
                loadMap("/cad2/ece297s/public/maps/toronto_canada.streets.bin");
            } catch (...) {
                std::cout << "!!!! BaseMapFixture test setup: loadMap threw an exception !!!!" << std::endl;
                throw; // re-throw exception
            }
        }
    
        ~BaseMapFixture() {
            //Clean-up
            try {
                closeMap();
            } catch (const std::exception& e) {
                std::cout << "!!!! BaseMapFixture test teardown: closeMap threw an exception. what(): " << e.what() << " !!!!" << std::endl;
                std::terminate(); // we're in a destructor
            } catch (...) {
                std::cout << "!!!! BaseMapFixture test teardown: closeMap threw an exception !!!!" << std::endl;
                std::terminate(); // we're in a destructor
            }
        }
    };


    struct MapFixture : BaseMapFixture {};

    TEST_FIXTURE(MapFixture, all_street_intersections) {
        std::vector<IntersectionIdx> expected;

        expected = {2, 3, 10, 745, 746, 749, 750, 751, 752, 755, 756, 759, 760, 763, 764, 772, 773, 774, 776, 777, 781, 1917, 1934, 7470, 10465, 10472, 10473, 10597, 10760, 10761, 10768, 10769, 10807, 10808, 14060, 14067, 14072, 15743, 15744, 15745, 15786, 15787, 15904, 15905, 15906, 17904, 17905, 20440, 21452, 25925, 25948, 25949, 31104, 31105, 31106, 31117, 31118, 31119, 31437, 31438, 31439, 31448, 31449, 41767, 41768, 62961, 62962, 63317, 63318, 63331, 80044, 80058, 80059, 80060, 80073, 81003, 81004, 81508, 81509, 81514, 81517, 81518, 81525, 98166, 98415, 98424, 104527, 104528, 121595};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(2)));

        expected = {12182, 24011, 24034};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(5219)));

        expected = {18866, 18867, 117097, 117098};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(4044)));

        expected = {20392, 20393, 20394, 20397};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(4392)));

        expected = {21808, 170437, 170439};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(22301)));

        expected = {26977, 26978, 26979};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(5774)));

        expected = {28316, 28317, 28318};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(6081)));

        expected = {29890, 29891, 35415, 35416, 35465, 39199, 39201, 39218, 86431, 86432, 86433, 86434, 86435, 86436, 86437, 86438, 86439, 86457, 86458, 86459, 86460, 86461, 86462, 86463, 86464, 86465, 86466, 86467, 86480, 86481, 86482, 86483, 103471, 103472, 103473, 103474, 103475, 113313, 119360, 119362, 119365, 119366, 120226, 131661, 136847, 136848, 142221, 142222, 142223, 142224};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(6296)));

        expected = {31941, 106425, 106426, 106427, 106428, 106429, 106430};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(20023)));

        expected = {33680, 77421, 77437};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(15949)));

        expected = {47154, 64964};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(12881)));

        expected = {52142, 53958};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(9704)));

        expected = {53287, 62356, 62357, 62358, 62363};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(12113)));

        expected = {58851, 58873};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(11185)));

        expected = {63016, 63030};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(12334)));

        expected = {66025, 66026, 66036};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(12992)));

        expected = {71160, 71161, 71163};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(14258)));

        expected = {75108, 75152};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(15275)));

        expected = {75576, 75599};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(15418)));

        expected = {77033, 77034, 77040, 77042, 77045};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(15810)));

        expected = {78715, 78716, 78718, 78719, 78720, 78723, 78731, 78732, 78738, 78740, 103190};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(16309)));

        expected = {80671, 80673, 80674, 80675, 80676, 80677, 80678, 80679, 80683, 80685, 80688, 80693, 80780};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(16901)));

        expected = {87865, 87866};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(17559)));

        expected = {88804, 88805, 88806, 88807, 88808, 88809, 88810};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(17852)));

        expected = {89280, 89694};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(18200)));

        expected = {99516, 115090, 115092};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(20459)));

        expected = {104834, 104835, 192022};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(19966)));

        expected = {108753, 108754};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(20174)));

        expected = {112225, 112235};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(20367)));

        expected = {134336, 142366};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfStreet(21707)));

    } //all_street_intersections

    TEST_FIXTURE(MapFixture, intersection_ids_from_street_ids) {
        std::vector<IntersectionIdx> expected;

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(4281, 6514))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(6343, 4530))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(7364, 2533))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(7712, 13851))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(8982, 17636))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(10247, 17317))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(11275, 3834))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(11600, 9105))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(12961, 11231))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(13442, 18367))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(15617, 18451))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(16683, 506))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(19062, 17273))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(19619, 407))));

        expected = {};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(19820, 22010))));

        expected = {2, 3, 10, 746, 750, 759, 773, 776, 1917, 1934, 7470, 10465, 14060, 14067, 14072, 20440, 21452, 31119, 81518};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(2, 0))));

        expected = {7675, 7676};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(1527, 0))));

        expected = {12297};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(19242, 2594))));

        expected = {33680, 77437};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(15949, 0))));

        expected = {36788};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(7165, 16721))));

        expected = {46676};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(8315, 8172))));

        expected = {78738, 78740};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(16309, 16320))));

        expected = {79395, 79403};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(16550, 16543))));

        expected = {80674, 80675, 80677, 80678, 80693, 80780};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(16901, 0))));

        expected = {87881};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(4867, 17568))));

        expected = {88805};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(17852, 17888))));

        expected = {89484, 89683};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(18077, 18156))));

        expected = {108753};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(20174, 20176))));

        expected = {115090};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(20459, 19373))));

        expected = {127129, 127130, 127131, 127132};
        ECE297_CHECK_EQUAL(expected, sorted(findIntersectionsOfTwoStreets(std::make_pair(20945, 0))));

    } //intersection_ids_from_street_ids

} //street_queries_public_toronto_canada

