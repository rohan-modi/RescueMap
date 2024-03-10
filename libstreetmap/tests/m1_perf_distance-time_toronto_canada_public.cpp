
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

SUITE(distance_time_queries_perf_public_toronto_canada) {

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


    struct MapFixture : BaseMapFixture {
        MapFixture()
            : BaseMapFixture()
            , rng(3)
            , rand_intersection(0, getNumIntersections()-1)
            , rand_street(1, getNumStreets()-1) // Start from 1 to avoid getting id 0 (<unknown>)
            , rand_segment(0, getNumStreetSegments()-1)
            , rand_poi(0, getNumPointsOfInterest()-1)
            , rand_feature(0, getNumFeatures()-1)
            , rand_node(0, 8951918)
            , rand_way(0, 1192953)
            , rand_relation(0, 8223)
            , rand_lat(43.479999542, 43.919982910)
            , rand_lon(-79.789985657, -79.000000000)
        { }

        std::minstd_rand rng;
        std::uniform_int_distribution<int> rand_intersection;
        std::uniform_int_distribution<int> rand_street;
        std::uniform_int_distribution<int> rand_segment;
        std::uniform_int_distribution<int> rand_poi;
        std::uniform_int_distribution<int> rand_feature;
        std::uniform_int_distribution<int> rand_node;
        std::uniform_int_distribution<int> rand_way;
        std::uniform_int_distribution<int> rand_relation;
        std::uniform_real_distribution<double> rand_lat;
        std::uniform_real_distribution<double> rand_lon;
    };

    

    TEST_FIXTURE(MapFixture, street_segment_travel_time_perf) {
        //Verify Functionality
        double expected;

        expected = 0.33234691132153937;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(158504), 0.001000000);

        expected = 0.64589127667330948;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(248257), 0.001000000);

        expected = 0.75777867216591310;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(74825), 0.001000000);

        expected = 0.88378038087254807;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(216073), 0.001000000);

        expected = 1.21160613960374652;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(246389), 0.001000000);

        expected = 1.26511396895948303;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(187962), 0.001000000);

        expected = 1.28142095937572953;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(251756), 0.001000000);

        expected = 1.33581825123845532;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(159866), 0.001000000);

        expected = 1.43560750974394202;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(267111), 0.001000000);

        expected = 1.45891096986915780;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(189727), 0.001000000);

        expected = 2.30735213185562582;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(207976), 0.001000000);

        expected = 2.44804013960716471;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(119406), 0.001000000);

        expected = 2.59632571621061903;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(274424), 0.001000000);

        expected = 2.95934909336146257;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(137633), 0.001000000);

        expected = 2.98433587753430318;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(219683), 0.001000000);

        expected = 3.10857985751341470;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(175447), 0.001000000);

        expected = 3.12531574127367717;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(77474), 0.001000000);

        expected = 3.93001933164743589;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(196254), 0.001000000);

        expected = 4.11624687274447787;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(200691), 0.001000000);

        expected = 4.76260626451067370;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(54042), 0.001000000);

        expected = 4.79480918771576459;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(194552), 0.001000000);

        expected = 4.94644871789605567;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(18), 0.001000000);

        expected = 5.16017159941526948;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(151779), 0.001000000);

        expected = 5.37033943959530813;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(64212), 0.001000000);

        expected = 6.49915614876212366;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(223956), 0.001000000);

        expected = 7.72869240930012946;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(149053), 0.001000000);

        expected = 8.10276508443466170;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(250632), 0.001000000);

        expected = 9.32035338763629539;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(71052), 0.001000000);

        expected = 15.88499341890157801;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(49755), 0.001000000);

        expected = 15.98696900886002226;
        ECE297_CHECK_RELATIVE_ERROR(expected, findStreetSegmentTravelTime(245687), 0.001000000);

        //Generate random inputs
        std::vector<StreetSegmentIdx> segment_ids;
        for(size_t i = 0; i < 250000000; i++) {
            segment_ids.push_back(rand_segment(rng));
        }
        {
            //Timed Test
            ECE297_TIME_CONSTRAINT(1116);
            double result;
            for(size_t i = 0; i < 250000000; i++) {
                result += findStreetSegmentTravelTime(segment_ids[i]);
            }
        }
    } //street_segment_travel_time_perf

   
} //distance_time_queries_perf_public_toronto_canada

