
#include <iostream>
#include <stdlib.h>
#include <random>

#include "BoardGrid.h"

/*********************************************************************************
This is a maze router program

This program was written by Devon Merrill (devon@ucsd.edu)

Dijkstra's algorithm has been implemented between two points

TODO:
    1. multiple two pin net routing
        a. add cost to routed net features (done)
        b. display multiple nets (doneish -- can use output in EAGLE file)
    2. multipin net routing
        a. multi-goal graph search
        b. change functions to accept goal lists
            i. better location hashing? bitmap for wires?
    3. ripup and reroute
        a. remove cost when net is ripped up (done)
        b. reroute loop (done)
        c. display / record cost over time
        d. shadow cost
            i. design rule checker
            ii. design rule cost balancing
            iii. design rule spec file format
            iv. design rule spec file interface
    4. cost profiles for routing and vias etc. (done badly)
        a. easy way to have different net classes
        b. easy way to have different via costs that can be changed online
    5. Interface with EAGLE files 
        a. input (Connie is working on this)
        b. output (partially done -- output net wire tags)
            i. vias
            ii. integrate with board file (waiting for Connie)




*********************************************************************************/

void test_two_pin_routing()
{
    std::cout << "test_two_pin_routing starting\n";

    // height, width, layers
    const unsigned int h = 31;
    const unsigned int w = 31;
    const unsigned int l = 2;
    const float grid_factor = 1.0;
    const int max_ripups = 20000;

    std::vector<Route> nets{
        Route(Location(0, 0, 0), Location(w - 1, h - 1, 0)),
        Route(Location(0, h - 1, 0), Location(w - 1, 0, 0)),
        Route(Location(0, h / 2, 0), Location(w - 1, h / 2, 0)),
        Route(Location(w / 2, 0, 0), Location(w / 2, h - 1, 0)),
        Route(Location(0, h / 4, 0), Location(w - 1, h / 4, 0)),
        Route(Location(0, 3 * h / 4, 0), Location(w - 1, 3 * h / 4, 0)),
        Route(Location(w / 4, h - 1, 0), Location(w / 4, 0, 0)),
        Route(Location(3 * w / 4, h - 1, 0), Location(3 * w / 4, 0, 0))};

    BoardGrid bg(w, h, l);
    bg.base_cost_fill(0.0);

    std::cout << "Initial routing" << std::endl;
    // initial route
    for (int i = 0; i < nets.size(); i += 1)
    {
        bg.add_route(nets[i]);
        // std::cout << "Printing route" << std::endl;
        // bg.print_route(nets[i].came_from, nets[i].end);
        // std::cout << "Printing route done" << std::endl;
    }

    for (Route route : nets)
    {
        for (Location l : route.features)
        {
            if (l.x > bg.w || l.y > bg.h || l.z > bg.l)
            {
                std::cout << "Bad route initial: " << l << std::endl;
                exit(-1);
            }
        }
    }

    // ripup loop
    std::cout << "Doing ripups" << std::endl;
    std::random_device rd;                                 // obtain a random number from hardware
    std::mt19937 eng(rd());                                // seed the generator
    std::uniform_int_distribution<> distr(0, nets.size()); // define the range
    for (int i = 0; i < max_ripups; i += 1)
    {
        int rand_index = distr(eng);
        bg.ripup_route(nets[rand_index]);
        bg.add_route(nets[rand_index]);
    }

    std::cout << "Printing costs" << std::endl;
    bg.pprint();
    std::cout << "Printing costs done" << std::endl;

    for (int r = 0; r < nets.size(); r += 1)
    {
        // std::cout << std::endl;
        std::cout << "<signal name=\"" << char('A' + r) << "\">" << std::endl;
        Location last_location = nets[r].features[0];

        for (int i = 1; i < nets[r].features.size(); i += 1)
        {
            int layer = nets[r].features[i].z;
            if (layer == 0)
            {
                layer = 1;
            }
            else if (layer == 1)
            {
                layer = 16;
            }

            std::cout << "\t"
                      << "<wire ";
            std::cout << "x1=\"" << grid_factor * last_location.x << "\" y1=\"" << grid_factor * last_location.y << "\" ";
            std::cout << "x2=\"" << grid_factor * nets[r].features[i].x << "\" y2=\"" << grid_factor * nets[r].features[i].y << "\" ";
            std::cout << "width=\"" << 0.6096 << "\" layer=\"" << layer << "\"";
            std::cout << "/>";
            std::cout << std::endl;

            last_location = nets[r].features[i];
        }
        std::cout << "</signal>";
        std::cout << std::endl;
    }

    std::cout << "test_two_pin_routing exiting\n";
}

void test_multipin()
{
    std::cout << "test_two_pin_routing starting\n";

    // height, width, layers
    const unsigned int h = 30;
    const unsigned int w = 30;
    const unsigned int l = 2;
    const float grid_factor = 1.0;
    const int max_ripups = 20000;

    BoardGrid bg(w, h, l);
    bg.base_cost_fill(0.0);

    std::cout << "Initial routing" << std::endl;

    //Skip doing multipin net
    /*
    std::vector<Location> pins = {
        Location(0, 0, 0),
        Location(w-1, h/2, 0),
        Location(w/2, h-1, 0),
        Location(w-1, h-1, 0),
        Location(w/2, h/2, 1)
    };

    MultipinRoute mp(pins);

    // Add net and route immediately
    bg.add_route(mp);
    */

    // five 2-pins nets
    std::vector<Route> nets{
        Route(Location(w / 2, 0, 0), Location(0, h - 1, 0)),
        Route(Location(0, h / 4, 0), Location(w - 1, h / 4, 0)),
        Route(Location(0, 3 * h / 4, 0), Location(w - 1, 3 * h / 4, 0)),
        Route(Location(w / 4, h - 1, 0), Location(w / 4, 0, 0)),
        Route(Location(3 * w / 4, h - 1, 0), Location(3 * w / 4, 0, 0))};

    for (int i = 0; i < nets.size(); i += 1)
    {
        // Add net and route immediately
        bg.add_route(nets[i]);
        std::cout << std::endl
                  << "============Printing route " << i << " ============" << std::endl;
        std::cout << "Start: " << nets.at(i).start << ", End: " << nets.at(i).end << std::endl;
        bg.print_route(nets[i].came_from, nets[i].end);
        std::cout << std::endl
                  << "============Printing route " << i << " done============" << std::endl;
    }

    std::cout << std::endl
              << "============Printing final costs============" << std::endl;
    bg.pprint();
    std::cout << std::endl
              << "============Printing final costs done============" << std::endl;

    /*
    
    // Print EAGLE file format to show the wires
    // 2-pins
    for (int r = 0; r < nets.size(); r += 1) {
        // std::cout << std::endl;
        std::cout << "<signal name=\"" << char ('A' + r) <<"\">" << std::endl;
        Location last_location = nets[r].features[0];

        for (int i = 1; i < nets[r].features.size(); i += 1) {
            int layer = nets[r].features[i].z;
            if (layer == 0) {
                layer = 1;
            }
            else if (layer == 1) {
                layer = 16;
            }

            std::cout << "\t" << "<wire ";
            std::cout << "x1=\"" << grid_factor * last_location.x << "\" y1=\"" << grid_factor * last_location.y << "\" ";
            std::cout << "x2=\"" << grid_factor * nets[r].features[i].x << "\" y2=\"" << grid_factor * nets[r].features[i].y << "\" ";
            std::cout << "width=\"" << 0.6096 << "\" layer=\"" << layer << "\"";
            std::cout << "/>";
            std::cout << std::endl;

            last_location = nets[r].features[i];
        }
        std::cout << "</signal>";
        std::cout << std::endl;
    }

    // Print EAGLE file format to show the wires
    // Multipin net
    std::cout << "<signal name=\"" << "MP" <<"\">" << std::endl;
    Location last_location = mp.features[0];
    for (int i = 1; i < mp.features.size(); i += 1) {
        int layer = mp.features[i].z;

        // check if near
        if (
            abs(mp.features[i].x - last_location.x) <= 1 &&
            abs(mp.features[i].y - last_location.y) <= 1 &&
            abs(mp.features[i].z - last_location.z) <= 1
        ){
            if (layer == 0) {
                layer = 1;
            }
            else if (layer == 1) {
                layer = 16;
            }

            std::cout << "\t" << "<wire ";
            std::cout << "x1=\"" << grid_factor * last_location.x << "\" y1=\"" << grid_factor * last_location.y << "\" ";
            std::cout << "x2=\"" << grid_factor * mp.features[i].x << "\" y2=\"" << grid_factor * mp.features[i].y << "\" ";
            std::cout << "width=\"" << 0.6096 << "\" layer=\"" << layer << "\"";
            std::cout << "/>";
            std::cout << std::endl;
        }

        last_location = mp.features[i];
    }
    std::cout << "</signal>" << std::endl;
    // Multipin net finished

    std::cout << "test_two_pin_routing exiting\n";

    */
}

void mixed_test()
{
    std::cout << "mixed_test starting\n";

    // height, width, layers
    const unsigned int h = 10;
    const unsigned int w = 10;
    const unsigned int l = 2;
    const float grid_factor = 1.0;
    const int max_ripups = 20000;

    BoardGrid bg(w, h, l);
    bg.base_cost_fill(0.0);

    std::cout << "Initial routing" << std::endl;

    std::vector<Location> pins = {
        Location(0, 0, 0),
        Location(9, 5, 0),
        Location(5, 9, 0),
        Location(9, 9, 0),
        Location(4, 4, 1)};

    MultipinRoute mp(pins);
    bg.add_route(mp);

    std::cout << "mixed_test exiting\n";
}

/*
int main() {
    test_multipin();
    return 0;
}
*/