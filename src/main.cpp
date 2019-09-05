#include "kicadPcbDataBase.h"
#include "BoardGrid.h"
#include <limits>

extern void test_multipin();
void test_router(kicadPcbDataBase& db);

int main(int argc, char *argv[])
{

  std::string designName = argv[1];
  
  //auto db = kicadPcbDataBase{designName};
  kicadPcbDataBase db(designName);

  db.printComp();
  db.printInst();
  db.printNet();
  db.printPcbRouterInfo();

  //test_multipin();
  test_router(db);

  return 0;
}

void test_router(kicadPcbDataBase& db){
  std::vector<std::set<std::pair<double, double>>> routerInfo;
  db.getPcbRouterInfo(&routerInfo);
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::min();

  std::cout<<std::fixed<<std::setprecision(5);
  std::cout<<"=================test_router=================="<<std::endl;

  for(int i=0; i<routerInfo.size(); ++i){
    std::cout<<"netId: "<<i<<std::endl;

    for(auto ite : routerInfo.at(i)){
      std::cout<<" ("<<ite.first<<", "<<ite.second<<")";
      min_x = std::min(ite.first,min_x);
      max_x = std::max(ite.first,max_x);
      min_y = std::min(ite.second,min_y);
      max_y = std::max(ite.second,max_y);
    }
    std::cout<<std::endl;
  }

  std::cout<<"Routing Outline: ("<<min_x<<", "<<min_y<<"), ("<<max_x<<", "<<max_y<<")"<<std::endl;

  // Initialize board grid
  const unsigned int inputScale = 10;
  const unsigned int enlargeBoundary = 10;
  const unsigned int h = int(std::abs(max_y*inputScale-min_y*inputScale)) + enlargeBoundary;
  const unsigned int w = int(std::abs(max_x*inputScale-min_x*inputScale)) + enlargeBoundary;
  // BM*
  //const unsigned int l = 2;
  // BBBW
  const unsigned int l = 4;
  const float grid_factor = 0.1;//For outputing
  const int max_ripups = 20000;
  std::vector<Route> twoPinNets;
  std::vector<MultipinRoute> multipinNets;
  std::ofstream ofs ("routerOutput", std::ofstream::out);

  ofs<<std::fixed<<std::setprecision(5);

  std::cout<<"BoardGrid Size: w:"<<w<<", h:"<<h<<", l:"<<l<<std::endl;

  BoardGrid bg(w,h,l);
  bg.base_cost_fill(0.0);

  // Prepare all the nets to route
  for(int i=0; i<routerInfo.size(); ++i){
    std::cout<<"Routing netId: "<<i<<"..."<<std::endl;
    size_t netDegree = routerInfo.at(i).size();
    if(netDegree < 2) continue;

    std::vector<Location> pins;
    for(auto ite : routerInfo.at(i)){
      std::cout<<"\t("<<ite.first<<", "<<ite.second<<")";
      pins.push_back(Location(ite.first*inputScale-min_x*inputScale+enlargeBoundary/2, ite.second*inputScale-min_y*inputScale+enlargeBoundary/2, 0));
      std::cout<<" location in grid: "<<pins.back()<<std::endl; 
    }

    if(netDegree == 2){
      //Route route = Route(pins.at(0), pins.at(1));
      twoPinNets.push_back(Route(pins.at(0), pins.at(1), i));
      bg.add_route(twoPinNets.back());
    } else {
      //continue;
      //MultipinRoute mpRoute(pins);
      multipinNets.push_back(MultipinRoute(pins, i));
      bg.add_route(multipinNets.back());
    }
    std::cout<<"Routing netId: "<<i<<"...done!"<<std::endl;
  }

  // Print KiCad file format
  // 2-pins
  for (int r = 0; r < twoPinNets.size(); r += 1) {
      Location last_location = twoPinNets[r].features[0];

      for (int i = 1; i < twoPinNets[r].features.size(); i += 1) {
          std::string layer;
          if (twoPinNets[r].features[i].z == 0) {
            layer = "Top";
          } else if (twoPinNets[r].features[i].z == 1) {
            layer = "Route3";
          } else if (twoPinNets[r].features[i].z == 2) {
            layer = "Route14";
          } else if (twoPinNets[r].features[i].z == 3) {
            layer = "Bottom";
          } else {
            layer = "Top";
          }

          // Via
          // TODO check via layer order
          // TODO feature will have two consequences same points...
          /* Eg.   
          (via (at 153.9341 96.7756) (size 0.8001) (drill 0.3937) (layers Top Bottom) (net 18) (tstamp 384ED00))
          (via blind (at 153.9341 95.7756) (size 0.8001) (drill 0.3937) (layers In1.Cu Bottom) (net 18) (tstamp 384ED00))
          (via micro (at 153.9341 94.7756) (size 0.8001) (drill 0.3937) (layers In1.Cu In2.Cu) (net 18) (tstamp 384ED00))
          */
          if (twoPinNets[r].features[i].z != last_location.z) {
            std::cout << "(via";
            std::cout << " (at " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
            // BM2
            //std::cout << " (size 0.8001)";
            //std::cout << " (drill 0.3937)";
            // BBBW
            std::cout << " (size 0.6604)";
            std::cout << " (drill 0.4064)";
            std::cout << " (layers Top Bottom)";
            std::cout << " (net " << twoPinNets[r].netId << ")";
            std::cout << ")" << std::endl;

            ofs << "(via";
            ofs << " (at " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
            // BM2
            //ofs << " (size 0.8001)";
            //ofs << " (drill 0.3937)";
            // BBBW
            ofs << " (size 0.6604)";
            ofs << " (drill 0.4064)";
            ofs << " (layers Top Bottom)";
            ofs << " (net " << twoPinNets[r].netId << ")";
            ofs << ")" << std::endl;
          }

          if (twoPinNets[r].features[i].x != last_location.x || twoPinNets[r].features[i].y != last_location.y) {
            // Wire/Tack/Segment
            std::cout << "(segment";
            std::cout << " (start " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
            std::cout << " (end " << grid_factor*(twoPinNets[r].features[i].x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(twoPinNets[r].features[i].y+min_y*inputScale-enlargeBoundary/2) << ")";
            // BM2
            //std::cout << " (width 0.2032)";
            // BBBW
            std::cout << " (width 0.1524)";
            std::cout << " (layer " << layer << ")";
            std::cout << " (net " << twoPinNets[r].netId << ")";
            std::cout << ")" << std::endl;

            ofs << "(segment";
            ofs << " (start " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
            ofs << " (end " << grid_factor*(twoPinNets[r].features[i].x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(twoPinNets[r].features[i].y+min_y*inputScale-enlargeBoundary/2) << ")";
            // BM2
            //ofs << " (width 0.2032)";
            // BBBW
            ofs << " (width 0.1524)";
            ofs << " (layer " << layer << ")";
            ofs << " (net " << twoPinNets[r].netId << ")";
            ofs << ")" << std::endl;
          }

          last_location = twoPinNets[r].features[i];
      }
  }

  // Multipin net
  for(int mpr = 0; mpr<multipinNets.size(); ++mpr){
    Location last_location = multipinNets[mpr].features[0];
    for (int i = 1; i < multipinNets[mpr].features.size(); i += 1) {
      // check if near ??????????
      if (
        abs(multipinNets[mpr].features[i].x - last_location.x) <= 1 &&
        abs(multipinNets[mpr].features[i].y - last_location.y) <= 1 &&
        abs(multipinNets[mpr].features[i].z - last_location.z) <= 1
      ){
        std::string layer;
        if (multipinNets[mpr].features[i].z == 0) {
          layer = "Top";
        } else if (multipinNets[mpr].features[i].z == 1) {
          // BBBW
          layer = "Route3";
        } else if (multipinNets[mpr].features[i].z == 2) {
          // BBBW
          layer = "Route14";
        } else if (multipinNets[mpr].features[i].z == 3) {
          layer = "Bottom";
        } else {
          layer = "Top";
        }

        // Print Via 
        if (multipinNets[mpr].features[i].z != last_location.z) {
          std::cout << "(via";
          std::cout << " (at " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
          // BM2
          //std::cout << " (size 0.8001)";
          //std::cout << " (drill 0.3937)";
          // BBBW
          std::cout << " (size 0.6604)";
          std::cout << " (drill 0.4064)";
          std::cout << " (layers Top Bottom)";
          std::cout << " (net " << multipinNets[mpr].netId << ")";
          std::cout << ")" << std::endl;

          ofs << "(via";
          ofs << " (at " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
          // BM2
          //ofs << " (size 0.8001)";
          //ofs << " (drill 0.3937)";
          // BBBW
          ofs << " (size 0.6604)";
          ofs << " (drill 0.4064)";
          ofs << " (layers Top Bottom)";
          ofs << " (net " << multipinNets[mpr].netId << ")";
          ofs << ")" << std::endl;
        }

        // Print Segment/Track/Wire
        if (multipinNets[mpr].features[i].x != last_location.x || multipinNets[mpr].features[i].y != last_location.y) {
          std::cout << "(segment";
          std::cout << " (start " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
          std::cout << " (end " << grid_factor*(multipinNets[mpr].features[i].x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(multipinNets[mpr].features[i].y+min_y*inputScale-enlargeBoundary/2) << ")";
          // BM2
          //std::cout << " (width 0.2032)";
          // BBBW
          std::cout << " (width 0.1524)";
          std::cout << " (layer " << layer << ")";
          std::cout << " (net " << multipinNets[mpr].netId << ")";
          std::cout << ")" <<std::endl;

          ofs << "(segment";
          ofs << " (start " << grid_factor*(last_location.x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(last_location.y+min_y*inputScale-enlargeBoundary/2) << ")";
          ofs << " (end " << grid_factor*(multipinNets[mpr].features[i].x+min_x*inputScale-enlargeBoundary/2) << " " << grid_factor*(multipinNets[mpr].features[i].y+min_y*inputScale-enlargeBoundary/2) << ")";
          // BM2
          //ofs << " (width 0.2032)";
          // BBBW
          ofs << " (width 0.1524)";
          ofs << " (layer " << layer << ")";
          ofs << " (net " << multipinNets[mpr].netId << ")";
          ofs << ")" <<std::endl;
        }
      }
      last_location = multipinNets[mpr].features[i];
    }
  }

  ofs.close();

  /*
  // Print EAGLE file format to show the wires
  // 2-pins
  for (int r = 0; r < twoPinNets.size(); r += 1) {
      // std::cout << std::endl;
      std::cout << "<signal name=\"" << "2P" << r <<"\">" << std::endl;
      Location last_location = twoPinNets[r].features[0];

      for (int i = 1; i < twoPinNets[r].features.size(); i += 1) {
          int layer = twoPinNets[r].features[i].z;
          if (layer == 0) {
              layer = 1;
          }
          else if (layer == 1) {
              layer = 16;
          }

          std::cout << "\t" << "<wire ";
          std::cout << "x1=\"" << grid_factor * last_location.x << "\" y1=\"" << grid_factor * last_location.y << "\" ";
          std::cout << "x2=\"" << grid_factor * twoPinNets[r].features[i].x << "\" y2=\"" << grid_factor * twoPinNets[r].features[i].y << "\" ";
          std::cout << "width=\"" << 0.6096 << "\" layer=\"" << layer << "\"";
          std::cout << "/>";
          std::cout << std::endl;

          last_location = twoPinNets[r].features[i];
      }
      std::cout << "</signal>";
      std::cout << std::endl;
  }

  // Print EAGLE file format to show the wires
  // Multipin net
  for(int mpr = 0; mpr<multipinNets.size(); ++mpr){
    std::cout << "<signal name=\"" << "MP" << mpr <<"\">" << std::endl;
    Location last_location = multipinNets[mpr].features[0];
    for (int i = 1; i < multipinNets[mpr].features.size(); i += 1) {
        int layer = multipinNets[mpr].features[i].z;

        // check if near
        if (
            abs(multipinNets[mpr].features[i].x - last_location.x) <= 1 &&
            abs(multipinNets[mpr].features[i].y - last_location.y) <= 1 &&
            abs(multipinNets[mpr].features[i].z - last_location.z) <= 1
        ){
            if (layer == 0) {
                layer = 1;
            }
            else if (layer == 1) {
                layer = 16;
            }

            std::cout << "\t" << "<wire ";
            std::cout << "x1=\"" << grid_factor * last_location.x << "\" y1=\"" << grid_factor * last_location.y << "\" ";
            std::cout << "x2=\"" << grid_factor * multipinNets[mpr].features[i].x << "\" y2=\"" << grid_factor * multipinNets[mpr].features[i].y << "\" ";
            std::cout << "width=\"" << 0.6096 << "\" layer=\"" << layer << "\"";
            std::cout << "/>";
            std::cout << std::endl;
        }

        last_location = multipinNets[mpr].features[i];
    }
    std::cout << "</signal>" << std::endl;
  }
  */
}