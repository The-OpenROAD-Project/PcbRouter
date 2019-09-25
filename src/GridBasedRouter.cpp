//GridBasedRouter.cpp
#include "GridBasedRouter.h"

bool GridBasedRouter::outputResults2KiCadFile(std::vector<MultipinRoute> &nets)
{
  std::ifstream ifs;
  ifs.open(mDb.getFileName(), std::ifstream::in);
  if (!ifs)
  {
    std::cerr << "Cannot open input file: " << mDb.getFileName() << std::endl;
    return false;
  }

  std::vector<std::string> fileLines;
  while (!ifs.eof())
  {
    std::string inputLine;
    std::getline(ifs, inputLine);
    fileLines.push_back(inputLine);
    //std::cout << inputLine << std::endl;
  }

  ifs.close();

  if (fileLines.empty())
  {
    std::cerr << "Input file has no content: " << mDb.getFileName() << std::endl;
    return false;
  }

  // Handle output filename
  std::string fileExtension = util::getFileExtension(mDb.getFileName());
  std::string fileNameWoExtension = util::getFileNameWoExtension(mDb.getFileName());
  std::string outputFileName = fileNameWoExtension + ".routed.ours." + fileExtension;
  outputFileName = util::appendDirectory(GlobalParam::gOutputFolder, outputFileName);
  std::cout << __FUNCTION__ << "() outputFileName: " << outputFileName << std::endl;

  std::ofstream ofs;
  ofs.open(outputFileName, std::ofstream::out);
  if (!ofs)
  {
    std::cerr << "Cannot open output file: " << outputFileName << std::endl;
    return false;
  }
  // Remove the latest ")"
  for (int i = (int)fileLines.size() - 1; i >= 0; --i)
  {
    size_t found = fileLines.at(i).find_last_of(")");
    if (found != string::npos)
    {
      std::string newLine = fileLines.at(i).substr(0, found);
      fileLines.at(i) = newLine;
      break;
    }
  }
  // Paste inputs to outputs
  for (auto str : fileLines)
  {
    ofs << str << std::endl;
  }

  if (!writeNets(nets, ofs))
  {
    std::cerr << "Failed to write nets to the output file: " << outputFileName << std::endl;
    ofs.close();
    std::remove(outputFileName.c_str());
    return false;
  }

  ofs << ")" << std::endl;
  ofs.close();
  return true;
}

bool GridBasedRouter::writeNets(std::vector<MultipinRoute> &multipinNets, std::ofstream &ofs)
{
  if (!ofs)
    return false;

  // Set output precision
  ofs << std::fixed << std::setprecision(GlobalParam::gOutputPrecision);
  // Estimated total routed wirelength
  double totalEstWL = 0.0;

  // Multipin net
  for (auto &mpNet : multipinNets)
  {
    if (!mDb.isNetId(mpNet.netId))
    {
      std::cerr << __FUNCTION__ << "() Invalid net id: " << mpNet.netId << std::endl;
      continue;
    }

    auto &net = mDb.getNet(mpNet.netId);
    if (!mDb.isNetclassId(net.getNetclassId()))
    {
      std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
      continue;
    }

    auto &netclass = mDb.getNetclass(net.getNetclassId());
    Location last_location = mpNet.features[0];

    for (int i = 1; i < mpNet.features.size(); ++i)
    {
      auto &feature = mpNet.features[i];
      // check if close ???????
      if (
          abs(feature.m_x - last_location.m_x) <= 1 &&
          abs(feature.m_y - last_location.m_y) <= 1 &&
          abs(feature.m_z - last_location.m_z) <= 1)
      {
        // Print Through Hole Via
        if (feature.m_z != last_location.m_z)
        {
          ofs << "(via";
          ofs << " (at " << grid_factor * (last_location.m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (last_location.m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
          ofs << " (size " << netclass.getViaDia() << ")";
          ofs << " (drill " << netclass.getViaDrill() << ")";
          ofs << " (layers Top Bottom)";
          ofs << " (net " << mpNet.netId << ")";
          ofs << ")" << std::endl;
        }

        // Print Segment/Track/Wire
        if (feature.m_x != last_location.m_x || feature.m_y != last_location.m_y)
        {
          point_2d start{grid_factor * (last_location.m_x + mMinX * inputScale - enlargeBoundary / 2), grid_factor * (last_location.m_y + mMinY * inputScale - enlargeBoundary / 2)};
          point_2d end{grid_factor * (feature.m_x + mMinX * inputScale - enlargeBoundary / 2), grid_factor * (feature.m_y + mMinY * inputScale - enlargeBoundary / 2)};
          totalEstWL += point_2d::getDistance(start, end);

          ofs << "(segment";
          ofs << " (start " << start.m_x << " " << start.m_y << ")";
          ofs << " (end " << end.m_x << " " << end.m_y << ")";
          ofs << " (width " << netclass.getTraceWidth() << ")";
          ofs << " (layer " << mGridLayerToName.at(feature.m_z) << ")";
          ofs << " (net " << mpNet.netId << ")";
          ofs << ")" << std::endl;
        }
      }
      last_location = feature;
    }
  }

  std::cout << "=================" << __FUNCTION__ << "=================" << std::endl;
  std::cout << "\tEstimated Total WL: " << totalEstWL << std::endl;
  return true;
}

void GridBasedRouter::testRouterWithPinAndKeepoutAvoidance()
{
  std::cout << std::fixed << std::setprecision(5);
  std::cout << std::endl
            << "=================" << __FUNCTION__ << "==================" << std::endl;

  // TODO
  // bug at matplot function??
  // unified structure for Polygon, Rectangle

  // THROUGH HOLE Pad/Via?????? SMD Pad, Mirco Via?????
  // Loop Instaces: Add all pins into boardgrid with high cost
  // ?? Perform rip-up and re-route ??

  // Get board dimension
  mDb.getBoardBoundaryByPinLocation(mMinX, mMaxX, mMinY, mMaxY);
  std::cout << "Routing Outline: (" << mMinX << ", " << mMinY << "), (" << mMaxX << ", " << mMaxY << ")" << std::endl;
  std::cout << "inputScale: " << inputScale << ", enlargeBoundary: " << enlargeBoundary << ", grid_factor: " << grid_factor << std::endl;

  // Get grid dimension
  const unsigned int h = int(std::abs(mMaxY * inputScale - mMinY * inputScale)) + enlargeBoundary;
  const unsigned int w = int(std::abs(mMaxX * inputScale - mMinX * inputScale)) + enlargeBoundary;
  const unsigned int l = mDb.getNumCopperLayers();
  std::cout << "BoardGrid Size: w:" << w << ", h:" << h << ", l:" << l << std::endl;
  for (auto &layerIte : mDb.getCopperLayers())
  {
    std::cout << "Grid layer: " << mGridLayerToName.size() << ", mapped to DB: " << layerIte.second << std::endl;
    mLayerNameToGrid[layerIte.second] = mGridLayerToName.size();
    mGridLayerToName.push_back(layerIte.second);
  }

  // Initialize board grid
  mBg.initilization(w, h, l);
  mBg.base_cost_fill(0.0);

  // Add all instances' pins to a cost in grid
  auto &instances = mDb.getInstances();
  for (auto &inst : instances)
  {
    if (!mDb.isComponentId(inst.getComponentId()))
    {
      std::cerr << __FUNCTION__ << "(): Illegal component Id: " << inst.getComponentId() << ", from Instance: " << inst.getName() << std::endl;
      continue;
    }

    auto &comp = mDb.getComponent(inst.getComponentId());
    for (auto &pad : comp.getPadstacks())
    {
      addPinCost(pad, inst, pinCost);
    }
  }

  // Add all nets to route
  std::vector<MultipinRoute> multipinNets;
  auto &nets = mDb.getNets();
  for (auto &net : nets)
  {
    std::cout << "Routing net: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;
    if (net.getPins().size() < 2)
      continue;

    std::vector<Location> pinLocations;
    auto &pins = net.getPins();
    for (auto &pin : pins)
    {
      point_2d pinDbLocation;
      mDb.getPinPosition(pin, &pinDbLocation);
      point_2d pinGridLocation; // should be in int
      dbPointToGridPoint(pinDbLocation, pinGridLocation);

      // TODO: pad layer information
      // Throught hole???
      pinLocations.push_back(Location(pinGridLocation.m_x, pinGridLocation.m_y, 0));
      std::cout << " location in grid: " << pinLocations.back() << ", original abs. loc. : " << pinDbLocation.m_x << " " << pinDbLocation.m_y << std::endl;

      // Temporary reomve the pin cost
      addPinCost(pin, -pinCost);
    }

    if (!mDb.isNetclassId(net.getNetclassId()))
    {
      std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
      continue;
    }
    auto &netclass = mDb.getNetclass(net.getNetclassId());
    int traceWidth = dbLengthToGridLength(netclass.getTraceWidth());
    int viaSize = dbLengthToGridLength(netclass.getViaDia());
    int clearance = dbLengthToGridLength(netclass.getClearance());
    std::cout << " traceWidth: " << traceWidth << "(db: " << netclass.getTraceWidth() << ")"
              << ", viaSize: " << viaSize << "(db: " << netclass.getViaDia() << ")"
              << ", clearance: " << clearance << "(db: " << netclass.getClearance() << ")" << std::endl;

    multipinNets.push_back(MultipinRoute(pinLocations, net.getId(), traceWidth, viaSize, clearance));
    mBg.add_route(multipinNets.back());

    // Put back the pin cost
    for (auto &pin : pins)
    {
      addPinCost(pin, pinCost);
    }
  }

  // Routing has done
  // Print the final base cost
  mBg.printGnuPlot();
  mBg.printMatPlot();

  // Output final result to KiCad file
  outputResults2KiCadFile(multipinNets);
}

void GridBasedRouter::addPinCost(const pin &p, const float cost)
{
  // TODO: Id Range Checking?
  auto &comp = mDb.getComponent(p.m_comp_id);
  auto &inst = mDb.getInstance(p.m_inst_id);
  auto &pad = comp.getPadstack(p.m_padstack_id);

  addPinCost(pad, inst, cost);
}

void GridBasedRouter::addPinCost(const padstack &pad, const instance &inst, const float cost)
{
  point_2d pinDbLocation;
  mDb.getPinPosition(pad, inst, &pinDbLocation);
  double width = 0, height = 0;
  mDb.getPadstackRotatedWidthAndHeight(inst, pad, width, height);
  point_2d pinDbUR{pinDbLocation.m_x + width / 2.0, pinDbLocation.m_y + height / 2.0};
  point_2d pinDbLL{pinDbLocation.m_x - width / 2.0, pinDbLocation.m_y - height / 2.0};
  point_2d pinGridLL, pinGridUR;
  dbPointToGridPoint(pinDbUR, pinGridUR);
  dbPointToGridPoint(pinDbLL, pinGridLL);
  std::cout << __FUNCTION__ << "() cost:" << cost << ", inst:" << inst.getName() << "(" << inst.getId() << "), pad:" << pad.getName() << ", at(" << pinDbLocation.m_x << ", " << pinDbLocation.m_y << "), w:" << width << ", h:" << height << std::endl;

  // TODO: Unify Rectangle to set costs
  const auto &layers = pad.getLayers();
  for (auto &layer : layers)
  {
    const auto &layerIte = mLayerNameToGrid.find(layer);
    if (layerIte != mLayerNameToGrid.end())
    {
      for (int x = pinGridLL.m_x; x < (int)pinGridUR.m_x; ++x)
      {
        for (int y = pinGridLL.m_y; y < (int)pinGridUR.m_y; ++y)
        {
          Location gridPt{x, y, layerIte->second};
          if (!mBg.validate_location(gridPt))
          {
            //std::cout << "\tWarning: Out of bound, pin cost at " << gridPt << std::endl;
            continue;
          }
          //std::cout << "\tAdd pin cost at " << gridPt << std::endl;
          mBg.base_cost_add(cost, gridPt);
          mBg.via_cost_add(cost, gridPt);
        }
      }
    }
  }
}

bool GridBasedRouter::dbPointToGridPoint(const point_2d &dbPt, point_2d &gridPt)
{
  //TODO: boundary checking
  //TODO: consider integer ceiling or flooring???
  gridPt.m_x = dbPt.m_x * inputScale - mMinX * inputScale + enlargeBoundary / 2;
  gridPt.m_y = dbPt.m_y * inputScale - mMinY * inputScale + enlargeBoundary / 2;
  return true;
}

bool GridBasedRouter::gridPointToDbPoint(const point_2d &gridPt, point_2d &dbPt)
{
  //TODO: boundary checking
  //TODO: consider integer ceiling or flooring???
  dbPt.m_x = grid_factor * (gridPt.m_x + mMinX * inputScale - enlargeBoundary / 2);
  dbPt.m_y = grid_factor * (gridPt.m_y + mMinY * inputScale - enlargeBoundary / 2);
  return true;
}

void GridBasedRouter::test_router()
{
  std::vector<std::set<std::pair<double, double>>> routerInfo;
  mDb.getPcbRouterInfo(&routerInfo);

  std::cout << std::fixed << std::setprecision(5);
  std::cout << "=================test_router==================" << std::endl;

  for (int i = 0; i < routerInfo.size(); ++i)
  {
    std::cout << "netId: " << i << std::endl;

    for (auto ite : routerInfo.at(i))
    {
      std::cout << " (" << ite.first << ", " << ite.second << ")";
      mMinX = std::min(ite.first, mMinX);
      mMaxX = std::max(ite.first, mMaxX);
      mMinY = std::min(ite.second, mMinY);
      mMaxY = std::max(ite.second, mMaxY);
    }
    std::cout << std::endl;
  }

  std::cout << "Routing Outline: (" << mMinX << ", " << mMinY << "), (" << mMaxX << ", " << mMaxY << ")" << std::endl;

  mDb.getBoardBoundaryByPinLocation(mMinX, mMaxX, mMinY, mMaxY);

  std::cout << "Routing Outline From DB: (" << mMinX << ", " << mMinY << "), (" << mMaxX << ", " << mMaxY << ")" << std::endl;

  // Initialize board grid
  const unsigned int h = int(std::abs(mMaxY * inputScale - mMinY * inputScale)) + enlargeBoundary;
  const unsigned int w = int(std::abs(mMaxX * inputScale - mMinX * inputScale)) + enlargeBoundary;
  // BM*
  const unsigned int l = 2;
  // BBBW
  //const unsigned int l = 4;
  // TODO
  //const int max_ripups = 20000;
  // std::vector<Route> twoPinNets;
  std::vector<MultipinRoute> multipinNets;

  //std::ofstream ofs("router.output", std::ofstream::out);
  //ofs << std::fixed << std::setprecision(5);

  std::cout << "BoardGrid Size: w:" << w << ", h:" << h << ", l:" << l << std::endl;

  mBg.initilization(w, h, l);
  mBg.base_cost_fill(0.0);

  // Prepare all the nets to route
  for (int i = 0; i < routerInfo.size(); ++i)
  {
    std::cout << "Routing netId: " << i << "..." << std::endl;
    size_t netDegree = routerInfo.at(i).size();
    if (netDegree < 2)
      continue;

    std::vector<Location> pins;
    for (auto ite : routerInfo.at(i))
    {
      std::cout << "\t(" << ite.first << ", " << ite.second << ")";
      pins.push_back(Location(ite.first * inputScale - mMinX * inputScale + enlargeBoundary / 2, ite.second * inputScale - mMinY * inputScale + enlargeBoundary / 2, 0));
      std::cout << " location in grid: " << pins.back() << std::endl;
    }

    /*    if (netDegree == 2)
    {
      //Route route = Route(pins.at(0), pins.at(1));
      twoPinNets.push_back(Route(pins.at(0), pins.at(1), i));
      mBg.add_route(twoPinNets.back());
    }
    else */
    {
      //continue;
      //MultipinRoute mpRoute(pins);
      multipinNets.push_back(MultipinRoute(pins, i));
      mBg.add_route(multipinNets.back());
    }
    std::cout << "Routing netId: " << i << "...done!" << std::endl;
  }

  // Routing has done
  // Print the final base cost
  mBg.printGnuPlot();
  mBg.printMatPlot();

  outputResults2KiCadFile(multipinNets);

  /*
  // Print KiCad file format
  // 2-pins
  for (int r = 0; r < twoPinNets.size(); r += 1)
  {
    Location last_location = twoPinNets[r].features[0];

    for (int i = 1; i < twoPinNets[r].features.size(); i += 1)
    {
      std::string layer;
      if (twoPinNets[r].features[i].m_z == 0)
      {
        layer = "Top";
      }
      else if (twoPinNets[r].features[i].m_z == 1)
      {
        layer = "Route3";
      }
      else if (twoPinNets[r].features[i].m_z == 2)
      {
        layer = "Route14";
      }
      else if (twoPinNets[r].features[i].m_z == 3)
      {
        layer = "Bottom";
      }
      else
      {
        layer = "Top";
      }

      // Via
      // TODO check via layer order
      // TODO feature will have two consequences same points...
      // Eg.   
      //    (via (at 153.9341 96.7756) (size 0.8001) (drill 0.3937) (layers Top Bottom) (net 18) (tstamp 384ED00))
      //    (via blind (at 153.9341 95.7756) (size 0.8001) (drill 0.3937) (layers In1.Cu Bottom) (net 18) (tstamp 384ED00))
      //    (via micro (at 153.9341 94.7756) (size 0.8001) (drill 0.3937) (layers In1.Cu In2.Cu) (net 18) (tstamp 384ED00))
      if (twoPinNets[r].features[i].m_z != last_location.m_z)
      {
        std::cout << "(via";
        std::cout << " (at " << grid_factor * (last_location.m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (last_location.m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
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
        ofs << " (at " << grid_factor * (last_location.m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (last_location.m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
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

      if (twoPinNets[r].features[i].m_x != last_location.m_x || twoPinNets[r].features[i].m_y != last_location.m_y)
      {
        // Wire/Tack/Segment
        std::cout << "(segment";
        std::cout << " (start " << grid_factor * (last_location.m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (last_location.m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
        std::cout << " (end " << grid_factor * (twoPinNets[r].features[i].m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (twoPinNets[r].features[i].m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
        // BM2
        //std::cout << " (width 0.2032)";
        // BBBW
        std::cout << " (width 0.1524)";
        std::cout << " (layer " << layer << ")";
        std::cout << " (net " << twoPinNets[r].netId << ")";
        std::cout << ")" << std::endl;

        ofs << "(segment";
        ofs << " (start " << grid_factor * (last_location.m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (last_location.m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
        ofs << " (end " << grid_factor * (twoPinNets[r].features[i].m_x + mMinX * inputScale - enlargeBoundary / 2) << " " << grid_factor * (twoPinNets[r].features[i].m_y + mMinY * inputScale - enlargeBoundary / 2) << ")";
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
  */

  //ofs.close();

  /*
  // Print EAGLE file format to show the wires
  // 2-pins
  for (int r = 0; r < twoPinNets.size(); r += 1) {
      // std::cout << std::endl;
      std::cout << "<signal name=\"" << "2P" << r <<"\">" << std::endl;
      Location last_location = twoPinNets[r].features[0];

      for (int i = 1; i < twoPinNets[r].features.size(); i += 1) {
          int layer = twoPinNets[r].features[i].m_z;
          if (layer == 0) {
              layer = 1;
          }
          else if (layer == 1) {
              layer = 16;
          }

          std::cout << "\t" << "<wire ";
          std::cout << "x1=\"" << grid_factor * last_location.m_x << "\" y1=\"" << grid_factor * last_location.m_y << "\" ";
          std::cout << "x2=\"" << grid_factor * twoPinNets[r].features[i].m_x << "\" y2=\"" << grid_factor * twoPinNets[r].features[i].m_y << "\" ";
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
        int layer = multipinNets[mpr].features[i].m_z;

        // check if near
        if (
            abs(multipinNets[mpr].features[i].m_x - last_location.m_x) <= 1 &&
            abs(multipinNets[mpr].features[i].m_y - last_location.m_y) <= 1 &&
            abs(multipinNets[mpr].features[i].m_z - last_location.m_z) <= 1
        ){
            if (layer == 0) {
                layer = 1;
            }
            else if (layer == 1) {
                layer = 16;
            }

            std::cout << "\t" << "<wire ";
            std::cout << "x1=\"" << grid_factor * last_location.m_x << "\" y1=\"" << grid_factor * last_location.m_y << "\" ";
            std::cout << "x2=\"" << grid_factor * multipinNets[mpr].features[i].m_x << "\" y2=\"" << grid_factor * multipinNets[mpr].features[i].m_y << "\" ";
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
