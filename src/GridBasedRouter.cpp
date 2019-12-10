//GridBasedRouter.cpp
#include "GridBasedRouter.h"

bool GridBasedRouter::outputResults2KiCadFile(std::vector<MultipinRoute> &nets, bool mergeSegments, std::string fileNameStamp) {
    std::ifstream ifs;
    ifs.open(mDb.getFileName(), std::ifstream::in);
    if (!ifs) {
        std::cerr << "Cannot open input file: " << mDb.getFileName() << std::endl;
        return false;
    }

    std::vector<std::string> fileLines;
    while (!ifs.eof()) {
        std::string inputLine;
        std::getline(ifs, inputLine);
        fileLines.push_back(inputLine);
        //std::cout << inputLine << std::endl;
    }

    ifs.close();

    if (fileLines.empty()) {
        std::cerr << "Input file has no content: " << mDb.getFileName() << std::endl;
        return false;
    }

    // Handle output filename
    std::string fileExtension = util::getFileExtension(mDb.getFileName());
    std::string fileNameWoExtension = util::getFileNameWoExtension(mDb.getFileName());
    std::string fileNameExtraTag;
    if (!fileNameStamp.empty()) {
        fileNameExtraTag = ".routed.ours." + fileNameStamp + ".";
    } else {
        fileNameExtraTag = ".routed.ours.";
    }
    std::string outputFileName = fileNameWoExtension + fileNameExtraTag + fileExtension;
    outputFileName = util::appendDirectory(GlobalParam::gOutputFolder, outputFileName);
    std::cout << __FUNCTION__ << "() outputFileName: " << outputFileName << std::endl;

    std::ofstream ofs;
    ofs.open(outputFileName, std::ofstream::out);
    if (!ofs) {
        std::cerr << "Cannot open output file: " << outputFileName << std::endl;
        return false;
    }
    // Remove the latest ")"
    for (int i = (int)fileLines.size() - 1; i >= 0; --i) {
        size_t found = fileLines.at(i).find_last_of(")");
        if (found != string::npos) {
            std::string newLine = fileLines.at(i).substr(0, found);
            fileLines.at(i) = newLine;
            break;
        }
    }
    // Paste inputs to outputs
    for (auto str : fileLines) {
        ofs << str << std::endl;
    }

    if ((mergeSegments && !writeNetsFromGridPaths(nets, ofs)) ||
        (!mergeSegments && !writeNets(nets, ofs))) {
        std::cerr << "Failed to write nets to the output file: " << outputFileName << std::endl;
        ofs.close();
        std::remove(outputFileName.c_str());
        return false;
    }

    ofs << ")" << std::endl;
    ofs.close();
    return true;
}

bool GridBasedRouter::writeNets(std::vector<MultipinRoute> &multipinNets, std::ofstream &ofs) {
    if (!ofs)
        return false;

    // Set output precision
    ofs << std::fixed << std::setprecision(GlobalParam::gOutputPrecision);
    // Estimated total routed wirelength
    double totalEstWL = 0.0;
    double totalEstGridWL = 0.0;
    int totalNumVia = 0;

    std::cout << "================= Start of " << __FUNCTION__ << "() =================" << std::endl;

    // Multipin net
    for (auto &mpNet : multipinNets) {
        if (!mDb.isNetId(mpNet.netId)) {
            std::cerr << __FUNCTION__ << "() Invalid net id: " << mpNet.netId << std::endl;
            continue;
        }

        auto &net = mDb.getNet(mpNet.netId);
        if (!mDb.isNetclassId(net.getNetclassId())) {
            std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
            continue;
        }

        if (mpNet.features.empty()) {
            continue;
        }

        auto &netclass = mDb.getNetclass(net.getNetclassId());
        Location prevLocation = mpNet.features.front();
        double netEstWL = 0.0;
        double netEstGridWL = 0.0;
        int netNumVia = 0;

        for (int i = 1; i < mpNet.features.size(); ++i) {
            auto &feature = mpNet.features[i];
            //std::cout << feature << std::endl;
            if (abs(feature.m_x - prevLocation.m_x) <= 1 &&
                abs(feature.m_y - prevLocation.m_y) <= 1 &&
                abs(feature.m_z - prevLocation.m_z) <= 1) {
                // Sanity Check
                if (feature.m_z != prevLocation.m_z &&
                    feature.m_y != prevLocation.m_y &&
                    feature.m_x != prevLocation.m_x) {
                    std::cerr << __FUNCTION__ << "() Invalid path between feature: " << feature << ", and lasLocation: " << prevLocation << std::endl;
                    continue;
                }
                // Print Through Hole Via
                if (feature.m_z != prevLocation.m_z) {
                    ++totalNumVia;
                    ++netNumVia;

                    ofs << "(via";
                    ofs << " (at " << GlobalParam::gridFactor * (prevLocation.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (prevLocation.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
                    ofs << " (size " << netclass.getViaDia() << ")";
                    ofs << " (drill " << netclass.getViaDrill() << ")";
                    ofs << " (layers Top Bottom)";
                    ofs << " (net " << mpNet.netId << ")";
                    ofs << ")" << std::endl;
                }

                // Print Segment/Track/Wire
                if (feature.m_x != prevLocation.m_x || feature.m_y != prevLocation.m_y) {
                    point_2d start{GlobalParam::gridFactor * (prevLocation.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2), GlobalParam::gridFactor * (prevLocation.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2)};
                    point_2d end{GlobalParam::gridFactor * (feature.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2), GlobalParam::gridFactor * (feature.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2)};
                    totalEstWL += point_2d::getDistance(start, end);
                    totalEstGridWL += Location::getDistance2D(prevLocation, feature);
                    netEstWL += point_2d::getDistance(start, end);
                    netEstGridWL += Location::getDistance2D(prevLocation, feature);

                    ofs << "(segment";
                    ofs << " (start " << start.m_x << " " << start.m_y << ")";
                    ofs << " (end " << end.m_x << " " << end.m_y << ")";
                    ofs << " (width " << netclass.getTraceWidth() << ")";
                    ofs << " (layer " << mGridLayerToName.at(feature.m_z) << ")";
                    ofs << " (net " << mpNet.netId << ")";
                    ofs << ")" << std::endl;
                }
            }
            prevLocation = feature;
        }
        std::cout << "\tNet " << net.getName() << "(" << net.getId() << "), netDegree: " << net.getPins().size()
                  << ", Total WL: " << netEstWL << ", Total Grid WL: " << netEstGridWL << ", #Vias: " << netNumVia << std::endl;
    }

    std::cout << "\tEstimated Total WL: " << totalEstWL << ", Total Grid WL: " << totalEstGridWL << ", Total # Vias: " << totalNumVia << std::endl;
    std::cout << "================= End of " << __FUNCTION__ << "() =================" << std::endl;
    return true;
}

bool GridBasedRouter::writeNetsFromGridPaths(std::vector<MultipinRoute> &multipinNets, std::ofstream &ofs) {
    if (!ofs)
        return false;

    // Set output precision
    ofs << std::fixed << std::setprecision(GlobalParam::gOutputPrecision);
    // Estimated total routed wirelength
    double totalEstWL = 0.0;
    double totalEstGridWL = 0.0;
    int totalNumVia = 0;

    std::cout << "================= Start of " << __FUNCTION__ << "() =================" << std::endl;

    // Multipin net
    for (auto &mpNet : multipinNets) {
        if (!mDb.isNetId(mpNet.netId)) {
            std::cerr << __FUNCTION__ << "() Invalid net id: " << mpNet.netId << std::endl;
            continue;
        }

        auto &net = mDb.getNet(mpNet.netId);
        if (!mDb.isNetclassId(net.getNetclassId())) {
            std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
            continue;
        }

        if (mpNet.features.empty()) {
            continue;
        }

        // Convert from features to grid paths
        mpNet.featuresToGridPaths();

        auto &netclass = mDb.getNetclass(net.getNetclassId());
        double netEstWL = 0.0;
        double netEstGridWL = 0.0;
        int netNumVia = 0;

        for (auto &gridPath : mpNet.mGridPaths) {
            Location prevLocation = gridPath.getSegments().front();

            for (auto &location : gridPath.getSegments()) {
                if (prevLocation == location) {
                    continue;
                }
                // Sanity Check
                if (location.m_z != prevLocation.m_z &&
                    location.m_y != prevLocation.m_y &&
                    location.m_x != prevLocation.m_x) {
                    std::cerr << __FUNCTION__ << "() Invalid path between location: " << location << ", and prevLocation: " << prevLocation << std::endl;
                    continue;
                }
                // Print Through Hole Via
                if (location.m_z != prevLocation.m_z) {
                    ++totalNumVia;
                    ++netNumVia;

                    ofs << "(via";
                    ofs << " (at " << GlobalParam::gridFactor * (prevLocation.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (prevLocation.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
                    ofs << " (size " << netclass.getViaDia() << ")";
                    ofs << " (drill " << netclass.getViaDrill() << ")";
                    ofs << " (layers Top Bottom)";
                    ofs << " (net " << mpNet.netId << ")";
                    ofs << ")" << std::endl;
                }

                // Print Segment/Track/Wire
                if (location.m_x != prevLocation.m_x || location.m_y != prevLocation.m_y) {
                    point_2d start{GlobalParam::gridFactor * (prevLocation.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2), GlobalParam::gridFactor * (prevLocation.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2)};
                    point_2d end{GlobalParam::gridFactor * (location.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2), GlobalParam::gridFactor * (location.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2)};
                    totalEstWL += point_2d::getDistance(start, end);
                    totalEstGridWL += Location::getDistance2D(prevLocation, location);
                    netEstWL += point_2d::getDistance(start, end);
                    netEstGridWL += Location::getDistance2D(prevLocation, location);

                    ofs << "(segment";
                    ofs << " (start " << start.m_x << " " << start.m_y << ")";
                    ofs << " (end " << end.m_x << " " << end.m_y << ")";
                    ofs << " (width " << netclass.getTraceWidth() << ")";
                    ofs << " (layer " << mGridLayerToName.at(location.m_z) << ")";
                    ofs << " (net " << mpNet.netId << ")";
                    ofs << ")" << std::endl;
                }
                prevLocation = location;
            }
        }
        std::cout << "\tNet " << net.getName() << "(" << net.getId() << "), netDegree: " << net.getPins().size()
                  << ", Total WL: " << netEstWL << ", Total Grid WL: " << netEstGridWL << ", #Vias: " << netNumVia << ", currentRouteCost: " << mpNet.currentRouteCost << std::endl;
    }

    std::cout << "\tEstimated Total WL: " << totalEstWL << ", Total Grid WL: " << totalEstGridWL << ", Total # Vias: " << totalNumVia << std::endl;
    std::cout << "================= End of " << __FUNCTION__ << "() =================" << std::endl;
    return true;
}

void GridBasedRouter::writeSolutionBackToDbAndSaveOutput(const std::string fileNameTag, std::vector<MultipinRoute> &multipinNets) {
    // Estimated total routed wirelength
    double totalEstWL = 0.0;
    double totalEstGridWL = 0.0;
    int totalNumVia = 0;

    std::cout << "================= Start of " << __FUNCTION__ << "() =================" << std::endl;

    // Multipin net
    for (auto &mpNet : multipinNets) {
        if (!mDb.isNetId(mpNet.netId)) {
            std::cout << __FUNCTION__ << "() Invalid net id: " << mpNet.netId << std::endl;
            continue;
        }

        auto &net = mDb.getNet(mpNet.netId);
        if (!mDb.isNetclassId(net.getNetclassId())) {
            std::cout << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
            continue;
        }

        if (mpNet.features.empty()) {
            continue;
        }

        // Convert from features to grid paths
        mpNet.featuresToGridPaths();

        // Clear current net's segments and vias
        net.clearSegments();
        net.clearVias();

        auto &netclass = mDb.getNetclass(net.getNetclassId());
        double netEstWL = 0.0;
        double netEstGridWL = 0.0;
        int netNumVia = 0;

        for (auto &gridPath : mpNet.mGridPaths) {
            Location prevLocation = gridPath.getSegments().front();

            for (auto &location : gridPath.getSegments()) {
                if (prevLocation == location) {
                    continue;
                }
                // Sanity Check
                if (location.m_z != prevLocation.m_z &&
                    location.m_y != prevLocation.m_y &&
                    location.m_x != prevLocation.m_x) {
                    std::cerr << __FUNCTION__ << "() Invalid path between location: " << location << ", and prevLocation: " << prevLocation << std::endl;
                    continue;
                }
                // Print Through Hole Via
                if (location.m_z != prevLocation.m_z) {
                    ++totalNumVia;
                    ++netNumVia;

                    Via via{net.getViaCount(), net.getId(), netclass.getViaDia()};
                    point_2d dbPoint;
                    this->gridPointToDbPoint(point_2d{(double)location.x(), (double)location.y()}, dbPoint);
                    via.setPosition(dbPoint);
                    via.setLayer(this->mGridLayerToName);
                    net.addVia(via);
                }
                // Print Segment/Track/Wire
                if (location.m_x != prevLocation.m_x || location.m_y != prevLocation.m_y) {
                    point_2d start, end;
                    this->gridPointToDbPoint(point_2d{(double)prevLocation.x(), (double)prevLocation.y()}, start);
                    this->gridPointToDbPoint(point_2d{(double)location.x(), (double)location.y()}, end);
                    totalEstWL += point_2d::getDistance(start, end);
                    totalEstGridWL += Location::getDistance2D(prevLocation, location);
                    netEstWL += point_2d::getDistance(start, end);
                    netEstGridWL += Location::getDistance2D(prevLocation, location);

                    Segment segment{net.getSegmentCount(), net.getId(), netclass.getTraceWidth(), mGridLayerToName.at(location.m_z)};
                    points_2d pts;
                    pts.push_back(start);
                    pts.push_back(end);
                    segment.setPosition(pts);
                    net.addSegment(segment);
                }
                prevLocation = location;
            }
        }
        std::cout << "\tNet " << net.getName() << "(" << net.getId() << "), netDegree: " << net.getPins().size()
                  << ", Total WL: " << netEstWL << ", Total Grid WL: " << netEstGridWL << ", #Vias: " << netNumVia << ", currentRouteCost: " << mpNet.currentRouteCost << std::endl;
    }

    std::cout << "\tEstimated Total WL: " << totalEstWL << ", Total Grid WL: " << totalEstGridWL << ", Total # Vias: " << totalNumVia << std::endl;
    std::cout << "================= End of " << __FUNCTION__ << "() =================" << std::endl;

    // Output the .kicad_pcb file
    std::string nameTag = fileNameTag;
    //std::string nameTag = "printKiCad";
    //nameTag = nameTag + "_s_" + std::to_string(GlobalParam::inputScale) + "_i_" + std::to_string(GlobalParam::numRipUpReRouteIteration) + "_b_" + std::to_string(GlobalParam::enlargeBoundary);
    mDb.printKiCad(GlobalParam::gOutputFolder, nameTag);
}

void GridBasedRouter::testRouterWithPinAndKeepoutAvoidance() {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << std::endl
              << "=================" << __FUNCTION__ << "==================" << std::endl;

    // Get board dimension
    mDb.getBoardBoundaryByPinLocation(mMinX, mMaxX, mMinY, mMaxY);
    std::cout << "Routing Outline: (" << mMinX << ", " << mMinY << "), (" << mMaxX << ", " << mMaxY << ")" << std::endl;
    std::cout << "GlobalParam::inputScale: " << GlobalParam::inputScale << ", GlobalParam::enlargeBoundary: " << GlobalParam::enlargeBoundary << ", GlobalParam::gridFactor: " << GlobalParam::gridFactor << std::endl;

    // Get grid dimension
    const unsigned int h = int(std::abs(mMaxY * GlobalParam::inputScale - mMinY * GlobalParam::inputScale)) + GlobalParam::enlargeBoundary;
    const unsigned int w = int(std::abs(mMaxX * GlobalParam::inputScale - mMinX * GlobalParam::inputScale)) + GlobalParam::enlargeBoundary;
    const unsigned int l = mDb.getNumCopperLayers();
    std::cout << "BoardGrid Size: w:" << w << ", h:" << h << ", l:" << l << std::endl;
    for (auto &layerIte : mDb.getCopperLayers()) {
        std::cout << "Grid layer: " << mGridLayerToName.size() << ", mapped to DB: " << layerIte.second << std::endl;
        mLayerNameToGrid[layerIte.second] = mGridLayerToName.size();
        mDbLayerIdToGridLayer[layerIte.first] = mGridLayerToName.size();
        mGridLayerToName.push_back(layerIte.second);
    }

    // Initialize board grid
    mBg.initilization(w, h, l);

    // Add all instances' pins to a cost in grid
    auto &instances = mDb.getInstances();
    for (auto &inst : instances) {
        if (!mDb.isComponentId(inst.getComponentId())) {
            std::cerr << __FUNCTION__ << "(): Illegal component Id: " << inst.getComponentId() << ", from Instance: " << inst.getName() << std::endl;
            continue;
        }

        auto &comp = mDb.getComponent(inst.getComponentId());
        for (auto &pad : comp.getPadstacks()) {
            // Add cost to both via/base cost grid
            addPinAvoidingCostToGrid(pad, inst, GlobalParam::gPinObstacleCost, true, false, true);
        }
    }

    // Add all nets to route
    std::vector<MultipinRoute> multipinNets;
    auto &nets = mDb.getNets();
    for (auto &net : nets) {
        std::cout << "Routing net: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;
        if (net.getPins().size() < 2)
            continue;

        std::vector<Location> pinLocations;
        auto &pins = net.getPins();
        for (auto &pin : pins) {
            point_2d pinDbLocation;
            mDb.getPinPosition(pin, &pinDbLocation);
            point_2d pinGridLocation;  // should be in int
            dbPointToGridPoint(pinDbLocation, pinGridLocation);

            // TODO: pad layer information
            // Throught hole???
            pinLocations.push_back(Location(pinGridLocation.m_x, pinGridLocation.m_y, 0));
            std::cout << " location in grid: " << pinLocations.back() << ", original abs. loc. : " << pinDbLocation.m_x << " " << pinDbLocation.m_y << std::endl;

            // Temporary reomve the pin cost on base cost grid
            // TODO:: true, true seems more reasonable
            addPinAvoidingCostToGrid(pin, -GlobalParam::gPinObstacleCost, true, false, true);
        }

        if (!mDb.isNetclassId(net.getNetclassId())) {
            std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
            continue;
        }
        auto &netclass = mDb.getNetclass(net.getNetclassId());
        int traceWidth = dbLengthToGridLengthCeil(netclass.getTraceWidth());
        int viaSize = dbLengthToGridLengthCeil(netclass.getViaDia());
        int clearance = dbLengthToGridLengthCeil(netclass.getClearance());
        std::cout << " traceWidth: " << traceWidth << "(db: " << netclass.getTraceWidth() << ")"
                  << ", viaSize: " << viaSize << "(db: " << netclass.getViaDia() << ")"
                  << ", clearance: " << clearance << "(db: " << netclass.getClearance() << ")" << std::endl;

        multipinNets.push_back(MultipinRoute(pinLocations, net.getId()));
        //mBg.set_current_rules(clearance, traceWidth, viaSize);
        mBg.setCurrentGridNetclassId(net.getNetclassId());
        //mBg.add_route(multipinNets.back());
        mBg.addRoute(multipinNets.back());

        // Put back the pin cost on base cost grid
        for (auto &pin : pins) {
            // TODO: true, true seems more reasonable
            addPinAvoidingCostToGrid(pin, GlobalParam::gPinObstacleCost, true, false, true);
        }
    }

    // Routing has done
    // Print the final base cost
    mBg.printGnuPlot();
    mBg.printMatPlot();

    // Output final result to KiCad file
    outputResults2KiCadFile(multipinNets);
}

void GridBasedRouter::setupBoardAndMappingStructure() {
    // Get board dimension
    mDb.getBoardBoundaryByPinLocation(this->mMinX, this->mMaxX, this->mMinY, this->mMaxY);
    std::cout << "Routing Outline: (" << this->mMinX << ", " << this->mMinY << "), (" << this->mMaxX << ", " << this->mMaxY << ")" << std::endl;
    std::cout << "GlobalParam::inputScale: " << GlobalParam::inputScale << ", GlobalParam::enlargeBoundary: " << GlobalParam::enlargeBoundary << ", GlobalParam::gridFactor: " << GlobalParam::gridFactor << std::endl;

    // Get grid dimension
    const unsigned int h = int(std::abs(mMaxY * GlobalParam::inputScale - mMinY * GlobalParam::inputScale)) + GlobalParam::enlargeBoundary;
    const unsigned int w = int(std::abs(mMaxX * GlobalParam::inputScale - mMinX * GlobalParam::inputScale)) + GlobalParam::enlargeBoundary;
    const unsigned int l = mDb.getNumCopperLayers();
    std::cout << "BoardGrid Size: w:" << w << ", h:" << h << ", l:" << l << std::endl;

    // Setup layer mappings
    for (auto &layerIte : mDb.getCopperLayers()) {
        std::cout << "Grid layer: " << mGridLayerToName.size() << ", mapped to DB: " << layerIte.second << std::endl;
        mLayerNameToGrid[layerIte.second] = mGridLayerToName.size();
        mDbLayerIdToGridLayer[layerIte.first] = mGridLayerToName.size();
        mGridLayerToName.push_back(layerIte.second);
    }

    // Setup netclass mapping
    for (auto &netclassIte : mDb.getNetclasses()) {
        int id = netclassIte.getId();
        int clearance = dbLengthToGridLengthCeil(netclassIte.getClearance());
        int traceWidth = dbLengthToGridLengthCeil(netclassIte.getTraceWidth());
        int viaDia = dbLengthToGridLengthCeil(netclassIte.getViaDia());
        int viaDrill = dbLengthToGridLengthCeil(netclassIte.getViaDrill());
        int microViaDia = dbLengthToGridLengthCeil(netclassIte.getMicroViaDia());
        int microViaDrill = dbLengthToGridLengthCeil(netclassIte.getMicroViaDrill());

        GridNetclass gridNetclass{id, clearance, traceWidth, viaDia, viaDrill, microViaDia, microViaDrill};

        // Setup derived values
        gridNetclass.setHalfTraceWidth((int)floor((double)traceWidth / 2.0));
        gridNetclass.setHalfViaDia((int)floor((double)viaDia / 2.0));

        // Diagnoal cases
        int diagonalTraceWidth = (int)ceil(dbLengthToGridLength(netclassIte.getTraceWidth()) / sqrt(2));
        gridNetclass.setDiagonalTraceWidth(diagonalTraceWidth);
        gridNetclass.setHalfDiagonalTraceWidth((int)floor((double)diagonalTraceWidth / 2.0));
        int diagonalClearance = (int)ceil(dbLengthToGridLength(netclassIte.getClearance()) / sqrt(2));
        gridNetclass.setDiagonalClearance(diagonalClearance);

        // !!! Move below expanding functions to a new function, to handle multiple netclasses

        // Setup expansion values
        gridNetclass.setViaExpansion(gridNetclass.getHalfViaDia());
        gridNetclass.setTraceExpansion(gridNetclass.getHalfTraceWidth());
        gridNetclass.setDiagonalTraceExpansion(gridNetclass.getHalfDiagonalTraceWidth());
        GridNetclass::setObstacleExpansion(0);
        // Expanded cases
        // gridNetclass.setViaExpansion(gridNetclass.getHalfViaDia() + gridNetclass.getClearance());
        // gridNetclass.setTraceExpansion(gridNetclass.getHalfTraceWidth() + gridNetclass.getClearance());
        // gridNetclass.setDiagonalTraceExpansion(gridNetclass.getHalfDiagonalTraceWidth() + gridNetclass.getDiagonalClearance());
        // GridNetclass::setObstacleExpansion(gridNetclass.getClearance());

        // Update Via shape grids
        double viaDiaFloating = dbLengthToGridLength(netclassIte.getViaDia());
        int halfViaDia = (int)floor((double)viaDia / 2.0);
        double halfViaDiaFloating = viaDiaFloating / 2.0;
        // Expanded cases
        // int halfViaDia = gridNetclass.getViaExpansion();
        // double halfViaDiaFloating = viaDiaFloating / 2.0 + dbLengthToGridLength(netclassIte.getClearance());

        std::vector<Point_2D<int>> viaGrids;
        getRasterizedCircle(halfViaDia, halfViaDiaFloating, viaGrids);
        gridNetclass.setViaShapeGrids(viaGrids);

        // Update via searching grids
        std::vector<Point_2D<int>> viaSearchingGrids;
        int viaSearchRadius = gridNetclass.getHalfViaDia() + gridNetclass.getClearance();
        double viaSearchRadiusFloating = halfViaDiaFloating + dbLengthToGridLength(netclassIte.getClearance());
        // Expanded cases
        // int viaSearchRadius = gridNetclass.getHalfViaDia();
        // double viaSearchRadiusFloating = halfViaDiaFloating;
        getRasterizedCircle(viaSearchRadius, viaSearchRadiusFloating, viaSearchingGrids);
        gridNetclass.setViaSearchingSpaceToGrids(viaSearchingGrids);

        // Update trace searching grids
        std::vector<Point_2D<int>> traceSearchingGrids;
        int traceSearchRadius = gridNetclass.getHalfTraceWidth() + gridNetclass.getClearance();
        double traceSearchRadiusFloating = dbLengthToGridLength(netclassIte.getTraceWidth()) / 2.0 + dbLengthToGridLength(netclassIte.getClearance());
        // Expanded cases
        // int traceSearchRadius = gridNetclass.getHalfTraceWidth();
        // double traceSearchRadiusFloating = dbLengthToGridLength(netclassIte.getTraceWidth()) / 2.0;
        getRasterizedCircle(traceSearchRadius, traceSearchRadiusFloating, traceSearchingGrids);
        gridNetclass.setTraceSearchingSpaceToGrids(traceSearchingGrids);
        // Debugging
        std::cout << "Relative trace searching grids points: " << std::endl;
        for (auto &pt : gridNetclass.getTraceSearchingSpaceToGrids()) {
            std::cout << pt << std::endl;
        }

        // Setup incremental searching grids
        gridNetclass.setupTraceIncrementalSearchGrids();
        gridNetclass.setupViaIncrementalSearchGrids();
        // Put the netclass into class vectors
        mBg.addGridNetclass(gridNetclass);

        std::cout << "==============DB netclass: id: " << netclassIte.getId() << "==============" << std::endl;
        std::cout << "clearance: " << netclassIte.getClearance() << ", traceWidth: " << netclassIte.getTraceWidth() << std::endl;
        std::cout << "viaDia: " << netclassIte.getViaDia() << ", viaDrill: " << netclassIte.getViaDrill() << std::endl;
        std::cout << "microViaDia: " << netclassIte.getMicroViaDia() << ", microViaDrill: " << netclassIte.getMicroViaDrill() << std::endl;
        std::cout << "==============Grid netclass: id: " << id << "==============" << std::endl;
        std::cout << "clearance: " << gridNetclass.getClearance() << ", diagonal clearance: " << gridNetclass.getDiagonalClearance() << std::endl;
        std::cout << "traceWidth: " << gridNetclass.getTraceWidth() << ", half traceWidth: " << gridNetclass.getHalfTraceWidth() << std::endl;
        std::cout << "diagonal traceWidth: " << gridNetclass.getDiagonalTraceWidth() << ", half diagonal traceWidth: " << gridNetclass.getHalfDiagonalTraceWidth() << std::endl;
        std::cout << "viaDia: " << gridNetclass.getViaDia() << ", halfViaDia: " << gridNetclass.getHalfViaDia() << ", viaDrill: " << gridNetclass.getViaDrill() << std::endl;
        std::cout << "microViaDia: " << gridNetclass.getMicroViaDia() << ", microViaDrill: " << gridNetclass.getMicroViaDrill() << std::endl;
        std::cout << "==============Grid netclass: id: " << id << ", Expansions==============" << std::endl;
        std::cout << "viaExpansion: " << gridNetclass.getViaExpansion() << std::endl;
        std::cout << "traceExpansion: " << gridNetclass.getTraceExpansion() << std::endl;
        std::cout << "DiagonalTraceExpansion: " << gridNetclass.getDiagonalTraceExpansion() << std::endl;
        std::cout << "(static)obstacleExpansion: " << GridNetclass::getObstacleExpansion() << std::endl;
    }

    // Initialize board grid
    mBg.initilization(w, h, l);
}

void GridBasedRouter::getRasterizedCircle(const int radius, const double radiusFloating, std::vector<Point_2D<int>> &grids) {
    // Center grid
    grids.push_back(Point_2D<int>{0, 0});
    if (radius == 0) {
        return;
    }
    // The rests
    for (int x = -radius; x <= radius; ++x) {
        for (int y = -radius; y <= radius; ++y) {
            if (x == 0 && y == 0) continue;

            // Check if any corner of grid is within the halfViaDiaFloating
            Point_2D<double> LL{(double)x - 0.5, (double)y - 0.5};
            Point_2D<double> LR{(double)x + 0.5, (double)y - 0.5};
            Point_2D<double> UL{(double)x - 0.5, (double)y + 0.5};
            Point_2D<double> UR{(double)x + 0.5, (double)y + 0.5};
            Point_2D<double> center{0.0, 0.0};

            if (Point_2D<double>::getDistance(LL, center) < radiusFloating ||
                Point_2D<double>::getDistance(LR, center) < radiusFloating ||
                Point_2D<double>::getDistance(UL, center) < radiusFloating ||
                Point_2D<double>::getDistance(UR, center) < radiusFloating) {
                grids.push_back(Point_2D<int>{x, y});
            }
        }
    }
}

void GridBasedRouter::setupGridNetsAndGridPins() {
    std::cout << "Starting " << __FUNCTION__ << "()..." << std::endl;

    // Iterate nets
    for (auto &net : mDb.getNets()) {
        std::cout << "Net: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;

        gridNets.push_back(MultipinRoute{net.getId(), net.getNetclassId()});
        auto &gridRoute = gridNets.back();
        auto &pins = net.getPins();
        for (auto &pin : pins) {
            // TODO: Id Range Checking?
            // DB elements
            auto &comp = mDb.getComponent(pin.getCompId());
            auto &inst = mDb.getInstance(pin.getInstId());
            auto &pad = comp.getPadstack(pin.getPadstackId());
            // Router grid element
            auto &gridPin = gridRoute.getNewGridPin();
            // Setup the GridPin
            this->getGridPin(pad, inst, gridPin);
        }
    }

    // Iterate instances
    auto &instances = mDb.getInstances();
    for (auto &inst : instances) {
        if (!mDb.isComponentId(inst.getComponentId())) {
            std::cerr << __FUNCTION__ << "(): Illegal component Id: " << inst.getComponentId() << ", from Instance: " << inst.getName() << std::endl;
            continue;
        }

        auto &comp = mDb.getComponent(inst.getComponentId());
        for (auto &pad : comp.getPadstacks()) {
            // Router grid element
            mGridPins.push_back(GridPin{});
            auto &gridPin = mGridPins.back();
            // Setup the GridPin
            this->getGridPin(pad, inst, gridPin);
        }
    }

    std::cout << "End of " << __FUNCTION__ << "()..." << std::endl;
}

void GridBasedRouter::getGridPin(const padstack &pad, const instance &inst, GridPin &gridPin) {
    getGridPin(pad, inst, GridNetclass::getObstacleExpansion(), gridPin);
}

void GridBasedRouter::getGridPin(const padstack &pad, const instance &inst, const int gridExpansion, GridPin &gridPin) {
    // Setup GridPin's location with layers
    Point_2D<double> pinDbLocation;
    mDb.getPinPosition(pad, inst, &pinDbLocation);
    Point_2D<int> pinGridLocation;
    dbPointToGridPointRound(pinDbLocation, pinGridLocation);
    std::vector<int> layers;
    this->getGridLayers(pad, inst, layers);

    std::cout << " location in grid: " << pinGridLocation << ", original abs. loc. : " << pinDbLocation.m_x << " " << pinDbLocation.m_y << ", layers:";
    for (auto layer : layers) {
        gridPin.pinWithLayers.push_back(Location(pinGridLocation.m_x, pinGridLocation.m_y, layer));
        std::cout << " " << layer;
    }
    std::cout << ", #layers:" << gridPin.pinWithLayers.size() << " " << layers.size() << std::endl;

    // Setup GridPin's LL,UR boundary
    double width = 0, height = 0;
    mDb.getPadstackRotatedWidthAndHeight(inst, pad, width, height);
    Point_2D<double> pinDbUR{pinDbLocation.m_x + width / 2.0, pinDbLocation.m_y + height / 2.0};
    Point_2D<double> pinDbLL{pinDbLocation.m_x - width / 2.0, pinDbLocation.m_y - height / 2.0};
    Point_2D<int> pinGridLL, pinGridUR;
    dbPointToGridPointRound(pinDbUR, pinGridUR);
    dbPointToGridPointRound(pinDbLL, pinGridLL);
    pinGridUR.m_x += gridExpansion;
    pinGridUR.m_y += gridExpansion;
    pinGridLL.m_x -= gridExpansion;
    pinGridLL.m_y -= gridExpansion;
    gridPin.setPinLL(pinGridLL);
    gridPin.setPinUR(pinGridUR);

    // Handle expanded pad shape polygon
    double dbExpansion = gridLengthToDbLength((double)gridExpansion);
    Point_2D<double> expandedPadSize = pad.getSize();
    expandedPadSize.m_x += (2.0 * dbExpansion);
    expandedPadSize.m_y += (2.0 * dbExpansion);
    std::vector<Point_2D<double>> expandedPadPoly = shape_to_coords(expandedPadSize, point_2d{0, 0}, pad.getPadShape(), inst.getAngle(), pad.getAngle(), pad.getRoundRectRatio(), 32);

    // Calculate pinShapeToGrids
    // 1. Make Boost polygon of pad shape
    polygon_t padShapePoly;
    for (auto pt : expandedPadPoly) {
        bg::append(padShapePoly.outer(), point(pt.x() + pinDbLocation.x(), pt.y() + pinDbLocation.y()));
    }
    // printPolygon(padShapePoly);

    for (int x = pinGridLL.m_x; x <= pinGridUR.m_x; ++x) {
        for (int y = pinGridLL.m_y; y <= pinGridUR.m_y; ++y) {
            // 2. Make fake grid box as Boost polygon
            point_2d gridDbLL, gridDbUR;
            polygon_t gridDbPoly;
            this->gridPointToDbPoint(point_2d{(double)x - 0.5, (double)y - 0.5}, gridDbLL);
            this->gridPointToDbPoint(point_2d{(double)x + 0.5, (double)y + 0.5}, gridDbUR);
            //std::cout << "gridDbLL: " << gridDbLL << ", gridDbUR" << gridDbUR << std::endl;
            bg::append(gridDbPoly.outer(), point(gridDbLL.x(), gridDbLL.y()));
            bg::append(gridDbPoly.outer(), point(gridDbLL.x(), gridDbUR.y()));
            bg::append(gridDbPoly.outer(), point(gridDbUR.x(), gridDbUR.y()));
            bg::append(gridDbPoly.outer(), point(gridDbUR.x(), gridDbLL.y()));
            bg::append(gridDbPoly.outer(), point(gridDbLL.x(), gridDbLL.y()));  // Closed loop
            // printPolygon(gridDbPoly);

            // Compare if the grid box polygon has overlaps with padstack polygon
            if (bg::overlaps(gridDbPoly, padShapePoly) || bg::within(gridDbPoly, padShapePoly)) {
                gridPin.addPinShapeGridPoint(Point_2D<int>{x, y});
            }
        }
    }
}

void GridBasedRouter::testRouterWithAvoidanceAndVariousPadType() {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << std::endl
              << "=================" << __FUNCTION__ << "==================" << std::endl;

    this->setupBoardAndMappingStructure();

    // Add all instances' pins to a cost in grid
    auto &instances = mDb.getInstances();
    for (auto &inst : instances) {
        if (!mDb.isComponentId(inst.getComponentId())) {
            std::cerr << __FUNCTION__ << "(): Illegal component Id: " << inst.getComponentId() << ", from Instance: " << inst.getName() << std::endl;
            continue;
        }

        auto &comp = mDb.getComponent(inst.getComponentId());
        for (auto &pad : comp.getPadstacks()) {
            // Add cost to both via/base cost grid
            addPinAvoidingCostToGrid(pad, inst, GlobalParam::gPinObstacleCost, true, true, true);
        }
    }

    // Add all nets to route
    std::vector<MultipinRoute> multipinNets;
    auto &nets = mDb.getNets();
    for (auto &net : nets) {
        std::cout << "\n\nRouting net: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;
        if (net.getPins().size() < 2)
            continue;

        multipinNets.push_back(MultipinRoute{net.getId()});
        auto &route = multipinNets.back();
        auto &pins = net.getPins();
        for (auto &pin : pins) {
            point_2d pinDbLocation;
            mDb.getPinPosition(pin, &pinDbLocation);
            point_2d pinGridLocation;  // should be in int
            dbPointToGridPoint(pinDbLocation, pinGridLocation);
            std::vector<int> layers;
            this->getGridLayers(pin, layers);
            auto &gridPin = route.getNewGridPin();

            std::cout << " location in grid: " << pinGridLocation << ", original abs. loc. : " << pinDbLocation.m_x << " " << pinDbLocation.m_y << ", layers:";
            for (auto layer : layers) {
                gridPin.pinWithLayers.push_back(Location(pinGridLocation.m_x, pinGridLocation.m_y, layer));
                std::cout << " " << layer;
            }
            std::cout << ", #layers:" << gridPin.pinWithLayers.size() << " " << layers.size() << std::endl;

            // Temporary remove the pin cost on base cost grid
            addPinAvoidingCostToGrid(pin, -GlobalParam::gPinObstacleCost, true, false, true);
        }

        if (!mDb.isNetclassId(net.getNetclassId())) {
            std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
            continue;
        }
        //mBg.set_current_rules(net.getNetclassId());
        mBg.setCurrentGridNetclassId(net.getNetclassId());

        mBg.addRouteWithGridPins(multipinNets.back());

        // Put back the pin cost on base cost grid
        for (auto &pin : pins) {
            addPinAvoidingCostToGrid(pin, GlobalParam::gPinObstacleCost, true, false, true);
        }
    }

    std::cout << "\n\n======= Finished Routing all nets. =======\n\n"
              << std::endl;

    // Routing has done
    // Print the final base cost
    mBg.printGnuPlot();
    mBg.printMatPlot();

    // Output final result to KiCad file
    outputResults2KiCadFile(multipinNets);
}

void GridBasedRouter::testRouterWithRipUpAndReroute() {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << std::endl
              << "=================" << __FUNCTION__ << "==================" << std::endl;

    // TODO
    // unified structure for Polygon, Rectangle
    // ====> Rectangle Template....
    // ====> Put pin rectangle into GridPin and use the rect template....
    // THROUGH HOLE Pad/Via?????? SMD Pad, Mirco Via?????

    // Initilization
    this->setupBoardAndMappingStructure();
    this->setupGridNetsAndGridPins();

    // Add all instances' pins to a cost in grid (without inflation for spacing)
    // this->addAllPinCostToGrid(0);
    for (auto &gridPin : this->mGridPins) {
        this->addPinShapeAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, true, true);
        // this->addPinShapeAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, false, true);
    }

    std::string initialMapNameTag = util::getFileNameWoExtension(mDb.getFileName()) + ".initial" + "_s_" + std::to_string(GlobalParam::inputScale) + "_i_" + std::to_string(GlobalParam::numRipUpReRouteIteration) + "_b_" + std::to_string(GlobalParam::enlargeBoundary);
    mBg.printMatPlot(initialMapNameTag);

    // Add all nets to grid routes
    double totalCurrentRouteCost = 0.0;
    double bestTotalRouteCost = 0.0;
    auto &nets = mDb.getNets();
    for (auto &net : nets) {
        // if (net.getId() != 7)
        //     continue;
        // if (net.getId() != 18 && net.getId() != 19)
        //     continue;
        // if (net.getId() > 24)
        //     continue;

        std::cout << "\n\nRouting net: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;
        if (net.getPins().size() < 2)
            continue;

        gridNets.push_back(MultipinRoute{net.getId()});
        auto &gridRoute = gridNets.at(net.getId());
        if (net.getId() != gridRoute.netId)
            std::cout << "!!!!!!! inconsistent net.getId(): " << net.getId() << ", gridRoute.netId: " << gridRoute.netId << std::endl;

        // Temporary reomve the pin cost on the cost grid
        for (auto &gridPin : gridRoute.mGridPins) {
            // addPinAvoidingCostToGrid(gridPin, -GlobalParam::gPinObstacleCost, true, false, true);
            this->addPinShapeAvoidingCostToGrid(gridPin, -GlobalParam::gPinObstacleCost, true, false, true);
        }

        // Setup design rules in board grid
        if (!mDb.isNetclassId(net.getNetclassId())) {
            std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
            continue;
        }
        mBg.setCurrentGridNetclassId(net.getNetclassId());
        //mBg.set_current_rules(net.getNetclassId());

        // Route the net
        mBg.addRouteWithGridPins(gridRoute);
        totalCurrentRouteCost += gridRoute.currentRouteCost;
        std::cout << "=====> currentRouteCost: " << gridRoute.currentRouteCost << ", totalCost: " << totalCurrentRouteCost << std::endl;

        // Put back the pin cost on base cost grid
        for (auto &gridPin : gridRoute.mGridPins) {
            // addPinAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, false, true);
            this->addPinShapeAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, false, true);
        }
    }

    // Set up the base solution
    std::vector<double> iterativeCost;
    iterativeCost.push_back(totalCurrentRouteCost);
    bestTotalRouteCost = totalCurrentRouteCost;
    this->bestSolution = this->gridNets;

    if (GlobalParam::gOutputDebuggingKiCadFile) {
        std::string nameTag = "fristTimeRouteAll";
        nameTag = nameTag + "_s_" + std::to_string(GlobalParam::inputScale) + "_i_" + std::to_string(GlobalParam::numRipUpReRouteIteration) + "_b_" + std::to_string(GlobalParam::enlargeBoundary);
        // outputResults2KiCadFile(this->gridNets, true, nameTag);
        writeSolutionBackToDbAndSaveOutput(nameTag, this->gridNets);
    }
    std::cout << "i=0, totalCurrentRouteCost: " << totalCurrentRouteCost << ", bestTotalRouteCost: " << bestTotalRouteCost << std::endl;

    std::cout << "\n\n======= Start Fixed-Order Rip-Up and Re-Route all nets. =======\n\n";

    // Rip-up and Re-route all the nets one-by-one ten times
    for (int i = 0; i < GlobalParam::numRipUpReRouteIteration; ++i) {
        for (auto &net : nets) {
            //continue;
            if (net.getPins().size() < 2)
                continue;

            //auto &gridRoute = gridNets.at(rippedUpGridNetId);
            auto &gridRoute = gridNets.at(net.getId());
            if (net.getId() != gridRoute.netId)
                std::cout << "!!!!!!! inconsistent net.getId(): " << net.getId() << ", gridRoute.netId: " << gridRoute.netId << std::endl;

            std::cout << "\n\ni=" << i << ", Routing net: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;

            // Temporary reomve the pin cost on the cost grid
            for (auto &gridPin : gridRoute.mGridPins) {
                // addPinAvoidingCostToGrid(gridPin, -GlobalParam::gPinObstacleCost, true, false, true);
                this->addPinShapeAvoidingCostToGrid(gridPin, -GlobalParam::gPinObstacleCost, true, false, true);
            }

            if (!mDb.isNetclassId(net.getNetclassId())) {
                std::cerr << __FUNCTION__ << "() Invalid netclass id: " << net.getNetclassId() << std::endl;
                continue;
            }
            mBg.setCurrentGridNetclassId(net.getNetclassId());
            //mBg.set_current_rules(net.getNetclassId());

            // Rip-up and re-route
            mBg.ripup_route(gridRoute);
            totalCurrentRouteCost -= gridRoute.currentRouteCost;
            mBg.addRouteWithGridPins(gridRoute);
            totalCurrentRouteCost += gridRoute.currentRouteCost;

            // Put back the pin cost on base cost grid
            for (auto &gridPin : gridRoute.mGridPins) {
                // addPinAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, false, true);
                this->addPinShapeAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, false, true);
            }
        }
        if (GlobalParam::gOutputDebuggingKiCadFile) {
            std::string nameTag = "i_" + std::to_string(i + 1);
            nameTag = nameTag + "_s_" + std::to_string(GlobalParam::inputScale) + "_i_" + std::to_string(GlobalParam::numRipUpReRouteIteration) + "_b_" + std::to_string(GlobalParam::enlargeBoundary);
            //outputResults2KiCadFile(this->gridNets, true, nameTag);
            writeSolutionBackToDbAndSaveOutput(nameTag, this->gridNets);
        }
        if (totalCurrentRouteCost < bestTotalRouteCost) {
            std::cout << "!!!!>!!!!> Found new bestTotalRouteCost: " << totalCurrentRouteCost << ", from: " << bestTotalRouteCost << std::endl;
            bestTotalRouteCost = totalCurrentRouteCost;
            this->bestSolution = this->gridNets;
        }
        iterativeCost.push_back(totalCurrentRouteCost);
        std::cout << "i=" << i + 1 << ", totalCurrentRouteCost: " << totalCurrentRouteCost << ", bestTotalRouteCost: " << bestTotalRouteCost << std::endl;
    }
    std::cout << "\n\n======= Rip-up and Re-route cost breakdown =======" << std::endl;
    for (int i = 0; i < iterativeCost.size(); ++i) {
        cout << "i=" << i << ", cost: " << iterativeCost.at(i) << std::endl;
    }

    std::cout << "\n\n======= Finished Routing all nets. =======\n\n"
              << std::endl;

    // Routing has done
    // Print the final base cost
    //mBg.printGnuPlot();
    std::string mapNameTag = util::getFileNameWoExtension(mDb.getFileName()) + "_s_" + std::to_string(GlobalParam::inputScale) + "_i_" + std::to_string(GlobalParam::numRipUpReRouteIteration) + "_b_" + std::to_string(GlobalParam::enlargeBoundary);
    mBg.printMatPlot(mapNameTag);

    // Output final result to KiCad file
    std::string nameTag = "bestSolutionWithMerging";
    nameTag = nameTag + "_s_" + std::to_string(GlobalParam::inputScale) + "_i_" + std::to_string(GlobalParam::numRipUpReRouteIteration) + "_b_" + std::to_string(GlobalParam::enlargeBoundary);
    // outputResults2KiCadFile(this->bestSolution, true, nameTag);
    writeSolutionBackToDbAndSaveOutput(nameTag, this->bestSolution);
    //outputResults2KiCadFile(this->bestSolution, false, "bestSolutionWoMerging");
}

void GridBasedRouter::testRouterWithPinShape() {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << std::endl
              << "=================" << __FUNCTION__ << "==================" << std::endl;

    // Initilization
    this->setupBoardAndMappingStructure();
    this->setupGridNetsAndGridPins();

    // Add all instances' pins to a cost in grid (without inflation for spacing)
    //this->addAllPinCostToGrid(0);
    for (auto &gridPin : this->mGridPins) {
        this->addPinShapeAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, true, true);
    }

    // Routing has done
    // Print the final base cost
    mBg.printGnuPlot();
    mBg.printMatPlot();

    // Output final result to KiCad file
    // outputResults2KiCadFile(this->bestSolution, true, "bestSolutionWithMerging");
    // writeSolutionBackToDbAndSaveOutput(this->bestSolution);
}

void GridBasedRouter::addAllPinCostToGrid(const int inflate) {
    for (auto &gridPin : mGridPins) {
        addPinAvoidingCostToGrid(gridPin, GlobalParam::gPinObstacleCost, true, true, true, inflate);
    }
}

void GridBasedRouter::addPinAvoidingCostToGrid(const Pin &p, const float value, const bool toViaCost, const bool toViaForbidden, const bool toBaseCost, const int inflate) {
    // TODO: Id Range Checking?
    auto &comp = mDb.getComponent(p.getCompId());
    auto &inst = mDb.getInstance(p.getInstId());
    auto &pad = comp.getPadstack(p.getPadstackId());

    addPinAvoidingCostToGrid(pad, inst, value, toViaCost, toViaForbidden, toBaseCost, inflate);
}

void GridBasedRouter::addPinAvoidingCostToGrid(const padstack &pad, const instance &inst, const float value, const bool toViaCost, const bool toViaForbidden, const bool toBaseCost, const int inflate) {
    Point_2D<double> pinDbLocation;
    mDb.getPinPosition(pad, inst, &pinDbLocation);
    double width = 0, height = 0;
    mDb.getPadstackRotatedWidthAndHeight(inst, pad, width, height);
    Point_2D<double> pinDbUR{pinDbLocation.m_x + width / 2.0, pinDbLocation.m_y + height / 2.0};
    Point_2D<double> pinDbLL{pinDbLocation.m_x - width / 2.0, pinDbLocation.m_y - height / 2.0};
    Point_2D<int> pinGridLL, pinGridUR;
    dbPointToGridPointRound(pinDbUR, pinGridUR);
    dbPointToGridPointRound(pinDbLL, pinGridLL);
    if (inflate > 0) {
        pinGridUR.m_x += inflate;
        pinGridUR.m_y += inflate;
        pinGridLL.m_x -= inflate;
        pinGridLL.m_y -= inflate;
    }
    std::cout << __FUNCTION__ << "()"
              << " toViaCostGrid:" << toViaCost << ", toViaForbidden:" << toViaForbidden << ", toBaseCostGrid:" << toBaseCost;
    std::cout << ", cost:" << value << ", inst:" << inst.getName() << "(" << inst.getId() << "), pad:"
              << pad.getName() << ", at(" << pinDbLocation.m_x << ", " << pinDbLocation.m_y
              << "), w:" << width << ", h:" << height << ", LLatgrid:" << pinGridLL << ", URatgrid:" << pinGridUR
              << ", inflate: " << inflate << ", layers:";

    // TODO: Unify Rectangle to set costs
    // Get layer from Padstack's type and instance's layers
    std::vector<int> layers;
    this->getGridLayers(pad, inst, layers);

    for (auto &layer : layers) {
        std::cout << " " << layer;
    }
    std::cout << std::endl;

    for (auto &layer : layers) {
        for (int x = pinGridLL.m_x; x <= pinGridUR.m_x; ++x) {
            for (int y = pinGridLL.m_y; y <= pinGridUR.m_y; ++y) {
                Location gridPt{x, y, layer};
                if (!mBg.validate_location(gridPt)) {
                    //std::cout << "\tWarning: Out of bound, pin cost at " << gridPt << std::endl;
                    continue;
                }
                //std::cout << "\tAdd pin cost at " << gridPt << std::endl;
                if (toBaseCost) {
                    mBg.base_cost_add(value, gridPt);
                }
                if (toViaCost) {
                    mBg.via_cost_add(value, gridPt);
                }
                //TODO:: How to controll clear/set
                if (toViaForbidden) {
                    mBg.setViaForbidden(gridPt);
                }
            }
        }
    }
}

void GridBasedRouter::addPinAvoidingCostToGrid(const GridPin &gridPin, const float value, const bool toViaCost, const bool toViaForbidden, const bool toBaseCost, const int inflate) {
    Point_2D<int> pinGridLL = gridPin.getPinLL();
    Point_2D<int> pinGridUR = gridPin.getPinUR();
    if (inflate > 0) {
        pinGridUR.m_x += inflate;
        pinGridUR.m_y += inflate;
        pinGridLL.m_x -= inflate;
        pinGridLL.m_y -= inflate;
    }
    std::cout << __FUNCTION__ << "()"
              << " toViaCostGrid:" << toViaCost << ", toViaForbidden:" << toViaForbidden << ", toBaseCostGrid:" << toBaseCost;
    std::cout << ", cost:" << value << ", LLatgrid:" << pinGridLL << ", URatgrid:" << pinGridUR
              << ", inflate: " << inflate << std::endl;

    for (auto &location : gridPin.getPinWithLayers()) {
        for (int x = pinGridLL.m_x; x <= pinGridUR.m_x; ++x) {
            for (int y = pinGridLL.m_y; y <= pinGridUR.m_y; ++y) {
                Location gridPt{x, y, location.z()};
                if (!mBg.validate_location(gridPt)) {
                    //std::cout << "\tWarning: Out of bound, pin cost at " << gridPt << std::endl;
                    continue;
                }
                //std::cout << "\tAdd pin cost at " << gridPt << std::endl;
                if (toBaseCost) {
                    mBg.base_cost_add(value, gridPt);
                }
                if (toViaCost) {
                    mBg.via_cost_add(value, gridPt);
                }
                //TODO:: How to controll clear/set
                if (toViaForbidden) {
                    mBg.setViaForbidden(gridPt);
                }
            }
        }
    }
}

void GridBasedRouter::addPinShapeAvoidingCostToGrid(const GridPin &gridPin, const float value, const bool toViaCost, const bool toViaForbidden, const bool toBaseCost) {
    Point_2D<int> pinGridLL = gridPin.getPinLL();
    Point_2D<int> pinGridUR = gridPin.getPinUR();

    std::cout << __FUNCTION__ << "()"
              << " toViaCostGrid:" << toViaCost << ", toViaForbidden:" << toViaForbidden << ", toBaseCostGrid:" << toBaseCost;
    std::cout << ", cost:" << value << ", LLatgrid:" << pinGridLL << ", URatgrid:" << pinGridUR << ", pinShape.size(): " << gridPin.getPinShapeToGrids().size() << std::endl;

    for (auto &location : gridPin.getPinWithLayers()) {
        for (auto pt : gridPin.getPinShapeToGrids()) {
            Location gridPt{pt.x(), pt.y(), location.z()};
            if (!mBg.validate_location(gridPt)) {
                // std::cout << "\tWarning: Out of bound, pin cost at " << gridPt << std::endl;
                continue;
            }
            //std::cout << "\tAdd pin cost at " << gridPt << std::endl;
            if (toBaseCost) {
                mBg.base_cost_add(value, gridPt);
            }
            if (toViaCost) {
                mBg.via_cost_add(value, gridPt);
            }
            //TODO:: How to controll clear/set
            if (toViaForbidden) {
                mBg.setViaForbidden(gridPt);
            }
        }
    }
}

bool GridBasedRouter::getGridLayers(const Pin &pin, std::vector<int> &layers) {
    // TODO: Id Range Checking?
    auto &comp = mDb.getComponent(pin.getCompId());
    auto &inst = mDb.getInstance(pin.getInstId());
    auto &pad = comp.getPadstack(pin.getPadstackId());

    return getGridLayers(pad, inst, layers);
}

bool GridBasedRouter::getGridLayers(const padstack &pad, const instance &inst, std::vector<int> &layers) {
    if (pad.getType() == padType::SMD) {
        auto layerIte = mDbLayerIdToGridLayer.find(inst.getLayer());
        if (layerIte != mDbLayerIdToGridLayer.end()) {
            layers.push_back(layerIte->second);
        }
    } else {
        //Put all the layers
        int layer = 0;
        for (auto l : mGridLayerToName) {
            layers.push_back(layer);
            ++layer;
        }
    }
    return true;
}

int GridBasedRouter::getNextRipUpNetId() {
    return rand() % gridNets.size();
}

bool GridBasedRouter::dbPointToGridPoint(const point_2d &dbPt, point_2d &gridPt) {
    //TODO: boundary checking
    gridPt.m_x = dbPt.m_x * GlobalParam::inputScale - mMinX * GlobalParam::inputScale + GlobalParam::enlargeBoundary / 2;
    gridPt.m_y = dbPt.m_y * GlobalParam::inputScale - mMinY * GlobalParam::inputScale + GlobalParam::enlargeBoundary / 2;
    return true;
}

bool GridBasedRouter::dbPointToGridPointCeil(const Point_2D<double> &dbPt, Point_2D<int> &gridPt) {
    //TODO: boundary checking
    gridPt.m_x = ceil(dbPt.m_x * GlobalParam::inputScale - mMinX * GlobalParam::inputScale + (double)GlobalParam::enlargeBoundary / 2);
    gridPt.m_y = ceil(dbPt.m_y * GlobalParam::inputScale - mMinY * GlobalParam::inputScale + (double)GlobalParam::enlargeBoundary / 2);
    return true;
}

bool GridBasedRouter::dbPointToGridPointFloor(const Point_2D<double> &dbPt, Point_2D<int> &gridPt) {
    //TODO: boundary checking
    gridPt.m_x = floor(dbPt.m_x * GlobalParam::inputScale - mMinX * GlobalParam::inputScale + (double)GlobalParam::enlargeBoundary / 2);
    gridPt.m_y = floor(dbPt.m_y * GlobalParam::inputScale - mMinY * GlobalParam::inputScale + (double)GlobalParam::enlargeBoundary / 2);
    return true;
}

bool GridBasedRouter::dbPointToGridPointRound(const Point_2D<double> &dbPt, Point_2D<int> &gridPt) {
    //TODO: boundary checking
    gridPt.m_x = round(dbPt.m_x * GlobalParam::inputScale - mMinX * GlobalParam::inputScale + (double)GlobalParam::enlargeBoundary / 2);
    gridPt.m_y = round(dbPt.m_y * GlobalParam::inputScale - mMinY * GlobalParam::inputScale + (double)GlobalParam::enlargeBoundary / 2);
    return true;
}

bool GridBasedRouter::gridPointToDbPoint(const point_2d &gridPt, point_2d &dbPt) {
    //TODO: boundary checking
    //TODO: consider integer ceiling or flooring???
    dbPt.m_x = GlobalParam::gridFactor * (gridPt.m_x + mMinX * GlobalParam::inputScale - (double)GlobalParam::enlargeBoundary / 2);
    dbPt.m_y = GlobalParam::gridFactor * (gridPt.m_y + mMinY * GlobalParam::inputScale - (double)GlobalParam::enlargeBoundary / 2);
    return true;
}

void GridBasedRouter::test_router() {
    std::vector<std::set<std::pair<double, double>>> routerInfo;
    mDb.getPcbRouterInfo(&routerInfo);

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "=================test_router==================" << std::endl;

    for (int i = 0; i < routerInfo.size(); ++i) {
        std::cout << "netId: " << i << std::endl;

        for (auto ite : routerInfo.at(i)) {
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
    const unsigned int h = int(std::abs(mMaxY * GlobalParam::inputScale - mMinY * GlobalParam::inputScale)) + GlobalParam::enlargeBoundary;
    const unsigned int w = int(std::abs(mMaxX * GlobalParam::inputScale - mMinX * GlobalParam::inputScale)) + GlobalParam::enlargeBoundary;
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

    // Prepare all the nets to route
    for (int i = 0; i < routerInfo.size(); ++i) {
        std::cout << "Routing netId: " << i << "..." << std::endl;
        size_t netDegree = routerInfo.at(i).size();
        if (netDegree < 2)
            continue;

        std::vector<Location> pins;
        for (auto ite : routerInfo.at(i)) {
            std::cout << "\t(" << ite.first << ", " << ite.second << ")";
            pins.push_back(Location(ite.first * GlobalParam::inputScale - mMinX * GlobalParam::inputScale + GlobalParam::enlargeBoundary / 2, ite.second * GlobalParam::inputScale - mMinY * GlobalParam::inputScale + GlobalParam::enlargeBoundary / 2, 0));
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
        std::cout << " (at " << GlobalParam::gridFactor * (last_location.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (last_location.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
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
        ofs << " (at " << GlobalParam::gridFactor * (last_location.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (last_location.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
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
        std::cout << " (start " << GlobalParam::gridFactor * (last_location.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (last_location.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
        std::cout << " (end " << GlobalParam::gridFactor * (twoPinNets[r].features[i].m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (twoPinNets[r].features[i].m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
        // BM2
        //std::cout << " (width 0.2032)";
        // BBBW
        std::cout << " (width 0.1524)";
        std::cout << " (layer " << layer << ")";
        std::cout << " (net " << twoPinNets[r].netId << ")";
        std::cout << ")" << std::endl;

        ofs << "(segment";
        ofs << " (start " << GlobalParam::gridFactor * (last_location.m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (last_location.m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
        ofs << " (end " << GlobalParam::gridFactor * (twoPinNets[r].features[i].m_x + mMinX * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << " " << GlobalParam::gridFactor * (twoPinNets[r].features[i].m_y + mMinY * GlobalParam::inputScale - GlobalParam::enlargeBoundary / 2) << ")";
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
          std::cout << "x1=\"" << GlobalParam::gridFactor * last_location.m_x << "\" y1=\"" << GlobalParam::gridFactor * last_location.m_y << "\" ";
          std::cout << "x2=\"" << GlobalParam::gridFactor * twoPinNets[r].features[i].m_x << "\" y2=\"" << GlobalParam::gridFactor * twoPinNets[r].features[i].m_y << "\" ";
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
            std::cout << "x1=\"" << GlobalParam::gridFactor * last_location.m_x << "\" y1=\"" << GlobalParam::gridFactor * last_location.m_y << "\" ";
            std::cout << "x2=\"" << GlobalParam::gridFactor * multipinNets[mpr].features[i].m_x << "\" y2=\"" << GlobalParam::gridFactor * multipinNets[mpr].features[i].m_y << "\" ";
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
