#include "BoardGrid.h"

void BoardGrid::initilization(int w, int h, int l) {
    this->w = w;
    this->h = h;
    this->l = l;
    this->size = w * h * l;

    assert(this->grid == nullptr);
    this->grid = new GridCell[this->size];
    assert(this->grid != nullptr);

    this->base_cost_fill(0.0);
    // this->via_cost_fill(0.0);
}

void BoardGrid::base_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].baseCost = value;
    }
}

void BoardGrid::working_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].workingCost = value;
    }
}

void BoardGrid::bending_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].bendingCost = value;
    }
}

void BoardGrid::cached_trace_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].cachedTraceCost = value;
    }
}

void BoardGrid::cached_via_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].cachedViaCost = value;
    }
}

// void BoardGrid::via_cost_fill(float value) {
//     for (int i = 0; i < this->size; ++i) {
//         //this->grid[i].viaCost = value;
//         this->grid[i].baseCost = value;
//     }
// }

float BoardGrid::base_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost;
}

float BoardGrid::via_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    //return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost;
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost;
}

float BoardGrid::working_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].workingCost;
}

float BoardGrid::bending_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].bendingCost;
}

float BoardGrid::cached_trace_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cachedTraceCost;
}

float BoardGrid::cached_via_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cachedViaCost;
}

void BoardGrid::base_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost = value;
}

void BoardGrid::base_cost_add(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost += value;
}

void BoardGrid::base_cost_add(float value, const Location &l, const std::vector<Point_2D<int>> &shapeToGrids) {
    for (const auto &relativePt : shapeToGrids) {
#ifdef BOUND_CHECKS
        assert(((l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (l.m_z) * this->w * this->h) < this->size);
#endif
        this->grid[(l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (l.m_z) * this->w * this->h].baseCost += value;
    }
}

void BoardGrid::working_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].workingCost = value;
}

void BoardGrid::bending_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].bendingCost = value;
}

void BoardGrid::cached_trace_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cachedTraceCost = value;
}

void BoardGrid::cached_via_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cachedViaCost = value;
}

void BoardGrid::setCameFromId(const Location &l, const int id) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cameFromId = id;
}

int BoardGrid::getCameFromId(const Location &l) const {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cameFromId;
}

int BoardGrid::getCameFromId(const int id) const {
#ifdef BOUND_CHECKS
    assert(id < this->size);
#endif
    return this->grid[id].cameFromId;
}

void BoardGrid::clearAllCameFromId() {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].cameFromId = -1;
    }
}

int BoardGrid::locationToId(const Location &l) const {
    return l.m_x + l.m_y * this->w + l.m_z * this->w * this->h;
}

void BoardGrid::idToLocation(const int id, Location &l) const {
    l.m_z = id / (this->w * this->h);
    l.m_y = (id - l.m_z * this->w * this->h) / this->w;
    l.m_x = (id - l.m_z * this->w * this->h) % this->w;
}

void BoardGrid::via_cost_set(const float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    //this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost = value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost = value;
}

void BoardGrid::via_cost_add(const float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    //this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost += value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost += value;
}

void BoardGrid::setTargetedPin(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    //this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].targetedPin = true;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cellType = GridCellType::TARGET_PIN;
}

void BoardGrid::clearTargetedPin(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    //this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].targetedPin = false;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cellType = GridCellType::VACANT;
}

bool BoardGrid::isTargetedPin(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    //return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].targetedPin;
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cellType == GridCellType::TARGET_PIN;
}

void BoardGrid::setTargetedPins(const std::vector<Location> &pins) {
    for (const auto &pin : pins) {
        this->setTargetedPin(pin);
    }
}
void BoardGrid::clearTargetedPins(const std::vector<Location> &pins) {
    for (const auto &pin : pins) {
        this->clearTargetedPin(pin);
    }
}

void BoardGrid::setViaForbidden(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaForbidden = true;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cellType = GridCellType::VIA_FORBIDDEN;
}

void BoardGrid::clearViaForbidden(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaForbidden = false;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cellType = GridCellType::VACANT;
}

bool BoardGrid::isViaForbidden(const Location &l) const {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaForbidden;
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cellType == GridCellType::VIA_FORBIDDEN;
}

void BoardGrid::setViaForbiddenArea(const std::vector<Location> &locations) {
    for (const auto &loc : locations) {
        this->setViaForbidden(loc);
    }
}
void BoardGrid::clearViaForbiddenArea(const std::vector<Location> &locations) {
    for (const auto &loc : locations) {
        this->clearViaForbidden(loc);
    }
}

// std::unordered_map<Location, Location> BoardGrid::dijkstras_with_came_from(
//     const std::vector<Location> &route, int via_size) {
//     std::unordered_map<Location, Location> came_from;
//     this->dijkstras_with_came_from(route, via_size, came_from);
//     return came_from;
// }

// void BoardGrid::dijkstras_with_came_from(
//     const std::vector<Location> &route, int via_size,
//     std::unordered_map<Location, Location> &came_from) {
//     std::cout << __FUNCTION__
//               << "() nets: route.features.size() = " << route.size()
//               << std::endl;

//     // For path to multiple points
//     // Searches from the multiple points to every other point
//     this->working_cost_fill(std::numeric_limits<float>::infinity());

//     LocationQueue<Location, float> frontier;  // search frontier
//     // std::unordered_map<Location, Location> came_from; // cheapest neighbor
//     for (Location start : route) {
//         this->working_cost_set(0.0, start);
//         frontier.push(start, 0.0);
//         // came_from[start] = start;
//         this->setCameFromId(start, this->locationToId(start));
//     }

//     std::cout << "came_from.size() = " << came_from.size()
//               << ", frontier.size(): " << frontier.size() << std::endl;

//     while (!frontier.empty()) {
//         Location current = frontier.front();
//         frontier.pop();

//         // std::cout << "Visiting " << current << ", frontierSize: "<<
//         // frontier.size() << std::endl;
//         std::vector<std::pair<float, Location>> neighbors;
//         this->getNeighbors(current, neighbors);

//         for (std::pair<float, Location> next : neighbors) {
//             if ((next.second.m_x < 0) || (next.second.m_x >= this->w) ||
//                 (next.second.m_y < 0) || (next.second.m_y >= this->h) ||
//                 (next.second.m_z < 0) || (next.second.m_z >= this->l)) {
//                 continue;  // continue if out of bounds
//             }
//             // std::cerr << "next.second.m_x: " << next.second.m_x << std::endl;
//             // std::cerr << "next.second.m_y: " << next.second.m_y << std::endl;
//             // std::cerr << "next.second.m_z: " << next.second.m_z << std::endl;

//             // std::cerr << "geting new cost" << std::endl;

//             // this->via_cost_at(next.second) ??????????
//             float new_cost = this->working_cost_at(current) +
//                              this->base_cost_at(next.second) +
//                              this->via_cost_at(next.second) + next.first;

//             // std::cerr << "Done" << std::endl;

//             if (new_cost < this->working_cost_at(next.second)) {
//                 // std::cerr << "setting working cost" << std::endl;
//                 this->working_cost_set(new_cost, next.second);
//                 // came_from[next.second] = current;
//                 this->setCameFromId(next.second, this->locationToId(current));

//                 frontier.push(next.second, new_cost);
//                 // std::cerr << "Done" << std::endl;
//             }

//             // std::cerr << std::endl;
//         }
//     }
//     // std::cerr << "finished dijkstras_with_came_from" << std::endl;
// }

// void BoardGrid::dijkstrasWithGridCameFrom(const std::vector<Location> &route,
//                                           int via_size) {
//     std::cout << __FUNCTION__
//               << "() nets: route.features.size() = " << route.size()
//               << std::endl;

//     // For path to multiple points
//     // Searches from the multiple points to every other point
//     this->working_cost_fill(std::numeric_limits<float>::infinity());

//     LocationQueue<Location, float> frontier;  // search frontier
//     for (Location start : route) {
//         this->working_cost_set(0.0, start);
//         frontier.push(start, 0.0);
//         // Set a ending for the backtracking
//         this->setCameFromId(start, this->locationToId(start));
//     }

//     std::cout << " frontier.size(): " << frontier.size() << std::endl;

//     while (!frontier.empty()) {
//         Location current = frontier.front();
//         frontier.pop();

//         // std::cout << "Visiting " << current << ", frontierSize: "<<
//         // frontier.size() << std::endl;
//         std::vector<std::pair<float, Location>> neighbors;
//         this->getNeighbors(current, neighbors);

//         for (std::pair<float, Location> next : neighbors) {
//             if ((next.second.m_x < 0) || (next.second.m_x >= this->w) ||
//                 (next.second.m_y < 0) || (next.second.m_y >= this->h) ||
//                 (next.second.m_z < 0) || (next.second.m_z >= this->l)) {
//                 continue;  // continue if out of bounds
//             }
//             // std::cerr << "next.second.m_x: " << next.second.m_x << std::endl;
//             // std::cerr << "next.second.m_y: " << next.second.m_y << std::endl;
//             // std::cerr << "next.second.m_z: " << next.second.m_z << std::endl;

//             // this->via_cost_at(next.second) ??????????
//             float new_cost = this->working_cost_at(current) +
//                              this->base_cost_at(next.second) +
//                              this->via_cost_at(next.second) + next.first;

//             if (new_cost < this->working_cost_at(next.second)) {
//                 // std::cerr << "setting working cost" << std::endl;
//                 this->working_cost_set(new_cost, next.second);
//                 this->setCameFromId(next.second, this->locationToId(current));

//                 frontier.push(next.second, new_cost);
//             }
//         }
//     }
// }

void BoardGrid::aStarWithGridCameFrom(const std::vector<Location> &route, Location &finalEnd, float &finalCost) {
    std::cout << __FUNCTION__ << "() nets: route.features.size() = " << route.size() << std::endl;

    // For path to multiple points
    // Searches from the multiple points to every other point
    this->working_cost_fill(std::numeric_limits<float>::infinity());
    this->bending_cost_fill(0);

    float bestCostWhenReachTarget = std::numeric_limits<float>::max();
    LocationQueue<Location, float> frontier;  // search frontier
    this->initializeFrontiers(route, frontier);

    std::cout << " frontier.size(): " << frontier.size() << ", current targeted pin:  " << std::endl;
    for (const auto &pt : currentTargetedPinWithLayers) {
        std::cout << "  " << pt << std::endl;
    }

    // int numPopLocation = 0;

    while (!frontier.empty()) {
        Location current = frontier.front();

        // // Debugging
        // numPopLocation++;
        // int prevId = this->getCameFromId(current);
        // Location prev;
        // this->idToLocation(prevId, prev);
        // std::cout << "==>Current pop " << numPopLocation << " at Loc: " << current << ", expanded from Loc: " << prev << ", with Key in queue: " << frontier.frontKey() << std::endl;

        // A* termination
        if (isTargetedPin(current)) {
            bestCostWhenReachTarget = frontier.frontKey();
            finalEnd = current;
            finalCost = bestCostWhenReachTarget;
            std::cout << "=> Find the target: " << current << " with cost at " << bestCostWhenReachTarget << std::endl;
            return;
        }

        frontier.pop();

        std::vector<std::pair<float, Location>> neighbors;
        this->getNeighbors(current, neighbors);
        float current_cost = this->working_cost_at(current);

        for (std::pair<float, Location> &next : neighbors) {
            float new_cost = current_cost + next.first;  // Can be optimized!!!!

            //float estCost = getEstimatedCost(next.second);
            // Test bending cost
            float estCost = getEstimatedCostWithBendingCost(current, next.second);
            int bendCost = getBendingCostOfNext(current, next.second);

            // Test bending cost + multi-layers (3D estimation cost)
            // float estCost = getEstimatedCostWithLayersAndBendingCost(current, next.second);

            if (new_cost + bendCost < this->working_cost_at(next.second) + this->bending_cost_at(next.second)) {
                //if () {
                this->working_cost_set(new_cost, next.second);
                this->bending_cost_set(bendCost, next.second);
                this->setCameFromId(next.second, this->locationToId(current));

                frontier.push(next.second, new_cost + estCost + bendCost);

                // float keyValue = new_cost + estCost + bendCost;
                // std::cout << "Better Cost at Location " << next.second << ", with Cost: " << new_cost << ", est Cost: " << estCost << ", bend Cost: " << bendCost << ", key value: " << keyValue << std::endl;

                // Show if the target is reached
                if (isTargetedPin(next.second)) {
                    std::cout << "Find target with estCost = " << estCost << ", walkedCost = " << new_cost << ", bend Cost: " << bendCost
                              << ", currentLoc: " << current << ", nextLoc: " << next.second << std::endl;
                }
            }
        }
    }
    //For Dijkstra to output
    finalCost = bestCostWhenReachTarget;
    std::cout << "=> Find the target with cost at " << bestCostWhenReachTarget << std::endl;
}

void BoardGrid::aStarSearching(MultipinRoute &route, Location &finalEnd, float &finalCost) {
    std::cout << __FUNCTION__ << "() nets: route.mGridPaths.size() = " << route.mGridPaths.size() << std::endl;

    this->working_cost_fill(std::numeric_limits<float>::infinity());
    this->bending_cost_fill(0);

    float bestCostWhenReachTarget = std::numeric_limits<float>::max();
    LocationQueue<Location, float> frontier;  // search frontier

    // For path to multiple points. Searches from the multiple points to every other point
    this->initializeFrontiers(route, frontier);

    std::cout << " frontier.size(): " << frontier.size() << ", current targeted pin:  " << std::endl;
    for (const auto &pt : currentTargetedPinWithLayers) {
        std::cout << "  " << pt << std::endl;
    }

    while (!frontier.empty()) {
        Location current = frontier.front();

        // A* termination
        if (isTargetedPin(current)) {
            bestCostWhenReachTarget = frontier.frontKey();
            finalEnd = current;
            finalCost = bestCostWhenReachTarget;
            std::cout << "=> Find the target: " << current << " with cost at " << bestCostWhenReachTarget << std::endl;
            return;
        }

        frontier.pop();

        std::vector<std::pair<float, Location>> neighbors;
        this->getNeighbors(current, neighbors);
        float current_cost = this->working_cost_at(current);

        for (std::pair<float, Location> &next : neighbors) {
            float new_cost = current_cost + next.first;  // Can be optimized!!!!

            //float estCost = getEstimatedCost(next.second);
            // Test bending cost
            float estCost = getEstimatedCostWithBendingCost(current, next.second);
            int bendCost = getBendingCostOfNext(current, next.second);
            pr::prIntCost layerPrefCost = getLayerPrefCost(route, next.second);
            new_cost += layerPrefCost;

            // Test bending cost + multi-layers (3D estimation cost)
            // float estCost = getEstimatedCostWithLayersAndBendingCost(current, next.second);

            if (new_cost + bendCost < this->working_cost_at(next.second) + this->bending_cost_at(next.second)) {
                this->working_cost_set(new_cost, next.second);
                this->bending_cost_set(bendCost, next.second);
                this->setCameFromId(next.second, this->locationToId(current));

                frontier.push(next.second, new_cost + estCost + bendCost);

                // Show if the target is reached
                if (isTargetedPin(next.second)) {
                    std::cout << "Find target with estCost = " << estCost << ", walkedCost = " << new_cost << ", bend Cost: " << bendCost
                              << ", currentLoc: " << current << ", nextLoc: " << next.second << std::endl;
                }
            }
        }
    }
}

void BoardGrid::initializeFrontiers(const MultipinRoute &route, LocationQueue<Location, float> &frontier) {
    if (route.getGridPaths().empty()) {
        // First pair of routing
        for (const auto &pt : route.mGridPins.front().pinWithLayers) {
            initializeLocationToFrontier(pt, frontier);
        }
        //     std::cout << " A* Start from: " << std::endl;
        //     for (const auto &pt : route.mGridPins.front().pinWithLayers) {
        //         std::cout << "  " << pt << std::endl;
        //         // Initialize the pin grids' obstacle costs
        //         // For incremental cost update of trace
        //         this->cached_trace_cost_set(sized_trace_cost_at(pt, traceRelativeSearchGrids), pt);
        //     }
        return;
    }

    for (const auto &gp : route.getGridPaths()) {
        const auto &locations = gp.getLocations();
        if (locations.size() == 1) {
            initializeLocationToFrontier(locations.front(), frontier);
            continue;
        }

        auto pointIte = ++locations.begin();
        auto prevPointIte = locations.begin();

        for (; pointIte != locations.end();) {
            // TODO: Through hole pins? how to put layers of through hole pins into frontier
            if (pointIte->m_x == prevPointIte->m_x && pointIte->m_y == prevPointIte->m_y && pointIte->m_z != prevPointIte->m_z) {
                // A via
                if (GlobalParam::gUseMircoVia) {
                    // Micro vias / Blind/buried vias
                    for (int z = std::min(pointIte->m_z, prevPointIte->m_z); z <= std::max(pointIte->m_z, prevPointIte->m_z); ++z) {
                        Location viaLocationOnALayer{pointIte->m_x, pointIte->m_y, z};
                        initializeLocationToFrontier(viaLocationOnALayer, frontier);
                    }
                } else {
                    // Put all the layers (through hole via) into the frontiers
                    for (int z = 0; z < this->l; ++z) {
                        Location viaLocationOnALayer{pointIte->m_x, pointIte->m_y, z};
                        initializeLocationToFrontier(viaLocationOnALayer, frontier);
                    }
                }
            } else {
                // Normal points
                initializeLocationToFrontier(*pointIte, frontier);

                if (prevPointIte == locations.begin()) {
                    initializeLocationToFrontier(*prevPointIte, frontier);
                }
            }

            ++pointIte;
            ++prevPointIte;
        }
    }
}

void BoardGrid::initializeFrontiers(const std::vector<Location> &route, LocationQueue<Location, float> &frontier) {
    if (route.empty()) {
        return;
    }
    if (route.size() == 1) {
        initializeLocationToFrontier(route.front(), frontier);
    }

    for (int i = 1; i < route.size(); ++i) {
        const auto &prevLocation = route.at(i - 1);
        const auto &location = route.at(i);

        // TODO: Through hole pins? how to put layers of through hole pins into frontier
        if (location.m_x == prevLocation.m_x && location.m_y == prevLocation.m_y && location.m_z != prevLocation.m_z) {
            // A via
            if (GlobalParam::gUseMircoVia) {
                // Micro vias / Blind/buried vias
                for (int z = std::min(location.m_z, prevLocation.m_z); z <= std::max(location.m_z, prevLocation.m_z); ++z) {
                    Location viaLocationOnALayer{location.m_x, location.m_y, z};
                    initializeLocationToFrontier(viaLocationOnALayer, frontier);
                }
            } else {
                // Put all the layers (through hole via) into the frontiers
                for (int z = 0; z < this->l; ++z) {
                    Location viaLocationOnALayer{location.m_x, location.m_y, z};
                    initializeLocationToFrontier(viaLocationOnALayer, frontier);
                }
            }
        } else {
            // Normal points
            initializeLocationToFrontier(location, frontier);

            if (i == 1) {
                initializeLocationToFrontier(prevLocation, frontier);
            }
        }
    }
}

void BoardGrid::initializeLocationToFrontier(const Location &start, LocationQueue<Location, float> &frontier) {
    // Walked cost (= 0) + estimated future cost
    // 2D cost estimation
    float cost = getEstimatedCost(start);
    // 3D cost estimation
    //float cost = getEstimatedCostWithLayers(start);

    this->working_cost_set(0.0, start);
    frontier.push(start, cost);
    // std::cerr << "\tPQ: cost: " << cost << ", at" << start << std::endl;

    // Set a ending for the backtracking
    this->setCameFromId(start, this->locationToId(start));
}

float BoardGrid::getEstimatedCost(const Location &l) {
    // return max(abs(l.m_x - this->current_targeted_pin.m_x), abs(l.m_y - this->current_targeted_pin.m_y));

    int absDiffX = abs(l.m_x - this->current_targeted_pin.m_x);
    int absDiffY = abs(l.m_y - this->current_targeted_pin.m_y);
    int minDiff = min(absDiffX, absDiffY);
    int maxDiff = max(absDiffX, absDiffY);
    return (float)minDiff * GlobalParam::gDiagonalCost + maxDiff - minDiff;
}

float BoardGrid::getEstimatedCostWithBendingCost(const Location &current, const Location &next) {
    int currentId = this->locationToId(current);
    int prevId = this->getCameFromId(current);
    float bendingCost = 0;
    if (prevId != currentId) {
        Location prev;
        this->idToLocation(prevId, prev);

        if (prev.z() == current.z() &&
            current.z() == next.z() &&
            prev.x() - current.x() == current.x() - next.x() &&
            prev.y() - current.y() == current.y() - next.y()) {
            bendingCost += 0.5;
        }
    } else {
        // Count the starting point as zero bending
        bendingCost += 0.5;
    }
    if (next.m_x == this->current_targeted_pin.m_x ||
        next.m_y == this->current_targeted_pin.m_y ||
        abs(next.m_x - this->current_targeted_pin.m_x) == abs(next.m_y - this->current_targeted_pin.m_y)) {
        bendingCost += 0.5;
    }

    // return max(abs(current.m_x - this->current_targeted_pin.m_x), abs(current.m_y - this->current_targeted_pin.m_y)) - bendingCost;

    int absDiffX = abs(next.m_x - this->current_targeted_pin.m_x);
    int absDiffY = abs(next.m_y - this->current_targeted_pin.m_y);
    int minDiff = min(absDiffX, absDiffY);
    int maxDiff = max(absDiffX, absDiffY);
    return (float)minDiff * GlobalParam::gDiagonalCost + maxDiff - minDiff - bendingCost;
}

int BoardGrid::getBendingCostOfNext(const Location &current, const Location &next) const {
    int currentBendingCost = this->bending_cost_at(current);
    int currentId = this->locationToId(current);
    int prevId = this->getCameFromId(current);
    int nextBendingCost = currentBendingCost;

    if (prevId != currentId) {
        Location prev;
        this->idToLocation(prevId, prev);

        if (prev.z() == current.z() &&
            current.z() == next.z() &&
            prev.x() - current.x() == current.x() - next.x() &&
            prev.y() - current.y() == current.y() - next.y()) {
        } else {
            nextBendingCost += 1;
        }
    } else {
        // Count the starting point as zero bending
    }

    return nextBendingCost;
}

pr::prIntCost BoardGrid::getLayerPrefCost(const MultipinRoute &route, const Location &pt) const {
    auto layerId = pt.m_z;

    if (layerId >= route.getLayerCosts().size()) {
        std::cerr << __FUNCTION__ << ": Invalid layer Id " << layerId << std::endl;
        return 0;
    } else {
        return route.getLayerCosts().at(layerId);
    }
}

float BoardGrid::getEstimatedCostWithLayers(const Location &l) {
    int absDiffX = abs(l.m_x - this->currentTargetedPinWithLayers.front().m_x);
    int absDiffY = abs(l.m_y - this->currentTargetedPinWithLayers.front().m_y);
    int minDiff = min(absDiffX, absDiffY);
    int maxDiff = max(absDiffX, absDiffY);
    float estCost = (float)minDiff * GlobalParam::gDiagonalCost + maxDiff - minDiff;

    // If is SMD pin, add the layer changing cost
    if (currentTargetedPinWithLayers.size() == 1) {
        estCost += GlobalParam::gLayerChangeCost * abs(this->currentTargetedPinWithLayers.front().m_z - l.m_z);
    }
    return estCost;
}

float BoardGrid::getEstimatedCostWithLayersAndBendingCost(const Location &current, const Location &next) {
    // Bending cost
    int currentId = this->locationToId(current);
    int prevId = this->getCameFromId(current);
    float bendingCost = 0;
    if (prevId != currentId) {
        Location prev;
        this->idToLocation(prevId, prev);

        if (prev.z() == current.z() &&
            current.z() == next.z() &&
            prev.x() - current.x() == current.x() - next.x() &&
            prev.y() - current.y() == current.y() - next.y()) {
            bendingCost = 0.5;
        }
    }

    int absDiffX = abs(next.m_x - this->currentTargetedPinWithLayers.front().m_x);
    int absDiffY = abs(next.m_y - this->currentTargetedPinWithLayers.front().m_y);
    int minDiff = min(absDiffX, absDiffY);
    int maxDiff = max(absDiffX, absDiffY);
    float estCost = (float)minDiff * GlobalParam::gDiagonalCost + maxDiff - minDiff - bendingCost;

    // If is SMD pin, add the layer changing cost
    if (currentTargetedPinWithLayers.size() == 1) {
        estCost += GlobalParam::gLayerChangeCost * abs(this->currentTargetedPinWithLayers.front().m_z - next.m_z);
    }
    return estCost;
}

void BoardGrid::getNeighbors(const Location &l, std::vector<std::pair<float, Location>> &ns) {
    auto &curGridNetclass = mGridNetclasses.at(currentGridNetclassId);
    const auto &traceRelativeSearchGrids = curGridNetclass.getTraceSearchingSpaceToGrids();
    const auto &viaRelativeSearchGrids = curGridNetclass.getViaSearchingSpaceToGrids();

    // For incremental cost update of trace
    // auto currentGridPenalty = this->cached_trace_cost_at(l);

    // left
    if (l.m_x - 1 > -1) {
        Location left{l.m_x - 1, l.m_y, l.m_z};
        float leftCost = 1.0;

        if (this->cached_trace_cost_at(left) == -1) {
            // Radius based searching
            //leftCost += sized_trace_cost_at(left, traceSearchRadius);
            // Vector based searching
            leftCost += sized_trace_cost_at(left, traceRelativeSearchGrids);

            // Incremental searching
            // leftCost += currentGridPenalty;
            // leftCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getLeftAddGrids());
            // leftCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getLeftDedGrids());

            // Put in the cache
            this->cached_trace_cost_set(leftCost - 1.0, left);
        } else {
            leftCost += this->cached_trace_cost_at(left);
        }
        // float golden = sized_trace_cost_at(left, traceRelativeSearchGrids);
        // if(golden != cached_trace_cost_at(left) ){
        //     std::cout << "Cost at "<<left<<": golden: " << golden << ", incremental: " << cached_trace_cost_at(left) << std::endl;
        // }

        // See if is negative cost
        // if (leftCost < 0.0) {
        //     float golden = sized_trace_cost_at(left, traceRelativeSearchGrids);
        //     std::cout << "Cost at " << left << ": golden: " << golden << ", incremental: " << cached_trace_cost_at(left) << std::endl;
        // }
        ns.push_back(std::pair<float, Location>(leftCost, left));
    }

    // right
    if (l.m_x + 1 < this->w) {
        Location right{l.m_x + 1, l.m_y, l.m_z};
        float rightCost = 1.0;

        if (this->cached_trace_cost_at(right) == -1) {
            //rightCost += sized_trace_cost_at(right, traceSearchRadius);
            rightCost += sized_trace_cost_at(right, traceRelativeSearchGrids);

            // Incremental searching
            // rightCost += currentGridPenalty;
            // rightCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getRightAddGrids());
            // rightCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getRightDedGrids());

            // Put in cache
            this->cached_trace_cost_set(rightCost - 1.0, right);
        } else {
            rightCost += this->cached_trace_cost_at(right);
        }
        ns.push_back(std::pair<float, Location>(rightCost, right));
    }

    // forward
    if (l.m_y + 1 < this->h) {
        Location forward{l.m_x, l.m_y + 1, l.m_z};
        float forwardCost = 1.0;

        if (this->cached_trace_cost_at(forward) == -1) {
            //forwardCost += sized_trace_cost_at(forward, traceSearchRadius);
            forwardCost += sized_trace_cost_at(forward, traceRelativeSearchGrids);

            // Incremental searching
            // forwardCost += currentGridPenalty;
            // forwardCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getForwardAddGrids());
            // forwardCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getForwardDedGrids());

            // Put in cache
            this->cached_trace_cost_set(forwardCost - 1.0, forward);
        } else {
            forwardCost += this->cached_trace_cost_at(forward);
        }
        ns.push_back(std::pair<float, Location>(forwardCost, forward));
    }

    // back
    if (l.m_y - 1 > -1) {
        Location backward{l.m_x, l.m_y - 1, l.m_z};
        float backwardCost = 1.0;

        if (this->cached_trace_cost_at(backward) == -1) {
            //backwardCost += sized_trace_cost_at(backward, traceSearchRadius);
            backwardCost += sized_trace_cost_at(backward, traceRelativeSearchGrids);

            // Incremental searching
            // backwardCost += currentGridPenalty;
            // backwardCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getBackwardAddGrids());
            // backwardCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getBackwardDedGrids());

            // Put in cache
            this->cached_trace_cost_set(backwardCost - 1.0, backward);
        } else {
            backwardCost += this->cached_trace_cost_at(backward);
        }
        ns.push_back(std::pair<float, Location>(backwardCost, backward));
    }

    if (GlobalParam::gUseMircoVia) {
        // up
        if (l.m_z + 1 < this->l) {
            Location up{l.m_x, l.m_y, l.m_z + 1};
            float upCost = 0.0;

            sizedViaCostBetweenStartEndLayer(l, l.m_z, l.m_z + 1, viaRelativeSearchGrids, upCost);
            upCost += GlobalParam::gLayerChangeCost;
            ns.push_back(std::pair<float, Location>(upCost, up));

            // Incremental searching
            // this->cached_trace_cost_set(sized_trace_cost_at(up, traceRelativeSearchGrids), up);
        }
        // down
        if (l.m_z - 1 > -1) {
            Location down{l.m_x, l.m_y, l.m_z - 1};
            float downCost = 0.0;

            sizedViaCostBetweenStartEndLayer(l, l.m_z - 1, l.m_z, viaRelativeSearchGrids, downCost);
            downCost += GlobalParam::gLayerChangeCost;
            ns.push_back(std::pair<float, Location>(downCost, down));

            // Incremental searching
            // this->cached_trace_cost_set(sized_trace_cost_at(down, traceRelativeSearchGrids), down);
        }
    } else {
        // Make a through hole via
        float viaCost = 0.0;
        Location viaCachedLocation{l.m_x, l.m_y, 0};
        // Correct Implementation
        // if (sizedViaExpandableAndCost(l, viaRelativeSearchGrids, viaCost)) {
        //     //std::cout << "VIA cached missed at " << viaCachedLocation << ", cost: " << viaCost << std::endl;
        //     viaCost += GlobalParam::gLayerChangeCost;

        //     // Put all the layers (through hole via) into the neighbors
        //     for (int z = 0; z < this->l; ++z) {
        //         Location viaLayer{l.m_x, l.m_y, z};
        //         ns.push_back(std::pair<float, Location>(viaCost, viaLayer));
        //     }
        // }

        // Trying to cached the via cost
        if (this->cached_via_cost_at(viaCachedLocation) < -1.5) {
            // ViaForbidden location, do nothing
        } else {
            if (this->cached_via_cost_at(viaCachedLocation) < -0.5) {
                ++this->viaCachedMissed;

                // For incremental Via cost update
                int currentId = this->locationToId(l);
                int prevId = this->getCameFromId(currentId);
                Location prevLocation;
                this->idToLocation(prevId, prevLocation);
                prevLocation.m_z = 0;  // To access the cache
                auto prevLocViaCost = this->cached_via_cost_at(prevLocation);

                // No cached via cost value - correct implementation
                // if (sizedViaExpandableAndCost(l, viaRelativeSearchGrids, viaCost)) {
                // No cached via cost value => try incremental cost updating
                if (sizedViaExpandableAndIncrementalCost(l, viaRelativeSearchGrids, prevLocation, prevLocViaCost, curGridNetclass.getViaIncrementalSearchGrids(), viaCost)) {
                    // Put in the cache
                    this->cached_via_cost_set(viaCost, viaCachedLocation);

                    viaCost += GlobalParam::gLayerChangeCost;

                    // Put all the layers (through hole via) into the neighbors
                    for (int z = 0; z < this->l; ++z) {
                        Location viaLayer{l.m_x, l.m_y, z};
                        ns.push_back(std::pair<float, Location>(viaCost, viaLayer));
                    }
                } else {
                    // Put in the cache the via forbidden flag
                    this->cached_via_cost_set(-2.0, viaCachedLocation);
                }

            } else {
                ++this->viaCachedHit;

                // Got a cached via cost value
                viaCost = this->cached_via_cost_at(viaCachedLocation) + GlobalParam::gLayerChangeCost;

                // Put all the layers (through hole via) into the neighbors
                for (int z = 0; z < this->l; ++z) {
                    Location viaLayer{l.m_x, l.m_y, z};
                    ns.push_back(std::pair<float, Location>(viaCost, viaLayer));
                }
            }
        }
    }

    /*
    // up
    if (l.m_z + 1 < this->l) {
        Location up{l.m_x, l.m_y, l.m_z + 1};
        float upCost = 0.0;

        // if (this->cached_via_cost_at(up) == -2) {
        //     // ViaForbidden location, do nothing
        // } else if (this->cached_via_cost_at(up) == -1) {
        //     // No cached via cost value
        //     if (sizedViaExpandableAndCost(up, viaRelativeSearchGrids, upCost)) {
        //         // Put in the cache
        //         this->cached_via_cost_set(upCost, up);

        //         upCost += GlobalParam::gLayerChangeCost;
        //         ns.push_back(std::pair<float, Location>(upCost, up));

        //         // For Incremental searching, which assume the previous grid done the cost calculation
        //         // this->cached_trace_cost_set(sized_trace_cost_at(up, traceRelativeSearchGrids), up);
        //     } else {
        //         // Put in the cache the forbidden flag
        //         this->cached_via_cost_set(-2, up);
        //     }
        // } else {
        //     // Got a cached via cost value
        //     upCost += this->cached_via_cost_at(up);
        //     upCost += GlobalParam::gLayerChangeCost;
        // }

        //===Original Implementation===
        if (sizedViaExpandableAndCost(l, viaRelativeSearchGrids, upCost)) {
            upCost += GlobalParam::gLayerChangeCost;
            ns.push_back(std::pair<float, Location>(upCost, up));

            // Incremental searching
            // this->cached_trace_cost_set(sized_trace_cost_at(up, traceRelativeSearchGrids), up);
        }
    }

    // down
    if (l.m_z - 1 > -1) {
        Location down{l.m_x, l.m_y, l.m_z - 1};
        float downCost = 0.0;

        // if (this->cached_via_cost_at(down) == -2) {
        //     // ViaForbidden location, do nothing
        // } else if (this->cached_via_cost_at(down) == -1) {
        //     // No cached via cost value
        //     if (sizedViaExpandableAndCost(down, viaRelativeSearchGrids, downCost)) {
        //         // Put in the cache
        //         this->cached_via_cost_set(downCost, down);

        //         downCost += GlobalParam::gLayerChangeCost;
        //         ns.push_back(std::pair<float, Location>(downCost, down));

        //         // For Incremental searching, which assume the previous grid done the cost calculation
        //         // this->cached_trace_cost_set(sized_trace_cost_at(up, traceRelativeSearchGrids), up);
        //     } else {
        //         // Put in the cache the forbidden flag
        //         this->cached_via_cost_set(-2, down);
        //     }
        // } else {
        //     // Got a cached via cost value
        //     downCost += this->cached_via_cost_at(down);
        //     downCost += GlobalParam::gLayerChangeCost;
        // }

        //===Original Implementation===
        if (sizedViaExpandableAndCost(l, viaRelativeSearchGrids, downCost)) {
            downCost += GlobalParam::gLayerChangeCost;
            ns.push_back(std::pair<float, Location>(downCost, down));

            // Incremental searching
            // this->cached_trace_cost_set(sized_trace_cost_at(down, traceRelativeSearchGrids), down);
        }
    }
    */

    // lf
    if (l.m_x - 1 > -1 && l.m_y + 1 < this->h) {
        Location lf{l.m_x - 1, l.m_y + 1, l.m_z};
        float lfCost = GlobalParam::gDiagonalCost;

        if (this->cached_trace_cost_at(lf) == -1) {
            //lfCost += sized_trace_cost_at(lf, traceSearchRadius);
            lfCost += sized_trace_cost_at(lf, traceRelativeSearchGrids);

            // Incremental searching
            // lfCost += currentGridPenalty;
            // lfCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getLFAddGrids());
            // lfCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getLFDedGrids());

            // Put in cache
            this->cached_trace_cost_set(lfCost - GlobalParam::gDiagonalCost, lf);
        } else {
            lfCost += this->cached_trace_cost_at(lf);
        }

        ns.push_back(std::pair<float, Location>(lfCost, lf));
    }

    // lb
    if (l.m_x - 1 > -1 && l.m_y - 1 > -1) {
        Location lb{l.m_x - 1, l.m_y - 1, l.m_z};
        float lbCost = GlobalParam::gDiagonalCost;

        if (this->cached_trace_cost_at(lb) == -1) {
            //lbCost += sized_trace_cost_at(lb, traceSearchRadius);
            lbCost += sized_trace_cost_at(lb, traceRelativeSearchGrids);

            // Incremental searching
            // lbCost += currentGridPenalty;
            // lbCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getLBAddGrids());
            // lbCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getLBDedGrids());

            // Put in cache
            this->cached_trace_cost_set(lbCost - GlobalParam::gDiagonalCost, lb);
        } else {
            lbCost += this->cached_trace_cost_at(lb);
        }

        ns.push_back(std::pair<float, Location>(lbCost, lb));
    }

    // rf
    if (l.m_x + 1 < this->w && l.m_y + 1 < this->h) {
        Location rf{l.m_x + 1, l.m_y + 1, l.m_z};
        float rfCost = GlobalParam::gDiagonalCost;

        if (this->cached_trace_cost_at(rf) == -1) {
            //rfCost += sized_trace_cost_at(rf, traceSearchRadius);
            rfCost += sized_trace_cost_at(rf, traceRelativeSearchGrids);

            // Incremental searching
            // rfCost += currentGridPenalty;
            // rfCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getRFAddGrids());
            // rfCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getRFDedGrids());

            // Put in cache
            this->cached_trace_cost_set(rfCost - GlobalParam::gDiagonalCost, rf);
        } else {
            rfCost += this->cached_trace_cost_at(rf);
        }

        ns.push_back(std::pair<float, Location>(rfCost, rf));
    }

    // rb
    if (l.m_x + 1 < this->w && l.m_y - 1 > -1) {
        Location rb{l.m_x + 1, l.m_y - 1, l.m_z};
        float rbCost = GlobalParam::gDiagonalCost;

        if (this->cached_trace_cost_at(rb) == -1) {
            //rbCost += sized_trace_cost_at(rb, traceSearchRadius);
            rbCost += sized_trace_cost_at(rb, traceRelativeSearchGrids);

            // Incremental searching
            // rbCost += currentGridPenalty;
            // rbCost += sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getRBAddGrids());
            // rbCost -= sized_trace_cost_at(l, curGridNetclass.getTraceIncrementalSearchGrids().getRBDedGrids());

            // Put in cache
            this->cached_trace_cost_set(rbCost - GlobalParam::gDiagonalCost, rb);
        } else {
            rbCost += this->cached_trace_cost_at(rb);
        }

        ns.push_back(std::pair<float, Location>(rbCost, rb));
    }
}

void BoardGrid::printGnuPlot() {
    float max_val = 0.0;
    for (int i = 0; i < this->size; i += 1) {
        if (this->grid[i].baseCost > max_val) max_val = this->grid[i].baseCost;
    }

    std::cout << "printGnuPlot()::Max Cost: " << max_val << std::endl;

    for (int l = 0; l < this->l; ++l) {
        std::string outFileName = "layer" + std::to_string(l) + "_baseCost.dat";
        outFileName =
            util::appendDirectory(GlobalParam::gOutputFolder, outFileName);
        std::ofstream ofs(outFileName, std::ofstream::out);
        ofs << std::fixed << std::setprecision(5);

        for (int r = 0; r < this->h; ++r) {
            for (int c = 0; c < this->w; ++c) {
                ofs << this->base_cost_at(Location(c, r, l)) << " ";
            }
            ofs << std::endl;
        }
    }
}

void BoardGrid::printMatPlot(const std::string fileNameTag) {
    float maxCost = std::numeric_limits<float>::min();
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < this->size; i += 1) {
        if (this->grid[i].baseCost > maxCost) {
            maxCost = this->grid[i].baseCost;
        } else if (this->grid[i].baseCost < minCost) {
            minCost = this->grid[i].baseCost;
        }
    }

    std::cout << "printMatPlot()::Max Cost: " << maxCost
              << ", Min Cost: " << minCost << std::endl;

    for (int l = 0; l < this->l; ++l) {
        std::string outFileName = fileNameTag + ".layer" + std::to_string(l) + "_baseCost.py";
        outFileName =
            util::appendDirectory(GlobalParam::gOutputFolder, outFileName);
        std::ofstream ofs(outFileName, std::ofstream::out);
        std::cout << "outFileName: " << outFileName << std::endl;

        ofs << std::fixed << std::setprecision(5);
        ofs << "import numpy as np\n";
        ofs << "import matplotlib.pyplot as plt\n";
        ofs << "plt.close()\n";
        //ofs << "viridis = plt.get_cmap('nipy_spectral', " << int(maxCost) << ")\n";
        ofs << "viridis = plt.get_cmap('nipy_spectral', " << 500 << ")\n";
        ofs << "data = np.array([[";

        for (int r = 0; r < this->h; ++r) {
            //for (int r = this->h - 1; r > -1; --r) {
            for (int c = 0; c < this->w; ++c) {
                ofs << this->base_cost_at(Location(c, r, l)) << " ";
                if (c < this->w - 1) {
                    ofs << ", ";
                } else {
                    ofs << "]";
                }
            }

            if (r < this->h - 1) {
                //if (r > 0) {
                ofs << ", [";
            }
        }

        ofs << "])\n";
        //ofs << "plt.pcolormesh(data, cmap=viridis, vmin=data.min(), vmax=data.max())\n";
        ofs << "plt.pcolormesh(data, cmap=viridis, vmin=data.min(), vmax=500)\n";
        ofs << "plt.title('"
            << "layer" << std::to_string(l) << " basecost"
            << "')\n";
        ofs << "plt.colorbar()\n";
        ofs << "plt.axis('equal')\n";
        ofs << "plt.gca().invert_yaxis()\n";
        ofs << "plt.show()\n";
    }
}

// void BoardGrid::pprint() {
//     char colors[11] = " .,:=+*#%@";
//     // long
//     // colors[36]    = {' ','.',':','░','#','▒','▓','█'};
//     int num_colors = 10;

//     float max_val = 0.0;
//     for (int i = 0; i < this->size; i += 1) {
//         if (this->grid[i].baseCost > max_val) max_val = this->grid[i].baseCost;
//     }

//     for (int l = 0; l < this->l; l += 1) {
//         std::cout << std::endl
//                   << "Layer: " << l << std::endl;

//         for (int r = 0; r < this->h; r += 1) {
//             for (int c = 0; c < this->w; c += 1) {
//                 int current_color = this->base_cost_at(Location(c, r, l)) /
//                                     max_val * num_colors;
//                 std::cout << colors[current_color] << " ";
//                 // std::cout << current_color << " ";

//                 // std::cout << std::setw(6) << std::setprecision(2) <<
//                 // std::right << this->at(c, r, l);
//             }
//             std::cout << std::endl;
//         }
//     }
// }

bool BoardGrid::sizedViaExpandableAndCost(const Location &l, const int viaRadius, float &cost) const {
    int radius = viaRadius;
    cost = 0.0;
    for (int z = 0; z < this->l; ++z) {
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                Location current_l = Location(l.m_x + x, l.m_y + y, z);
                if (!validate_location(current_l)) {
                    cost += GlobalParam::gViaTouchBoundaryCost;
                    continue;
                }
                if (this->isViaForbidden(current_l)) {
                    return false;
                }
                //cost += this->via_cost_at(current_l);
                cost += this->base_cost_at(current_l);
            }
        }
    }
    return true;
}

bool BoardGrid::sizedViaExpandableAndCost(const Location &l, const std::vector<Point_2D<int>> &viaRelativeSearchGrids, float &cost) const {
    cost = 0.0;
    // Check through hole via
    for (int z = 0; z < this->l; ++z) {
        for (const auto &gridPt : viaRelativeSearchGrids) {
            Location current_l = Location(l.m_x + gridPt.x(), l.m_y + gridPt.y(), z);
            if (!validate_location(current_l)) {
                cost += GlobalParam::gViaTouchBoundaryCost;
                continue;
            }
            if (this->isViaForbidden(current_l)) {
                //return false;
                cost += GlobalParam::gViaForbiddenCost;
                continue;
            }
            //cost += this->via_cost_at(current_l);
            cost += this->base_cost_at(current_l);
        }
    }
    return true;
}

void BoardGrid::sizedViaCostBetweenStartEndLayer(const Location &l, const int startLayerId, const int endLayerId, const std::vector<Point_2D<int>> &viaRelativeSearchGrids, float &cost) const {
    cost = 0.0;
    int start = std::min(startLayerId, endLayerId);
    int end = std::max(startLayerId, endLayerId);
    // Check through hole via
    for (int z = start; z <= end; ++z) {
        for (const auto &gridPt : viaRelativeSearchGrids) {
            Location current_l = Location(l.m_x + gridPt.x(), l.m_y + gridPt.y(), z);
            if (!validate_location(current_l)) {
                cost += GlobalParam::gViaTouchBoundaryCost;
                continue;
            }
            if (this->isViaForbidden(current_l)) {
                //return false;
                cost += GlobalParam::gViaForbiddenCost;
                continue;
            }
            cost += this->base_cost_at(current_l);
        }
    }
}

bool BoardGrid::sizedViaExpandableAndIncrementalCost(const Location &curLoc, const std::vector<Point_2D<int>> &viaRelativeSearchGrids, const Location &prevLoc, const float &prevCost, const IncrementalSearchGrids &searchGrids, float &cost) const {
    if (prevCost < -0.5) {
        // Cache missed or the previous location is via forbidded
        // i.e. Need to calculate the cost directly
        if (!sizedViaExpandableAndCost(curLoc, viaRelativeSearchGrids, cost)) {
            return false;
        }
    } else {
        // Has previous location's via cost
        // i.e. use incremental cost to get the current location's via cost
        cost = prevCost;
        float addCost = 0.0, dedCost = 0.0;
        // Left
        if ((curLoc.m_x - prevLoc.m_x) == -1 && curLoc.m_y == prevLoc.m_y) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getLeftAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getLeftDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! Left: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // Right
        else if ((curLoc.m_x - prevLoc.m_x) == 1 && curLoc.m_y == prevLoc.m_y) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getRightAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getRightDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! Right: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // Forward
        else if (curLoc.m_x == prevLoc.m_x && (curLoc.m_y - prevLoc.m_y) == 1) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getForwardAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getForwardDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! Forward: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // Backward
        else if (curLoc.m_x == prevLoc.m_x && (curLoc.m_y - prevLoc.m_y) == -1) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getBackwardAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getBackwardDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! Backward: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // LF
        else if ((curLoc.m_x - prevLoc.m_x) == -1 && (curLoc.m_y - prevLoc.m_y) == 1) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getLFAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getLFDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! LF: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // LB
        else if ((curLoc.m_x - prevLoc.m_x) == -1 && (curLoc.m_y - prevLoc.m_y) == -1) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getLBAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getLBDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! LB: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // RF
        else if ((curLoc.m_x - prevLoc.m_x) == 1 && (curLoc.m_y - prevLoc.m_y) == 1) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getRFAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getRFDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! RF: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        }
        // RB
        else if ((curLoc.m_x - prevLoc.m_x) == 1 && (curLoc.m_y - prevLoc.m_y) == -1) {
            if (sizedViaExpandableAndCost(prevLoc, searchGrids.getRBAddGrids(), addCost)) {
                cost += addCost;
                if (sizedViaExpandableAndCost(prevLoc, searchGrids.getRBDedGrids(), dedCost)) {
                    cost -= dedCost;
                } else {
                    std::cout << "!!!! RB: Errors, the deduction part has via forbidden flag !!!!" << std::endl;
                }
            } else {
                return false;
            }
        } else {
            std::cout << __FUNCTION__ << "(): Cannot find the relationship between current and previous locaiton...." << std::endl;
        }
    }
    return true;
}

float BoardGrid::sized_trace_cost_at(const Location &l, const std::vector<Point_2D<int>> &traRelativeSearchGrids) const {
    float cost = 0.0;
    for (const auto &gridPt : traRelativeSearchGrids) {
        Location current_l = Location(l.m_x + gridPt.x(), l.m_y + gridPt.y(), l.m_z);
        if (!validate_location(current_l)) {
            cost += GlobalParam::gTraceTouchBoundaryCost;
            continue;
        }
        cost += this->base_cost_at(current_l);
        // cost += this->via_cost_at(current_l);
    }
    return cost;
}

float BoardGrid::sized_trace_cost_at(const Location &l, int traceRadius) const {
    int radius = traceRadius;
    float cost = 0.0;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            Location current_l = Location(l.m_x + x, l.m_y + y, l.m_z);
            if (!validate_location(current_l)) {
                cost += GlobalParam::gTraceTouchBoundaryCost;
                continue;
            }
            cost += this->base_cost_at(current_l);
            // cost += this->via_cost_at(current_l);
        }
    }
    return cost;
}

// void BoardGrid::print_came_from(
//     const std::unordered_map<Location, Location> &came_from,
//     const Location &end) {
//     // Location end(ex, ey, ez);

//     std::cout << "Came froms: " << std::endl;
//     std::cout << "\tsize: " << came_from.size() << std::endl;

//     for (int l = 0; l < this->l; l += 1) {
//         std::cout << std::endl
//                   << "Layer: " << l << std::endl;

//         for (int r = 0; r < this->h; r += 1) {
//             for (int c = 0; c < this->w; c += 1) {
//                 Location current(c, r, l);

//                 if (current == end) {
//                     std::cout << "# ";
//                     continue;  // goal
//                 }

//                 if (came_from.find(current) == came_from.end()) {
//                     std::cout << ". ";  // not found
//                     continue;           // unexplored
//                 }

//                 Location cf = came_from.find(current)->second;

//                 if (current.m_x > cf.m_x) {
//                     std::cout << "> ";
//                 } else if (current.m_x < cf.m_x) {
//                     std::cout << "< ";
//                 } else if (current.m_y > cf.m_y) {
//                     std::cout << "V ";
//                 } else if (current.m_y < cf.m_y) {
//                     std::cout << "^ ";
//                 } else if (current.m_z > cf.m_z) {
//                     std::cout << "X ";
//                 } else if (current.m_z < cf.m_z) {
//                     std::cout << "O ";
//                 } else if ((current.m_x == cf.m_x) && (current.m_y == cf.m_y) &&
//                            (current.m_z == cf.m_z)) {
//                     std::cout << "* ";  // start
//                 }
//             }
//             std::cout << std::endl;
//         }
//     }
// }

void BoardGrid::add_via_cost(const Location &l, const int layer, const float cost, const int viaRadius) {
    int radius = viaRadius;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
#ifdef BOUND_CHECKS
            assert(((l.m_x + x) + (l.m_y + y) * this->w + (layer) * this->w * this->h) < this->size);
#endif
            //this->grid[(l.m_x + x) + (l.m_y + y) * this->w + (layer) * this->w * this->h].viaCost += cost;
            this->grid[(l.m_x + x) + (l.m_y + y) * this->w + (layer) * this->w * this->h].baseCost += cost;
        }
    }
}

void BoardGrid::add_via_cost(const Location &l, const int layer, const float cost, const std::vector<Point_2D<int>> &viaShapeToGrids) {
    for (const auto &relativePt : viaShapeToGrids) {
#ifdef BOUND_CHECKS
        assert(((l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (layer) * this->w * this->h) < this->size);
#endif
        //this->grid[(l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (layer) * this->w * this->h].viaCost += cost;
        this->grid[(l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (layer) * this->w * this->h].baseCost += cost;
    }
}

void BoardGrid::remove_route_from_base_cost(const MultipinRoute &route) {
    auto &curGridNetclass = mGridNetclasses.at(route.getGridNetclassId());
    int traceExpandingRadius = curGridNetclass.getTraceExpansion();
    int traceDiagonalExpandingRadius = curGridNetclass.getDiagonalTraceExpansion();
    int viaExpandingRadius = curGridNetclass.getViaExpansion();

    // Old API below
    // add_route_to_base_cost(route, traceExpandingRadius, -GlobalParam::gTraceBasicCost, viaExpandingRadius, -GlobalParam::gViaInsertionCost);

    for (const auto &path : route.getGridPaths()) {
        addGridPathToBaseCost(path, route.getGridNetclassId(), traceExpandingRadius, traceDiagonalExpandingRadius,
                              route.getCurNegTrackObstacleCost(), viaExpandingRadius, route.getCurNegViaObstacleCost());
    }
}

void BoardGrid::add_route_to_base_cost(const MultipinRoute &route) {
    auto &curGridNetclass = mGridNetclasses.at(route.getGridNetclassId());
    int traceExpandingRadius = curGridNetclass.getTraceExpansion();
    int traceDiagonalExpandingRadius = curGridNetclass.getDiagonalTraceExpansion();
    int viaExpandingRadius = curGridNetclass.getViaExpansion();

    // Old API below
    // add_route_to_base_cost(route, traceExpandingRadius, GlobalParam::gTraceBasicCost, viaExpandingRadius, GlobalParam::gViaInsertionCost);

    for (const auto &path : route.getGridPaths()) {
        addGridPathToBaseCost(path, route.getGridNetclassId(), traceExpandingRadius, traceDiagonalExpandingRadius,
                              route.getCurTrackObstacleCost(), viaExpandingRadius, route.getCurViaObstacleCost());
    }
}

void BoardGrid::add_route_to_base_cost(const MultipinRoute &route, const int traceRadius, const float traceCost, const int viaRadius, const float viaCost) {
    if (route.features.empty())
        return;

    //cout << __FUNCTION__ << "(): traceCost: " << traceCost << ", viaCost: " << viaCost << std::endl;

    // Add costs for traces
    for (const auto &l : route.features) {
        for (int current_radius = 0; current_radius <= traceRadius; ++current_radius) {
            for (int r = l.m_y - current_radius; r <= l.m_y + current_radius; ++r) {
                if (r < 0) continue;
                if (r >= this->h) continue;
                for (int c = l.m_x - current_radius; c <= l.m_x + current_radius; ++c) {
                    if (c < 0) continue;
                    if (c >= this->w) continue;
                    // std::cout << "\tsetting cost at " << Location(c, r, layer) << std::endl;
                    this->base_cost_add(traceCost, Location(c, r, l.m_z));
                }
            }
        }
    }

    if (route.features.size() < 2)
        return;

    // Add costs for vias
    // TODO:: Currently handle THROUGH VIA only
    for (int i = 1; i < route.features.size(); ++i) {
        const auto &prevLoc = route.features.at(i - 1);
        const auto &curLoc = route.features.at(i);

        if ((prevLoc.m_z != curLoc.m_z) && (prevLoc.m_x == curLoc.m_x) && (prevLoc.m_y == curLoc.m_y)) {
            for (int z = 0; z < this->l; ++z) {
                //this->add_via_cost(prevLoc, z, viaCost, viaRadius);

                // Via shape to grids
                const auto &gridNc = this->getGridNetclass(route.getGridNetclassId());
                this->add_via_cost(prevLoc, z, viaCost, gridNc.getViaShapeToGrids());
            }
        }
    }
}

void BoardGrid::addGridPathToBaseCost(const GridPath &path, const int gridNetclassId, const int traceRadius, const int diagonalTraceRadius, const float traceCost, const int viaRadius, const float viaCost) {
    const auto &segs = path.getSegments();
    if (segs.empty())
        return;

    //cout << __FUNCTION__ << "(): traceCost: " << traceCost << ", viaCost: " << viaCost << std::endl;

    // Add costs for traces
    auto pointIte = segs.begin();
    auto nextPointIte = ++segs.begin();

    for (; nextPointIte != segs.end();) {
        // Watch out the special case for Via over here, where pointIte->x()=nextPointIte->x() && pointIte->y()=nextPointIte->y()
        // Vertical
        if (pointIte->x() == nextPointIte->x() && pointIte->y() != nextPointIte->y()) {
            auto curX = pointIte->x();
            auto curZ = pointIte->z();
            auto startY = min(pointIte->y(), nextPointIte->y());
            auto endY = max(pointIte->y(), nextPointIte->y());
            // Central Line
            for (auto curY = startY; curY <= endY; ++curY) {
                this->base_cost_add(traceCost, Location(curX, curY, curZ));
            }
            // Extended Line
            for (int curRadius = 1; curRadius <= traceRadius; ++curRadius) {
                for (auto curY = startY; curY <= endY; ++curY) {
                    Location loc{curX + curRadius, curY, curZ};
                    if (this->validate_location(loc)) {
                        this->base_cost_add(traceCost, loc);
                    }
                    Location loc2{curX - curRadius, curY, curZ};
                    if (this->validate_location(loc2)) {
                        this->base_cost_add(traceCost, loc2);
                    }
                }
            }
        }
        // Horizontal
        else if (pointIte->x() != nextPointIte->x() && pointIte->y() == nextPointIte->y()) {
            auto curY = pointIte->y();
            auto curZ = pointIte->z();
            auto startX = min(pointIte->x(), nextPointIte->x());
            auto endX = max(pointIte->x(), nextPointIte->x());
            // Central Line
            for (auto curX = startX; curX <= endX; ++curX) {
                this->base_cost_add(traceCost, Location(curX, curY, curZ));
            }
            // Extended Line
            for (int curRadius = 1; curRadius <= traceRadius; ++curRadius) {
                for (auto curX = startX; curX <= endX; ++curX) {
                    Location loc{curX, curY + curRadius, curZ};
                    if (this->validate_location(loc)) {
                        this->base_cost_add(traceCost, loc);
                    }
                    Location loc2{curX, curY - curRadius, curZ};
                    if (this->validate_location(loc2)) {
                        this->base_cost_add(traceCost, loc2);
                    }
                }
            }
        }
        // LL -> UR || UR -> LL
        else if ((pointIte->x() < nextPointIte->x() && pointIte->y() < nextPointIte->y()) ||
                 (pointIte->x() > nextPointIte->x() && pointIte->y() > nextPointIte->y())) {
            auto curZ = pointIte->z();
            auto startX = min(pointIte->x(), nextPointIte->x());
            auto endX = max(pointIte->x(), nextPointIte->x());
            auto startY = min(pointIte->y(), nextPointIte->y());
            auto endY = max(pointIte->y(), nextPointIte->y());
            // Central Line
            for (int curX = startX, curY = startY; curX <= endX && curY <= endY; ++curX, ++curY) {
                this->base_cost_add(traceCost, Location(curX, curY, curZ));
            }
            // Extended Line
            for (int curRadius = 1; curRadius <= diagonalTraceRadius; ++curRadius) {
                for (int curX = startX, curY = startY; curX <= endX && curY <= endY; ++curX, ++curY) {
                    Location loc{curX + curRadius, curY - curRadius, curZ};
                    if (this->validate_location(loc)) {
                        this->base_cost_add(traceCost, loc);
                    }
                    Location loc2{curX - curRadius, curY + curRadius, curZ};
                    if (this->validate_location(loc2)) {
                        this->base_cost_add(traceCost, loc2);
                    }
                    if (curX < endX && curY < endY) {
                        Location loc3{curX + curRadius, curY - curRadius + 1, curZ};
                        if (this->validate_location(loc3)) {
                            this->base_cost_add(traceCost, loc3);
                        }
                        Location loc4{curX - curRadius + 1, curY + curRadius, curZ};
                        if (this->validate_location(loc4)) {
                            this->base_cost_add(traceCost, loc4);
                        }
                    }
                }
            }
        }
        // UL -> LR || LR -> UL
        else if ((pointIte->x() < nextPointIte->x() && pointIte->y() > nextPointIte->y()) ||
                 (pointIte->x() > nextPointIte->x() && pointIte->y() < nextPointIte->y())) {
            auto curZ = pointIte->z();
            auto startX = min(pointIte->x(), nextPointIte->x());
            auto endX = max(pointIte->x(), nextPointIte->x());
            auto startY = min(pointIte->y(), nextPointIte->y());
            auto endY = max(pointIte->y(), nextPointIte->y());
            // Central Line
            for (int curX = startX, curY = endY; curX <= endX && curY >= startY; ++curX, --curY) {
                this->base_cost_add(traceCost, Location(curX, curY, curZ));
            }
            // Extended Line
            for (int curRadius = 1; curRadius <= diagonalTraceRadius; ++curRadius) {
                for (int curX = startX, curY = endY; curX <= endX && curY >= startY; ++curX, --curY) {
                    Location loc{curX + curRadius, curY + curRadius, curZ};
                    if (this->validate_location(loc)) {
                        this->base_cost_add(traceCost, loc);
                    }
                    Location loc2{curX - curRadius, curY - curRadius, curZ};
                    if (this->validate_location(loc2)) {
                        this->base_cost_add(traceCost, loc2);
                    }
                    if (curX < endX && curY > startY) {
                        Location loc3{curX + curRadius, curY + curRadius - 1, curZ};
                        if (this->validate_location(loc3)) {
                            this->base_cost_add(traceCost, loc3);
                        }
                        Location loc4{curX - curRadius + 1, curY - curRadius, curZ};
                        if (this->validate_location(loc4)) {
                            this->base_cost_add(traceCost, loc4);
                        }
                    }
                }
            }
        }

        // Move to the next segment
        ++pointIte;
        ++nextPointIte;
    }

    // Handle trace end
    for (pointIte = segs.begin(); pointIte != segs.end(); ++pointIte) {
        const auto &gridNc = this->getGridNetclass(gridNetclassId);
        this->base_cost_add(traceCost, *pointIte, gridNc.getTraceEndShapeToGrids());
    }

    if (segs.size() < 2)
        return;

    // Handle Vias
    pointIte = segs.begin();
    nextPointIte = ++segs.begin();

    for (; nextPointIte != segs.end();) {
        if (pointIte->x() == nextPointIte->x() && pointIte->y() == nextPointIte->y() &&
            pointIte->z() != nextPointIte->z()) {
            if (GlobalParam::gUseMircoVia) {
                // Handle Micro/Blind/Buried vias
                for (int z = std::min(pointIte->z(), nextPointIte->z()); z <= std::max(pointIte->z(), nextPointIte->z()); ++z) {
                    // Via shape to grids
                    const auto &gridNc = this->getGridNetclass(gridNetclassId);
                    this->add_via_cost(*pointIte, z, viaCost, gridNc.getViaShapeToGrids());
                }
            } else {
                // Handle Through Hole Via
                for (int z = 0; z < this->l; ++z) {
                    // Via shape to grids
                    const auto &gridNc = this->getGridNetclass(gridNetclassId);
                    this->add_via_cost(*pointIte, z, viaCost, gridNc.getViaShapeToGrids());
                }
            }
        }

        // Move to the next segment
        ++pointIte;
        ++nextPointIte;
    }
}

// void BoardGrid::came_from_to_features(
//     const std::unordered_map<Location, Location> &came_from,
//     const Location &end, std::vector<Location> &features) const {
//     // std::vector<Location> features;
//     // features.insert(end);
//     std::cout << "Starting came_from_to_features " << std::endl;

//     if (!this->validate_location(end))
//         std::cout << "Bad end for came_from_to_features" << std::endl;

//     features.push_back(end);
//     Location current = end;

//     while (came_from.find(current) != came_from.end()) {  // while not the start
//         Location next = came_from.find(current)->second;
//         // if (features.find(next) != features.end()) { // next already in
//         // features, loops or start break;
//         // }
//         if (next == current) {
//             break;
//         }
//         // features.insert(next);
//         features.push_back(next);
//         current = next;
//     }

//     // ?????????
//     // for (Location l : features)
//     // {
//     // 	//std::cout << "\t" << l << std::endl;
//     // 	if (l.m_z > 2)
//     // 	{
//     // 		exit(-1);
//     // 	}
//     // }

//     std::cout << "Finished came_from_to_features " << std::endl;

//     // return features;
// }

// void BoardGrid::came_from_to_features(const Location &end,
//                                       std::vector<Location> &features) const {
//     std::cout << "Starting came_from_to_features ID" << std::endl;

//     if (!this->validate_location(end)) {
//         std::cout << "Bad end for came_from_to_features ID" << std::endl;
//     }

//     features.push_back(end);
//     Location current = end;
//     int currentId = this->locationToId(current);
//     int nextId = this->getCameFromId(currentId);

//     while (nextId != -1) {
//         if (nextId == currentId) {
//             break;
//         }

//         Location next;
//         this->idToLocation(nextId, next);

//         features.push_back(next);
//         currentId = nextId;
//         nextId = this->getCameFromId(currentId);
//     }

//     std::cout << "Finished came_from_to_features ID" << std::endl;
// }

void BoardGrid::backtrackingToGridPath(const Location &end, MultipinRoute &route) const {
    std::cout << __FUNCTION__ << ": Starting backtracking and create new GridPath" << std::endl;

    if (!this->validate_location(end)) {
        std::cout << __FUNCTION__ << "Bad final end" << std::endl;
    }

    GridPath &gp = route.getNewGridPath();

    gp.addLocation(end);
    // features.push_back(end);
    Location current = end;
    int currentId = this->locationToId(current);
    int nextId = this->getCameFromId(currentId);

    while (nextId != -1) {
        if (nextId == currentId) {
            break;
        }

        Location next;
        this->idToLocation(nextId, next);

        // features.push_back(next);
        gp.addLocation(next);
        currentId = nextId;
        nextId = this->getCameFromId(currentId);
    }

    std::cout << __FUNCTION__ << ": End of backtracking and create new GridPath" << std::endl;
}

// std::vector<Location> BoardGrid::came_from_to_features(
//     const std::unordered_map<Location, Location> &came_from,
//     const Location &end) const {
//     std::vector<Location> features;
//     // features.insert(end);
//     this->came_from_to_features(came_from, end, features);
//     return features;
// }

// void BoardGrid::print_route(
//     const std::unordered_map<Location, Location> &came_from,
//     const Location &end) {
//     // Location end(ex, ey, ez);

//     // std::cout << "end: " << end << std::endl;

//     // std::cout << "came_from: " << std::endl;
//     // for (std::pair<Location, Location> l : came_from) {
//     // 	std::cout << l.first << ", " << l.second << std::endl;
//     // }

//     std::vector<Location> features;
//     features = this->came_from_to_features(came_from, end);
//     // features.insert(end);
//     // Location current = end;

//     // std::cout << "current: " << current << std::endl;

//     // std::cout << "Finding features" << std::endl;
//     // while (came_from.find(current) != came_from.end()) { // while not the
//     // start

//     // 	Location next = came_from.find(current)->second;
//     // 	// std::cout << "next: " << next << std::endl;

//     // 	if (features.find(next) != features.end()) { // next already in
//     // features, loops or start 		break;
//     // 	}
//     // 	features.insert(next);
//     // 	// std::cout << "Got next features: " << next <<
//     // 	 std::endl;
//     // 	current = next;
//     // }

//     std::cout << "Printing features" << std::endl;
//     for (int l = 0; l < this->l; l += 1) {
//         std::cout << std::endl
//                   << "Layer: " << l << std::endl;

//         for (int r = 0; r < this->h; r += 1) {
//             for (int c = 0; c < this->w; c += 1) {
//                 Location current(c, r, l);

//                 if (std::find(features.begin(), features.end(), current) !=
//                     features.end()) {
//                     // if (features.find(current) != features.end()) {
//                     std::cout << "# ";  // contains feature
//                 } else {
//                     std::cout << ". ";
//                 }
//             }
//             std::cout << std::endl;
//         }
//     }
// }

// void BoardGrid::print_features(std::vector<Location> features) {
//     std::cout << "Printing features" << std::endl;
//     for (int l = 0; l < this->l; l += 1) {
//         std::cout << std::endl
//                   << "Layer: " << l << std::endl;

//         for (int r = 0; r < this->h; r += 1) {
//             for (int c = 0; c < this->w; c += 1) {
//                 Location current(c, r, l);

//                 if (std::find(features.begin(), features.end(), current) !=
//                     features.end()) {
//                     // if (features.find(current) != features.end()) {
//                     std::cout << "# ";  // contains feature
//                 } else {
//                     std::cout << ". ";
//                 }
//             }
//             std::cout << std::endl;
//         }
//     }
// }

void BoardGrid::addRouteWithGridPins(MultipinRoute &route) {
    std::cout << __FUNCTION__ << "() route.gridPins.size: " << route.mGridPins.size() << std::endl;

    if (route.mGridPins.size() <= 1) return;

    // Clear and initialize
    this->clearAllCameFromId();
    this->cached_trace_cost_fill(-1);
    this->cached_via_cost_fill(-1);
    route.currentRouteCost = 0.0;

    for (size_t i = 1; i < route.mGridPins.size(); ++i) {
        // For early break
        this->setTargetedPins(route.mGridPins.at(i).pinWithLayers);
        // For 2D cost estimation (cares about x and y only)
        current_targeted_pin = route.mGridPins.at(i).pinWithLayers.front();
        // For 3D cost estimation
        currentTargetedPinWithLayers = route.mGridPins.at(i).pinWithLayers;

        Location finalEnd{0, 0, 0};
        float routeCost = 0.0;

        this->aStarSearching(route, finalEnd, routeCost);
        route.currentRouteCost += routeCost;

        // TODO Fix this, when THROUGH PAD as a start?
        this->backtrackingToGridPath(finalEnd, route);

        // Reset temporary stuff
        // For early break
        this->clearTargetedPins(route.mGridPins.at(i).pinWithLayers);
        // For 2D cost estimation
        current_targeted_pin = Location{0, 0, 0};
        // For 3D cost estimation
        currentTargetedPinWithLayers.clear();
    }
    // Convert from grid locations to grid paths
    route.gridPathLocationsToSegments();
    this->add_route_to_base_cost(route);
}

void BoardGrid::ripup_route(MultipinRoute &route) {
    std::cout << "Doing ripup" << std::endl;
    this->remove_route_from_base_cost(route);
    route.clearGridPaths();
    std::cout << "Finished ripup" << std::endl;
}

void BoardGrid::addGridNetclass(const GridNetclass &gridNetclass) {
    this->mGridNetclasses.push_back(gridNetclass);
}

const GridNetclass &BoardGrid::getGridNetclass(const int gridNetclassId) {
    if (gridNetclassId < 0 || gridNetclassId > this->mGridNetclasses.size()) {
        std::cerr << "Illegal grid netclass id: " << gridNetclassId << std::endl;
        return this->mGridNetclasses.front();
    } else {
        return this->mGridNetclasses.at(gridNetclassId);
    }
}
