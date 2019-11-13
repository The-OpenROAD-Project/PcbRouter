// BoardGrid.cpp
#include "BoardGrid.h"

void GridPath::removeRedundantPoints() {
    if (this->mSegments.size() <= 2) {
        return;
    }

    // std::cout << "All pts:" << std::endl;
    // for (auto pt : this->mSegments) {
    //     std::cout << pt << std::endl;
    // }
    // std::cout << "End of All pts:" << std::endl;

    std::vector<std::list<Location>::iterator> pointsToRemove;
    auto pointIte = ++this->mSegments.begin();
    auto prevPointIte = this->mSegments.begin();
    auto nextPointIte = pointIte;
    ++nextPointIte;

    for (; nextPointIte != this->mSegments.end();) {
        // std::cout << "prev: " << *prevPointIte << ", pt: " << *pointIte << ",next: " << *nextPointIte << std::endl;
        // Watch out the special case for Via over here
        if (pointIte->m_x - prevPointIte->m_x == nextPointIte->m_x - pointIte->m_x &&
            pointIte->m_y - prevPointIte->m_y == nextPointIte->m_y - pointIte->m_y) {
            // std::cerr << *pointIte << "<= Remove this point" << std::endl;
            pointsToRemove.push_back(pointIte);
        } else {
            //std::cerr << *pointIte << std::endl;
        }
        ++pointIte;
        ++nextPointIte;
        ++prevPointIte;
    }

    for (auto &removePt : pointsToRemove) {
        this->mSegments.erase(removePt);
    }
}

void MultipinRoute::featuresToGridPaths() {
    if (this->features.empty() || this->features.size() == 1) {
        cerr << __FUNCTION__ << "(): No features to translate to segments. Features.size(): " << this->features.size() << std::endl;
        return;
    }

    // std::cout << "Starting of " << __FUNCTION__ << "() ..." << std::endl;

    // Clean up
    this->mGridPaths.clear();

    if (this->features.size() == 2) {
        auto &path = this->getNewGridPath();
        for (auto &location : this->features) {
            path.mSegments.emplace_back(location);
        }
    }

    // Handle this->features.size() > 2
    // New start of a path
    this->mGridPaths.push_back(GridPath{});
    mGridPaths.back().mSegments.emplace_back(this->features.front());

    // Debuging
    // for (auto &feature : this->features) {
    //     std::cout << feature << std::endl;
    // }

    // 1. Separate the features into paths
    for (int i = 1; i < this->features.size(); ++i) {
        auto prevLocation = this->features.at(i - 1);
        auto location = this->features.at(i);

        if (abs(location.m_x - prevLocation.m_x) <= 1 &&
            abs(location.m_y - prevLocation.m_y) <= 1 &&
            abs(location.m_z - prevLocation.m_z) <= 1) {
            // Sanity Check
            if (location.m_z != prevLocation.m_z &&
                location.m_y != prevLocation.m_y &&
                location.m_x != prevLocation.m_x) {
                std::cerr << __FUNCTION__ << "() Invalid path between location: " << location << ", and prevLocation: " << prevLocation << std::endl;
                continue;
            }
            mGridPaths.back().mSegments.emplace_back(location);
        } else {
            // New start of a path
            this->mGridPaths.push_back(GridPath{});
            mGridPaths.back().mSegments.emplace_back(location);
        }
    }

    // std::cout << __FUNCTION__ << "(): # paths: " << this->mGridPaths.size() << std::endl;

    // 2. Remove Redundant points in paths
    for (auto &path : this->mGridPaths) {
        path.removeRedundantPoints();
    }

    // std::cout << "End of " << __FUNCTION__ << "()" << std::endl;
}

void BoardGrid::initilization(int w, int h, int l) {
    this->w = w;
    this->h = h;
    this->l = l;
    this->size = w * h * l;

    assert(this->grid == nullptr);
    this->grid = new GridCell[this->size];
    assert(this->grid != nullptr);

    this->base_cost_fill(0.0);
    this->via_cost_fill(0.0);
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

void BoardGrid::via_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        this->grid[i].viaCost = value;
    }
}

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
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost;
}

float BoardGrid::working_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].workingCost;
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

void BoardGrid::working_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].workingCost = value;
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
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost = value;
}

void BoardGrid::via_cost_add(const float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost += value;
}

void BoardGrid::setTargetedPin(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].targetedPin = true;
}

void BoardGrid::clearTargetedPin(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].targetedPin = false;
}

bool BoardGrid::isTargetedPin(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].targetedPin;
}

void BoardGrid::setTargetedPins(const std::vector<Location> &pins) {
    for (auto pin : pins) {
        this->setTargetedPin(pin);
    }
}
void BoardGrid::clearTargetedPins(const std::vector<Location> &pins) {
    for (auto pin : pins) {
        this->clearTargetedPin(pin);
    }
}

void BoardGrid::setViaForbidden(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaForbidden = true;
}

void BoardGrid::clearViaForbidden(const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaForbidden = false;
}

bool BoardGrid::isViaForbidden(const Location &l) const {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaForbidden;
}

void BoardGrid::setViaForbiddenArea(const std::vector<Location> &locations) {
    for (auto loc : locations) {
        this->setViaForbidden(loc);
    }
}
void BoardGrid::clearViaForbiddenArea(const std::vector<Location> &locations) {
    for (auto loc : locations) {
        this->clearViaForbidden(loc);
    }
}

std::unordered_map<Location, Location> BoardGrid::dijkstras_with_came_from(
    const std::vector<Location> &route, int via_size) {
    std::unordered_map<Location, Location> came_from;
    this->dijkstras_with_came_from(route, via_size, came_from);
    return came_from;
}

void BoardGrid::dijkstras_with_came_from(
    const std::vector<Location> &route, int via_size,
    std::unordered_map<Location, Location> &came_from) {
    std::cout << __FUNCTION__
              << "() nets: route.features.size() = " << route.size()
              << std::endl;

    // For path to multiple points
    // Searches from the multiple points to every other point
    this->working_cost_fill(std::numeric_limits<float>::infinity());

    LocationQueue<Location, float> frontier;  // search frontier
    // std::unordered_map<Location, Location> came_from; // cheapest neighbor
    for (Location start : route) {
        this->working_cost_set(0.0, start);
        frontier.push(start, 0.0);
        // came_from[start] = start;
        this->setCameFromId(start, this->locationToId(start));
    }

    std::cout << "came_from.size() = " << came_from.size()
              << ", frontier.size(): " << frontier.size() << std::endl;

    while (!frontier.empty()) {
        Location current = frontier.front();
        frontier.pop();

        // std::cout << "Visiting " << current << ", frontierSize: "<<
        // frontier.size() << std::endl;
        std::vector<std::pair<float, Location>> neighbors;
        this->getNeighbors(current, neighbors);

        for (std::pair<float, Location> next : neighbors) {
            if ((next.second.m_x < 0) || (next.second.m_x >= this->w) ||
                (next.second.m_y < 0) || (next.second.m_y >= this->h) ||
                (next.second.m_z < 0) || (next.second.m_z >= this->l)) {
                continue;  // continue if out of bounds
            }
            // std::cerr << "next.second.m_x: " << next.second.m_x << std::endl;
            // std::cerr << "next.second.m_y: " << next.second.m_y << std::endl;
            // std::cerr << "next.second.m_z: " << next.second.m_z << std::endl;

            // std::cerr << "geting new cost" << std::endl;

            // this->via_cost_at(next.second) ??????????
            float new_cost = this->working_cost_at(current) +
                             this->base_cost_at(next.second) +
                             this->via_cost_at(next.second) + next.first;

            // std::cerr << "Done" << std::endl;

            if (new_cost < this->working_cost_at(next.second)) {
                // std::cerr << "setting working cost" << std::endl;
                this->working_cost_set(new_cost, next.second);
                // came_from[next.second] = current;
                this->setCameFromId(next.second, this->locationToId(current));

                frontier.push(next.second, new_cost);
                // std::cerr << "Done" << std::endl;
            }

            // std::cerr << std::endl;
        }
    }
    // std::cerr << "finished dijkstras_with_came_from" << std::endl;
}

void BoardGrid::dijkstrasWithGridCameFrom(const std::vector<Location> &route,
                                          int via_size) {
    std::cout << __FUNCTION__
              << "() nets: route.features.size() = " << route.size()
              << std::endl;

    // For path to multiple points
    // Searches from the multiple points to every other point
    this->working_cost_fill(std::numeric_limits<float>::infinity());

    LocationQueue<Location, float> frontier;  // search frontier
    for (Location start : route) {
        this->working_cost_set(0.0, start);
        frontier.push(start, 0.0);
        // Set a ending for the backtracking
        this->setCameFromId(start, this->locationToId(start));
    }

    std::cout << " frontier.size(): " << frontier.size() << std::endl;

    while (!frontier.empty()) {
        Location current = frontier.front();
        frontier.pop();

        // std::cout << "Visiting " << current << ", frontierSize: "<<
        // frontier.size() << std::endl;
        std::vector<std::pair<float, Location>> neighbors;
        this->getNeighbors(current, neighbors);

        for (std::pair<float, Location> next : neighbors) {
            if ((next.second.m_x < 0) || (next.second.m_x >= this->w) ||
                (next.second.m_y < 0) || (next.second.m_y >= this->h) ||
                (next.second.m_z < 0) || (next.second.m_z >= this->l)) {
                continue;  // continue if out of bounds
            }
            // std::cerr << "next.second.m_x: " << next.second.m_x << std::endl;
            // std::cerr << "next.second.m_y: " << next.second.m_y << std::endl;
            // std::cerr << "next.second.m_z: " << next.second.m_z << std::endl;

            // this->via_cost_at(next.second) ??????????
            float new_cost = this->working_cost_at(current) +
                             this->base_cost_at(next.second) +
                             this->via_cost_at(next.second) + next.first;

            if (new_cost < this->working_cost_at(next.second)) {
                // std::cerr << "setting working cost" << std::endl;
                this->working_cost_set(new_cost, next.second);
                this->setCameFromId(next.second, this->locationToId(current));

                frontier.push(next.second, new_cost);
            }
        }
    }
}

void BoardGrid::aStarWithGridCameFrom(const std::vector<Location> &route, Location &finalEnd, float &finalCost) {
    std::cout << __FUNCTION__ << "() nets: route.features.size() = " << route.size() << std::endl;
    // std::cerr << fixed;
    // std::cout << fixed;

    // For path to multiple points
    // Searches from the multiple points to every other point
    this->working_cost_fill(std::numeric_limits<float>::infinity());

    float bestCostWhenReachTarget = std::numeric_limits<float>::max();
    LocationQueue<Location, float> frontier;  // search frontier
    for (Location start : route) {
        // Walked cost + estimated future cost
        float cost = 0.0 + getEstimatedCost(start);
        this->working_cost_set(0.0, start);
        frontier.push(start, cost);
        // std::cerr << "\tPQ: cost: " << cost << ", at" << start << std::endl;
        // Set a ending for the backtracking
        this->setCameFromId(start, this->locationToId(start));
    }

    std::cout << " frontier.size(): " << frontier.size() << std::endl;

    while (!frontier.empty()) {
        Location current = frontier.front();
        frontier.pop();

        // std::cout << "Visiting " << current << ", frontierSize: " << frontier.size() << std::endl;
        std::vector<std::pair<float, Location>> neighbors;
        this->getNeighbors(current, neighbors);

        for (std::pair<float, Location> next : neighbors) {
            float new_cost = this->working_cost_at(current);
            new_cost += next.first;
            // A*
            // float estCost = getEstimatedCost(next.second);
            // Test bending cost
            float estCost = getEstimatedCostWithBendingCost(current, next.second);

            if (new_cost < this->working_cost_at(next.second)) {
                this->working_cost_set(new_cost, next.second);
                this->setCameFromId(next.second, this->locationToId(current));

                frontier.push(next.second, new_cost + estCost);
                //frontier.push(next.second, new_cost);

                // std::cerr << "\tPQ: cost: " << new_cost << ", at" <<
                // next.second << std::endl;

                // TODO: Test early break
                // if (isTargetedPin(next.second) && new_cost < bestCostWhenReachTarget) {
                //     bestCostWhenReachTarget = new_cost;
                //     finalEnd = next.second;
                // }
            }

            // Test early break
            if (isTargetedPin(next.second)) {
                bestCostWhenReachTarget = new_cost;
                finalEnd = next.second;
                finalCost = bestCostWhenReachTarget;
                std::cout << "=> Find the target with cost at " << bestCostWhenReachTarget << std::endl;
                return;
            }
        }
    }
    //For Dijkstra to output
    finalCost = bestCostWhenReachTarget;
    std::cout << "=> Find the target with cost at " << bestCostWhenReachTarget << std::endl;
}

float BoardGrid::getEstimatedCost(const Location &l) {
    // return max(abs(l.m_x - this->current_targeted_pin.m_x), abs(l.m_y - this->current_targeted_pin.m_y));
    // return abs(l.m_x - this->current_targeted_pin.m_x) + abs(l.m_y - this->current_targeted_pin.m_y);
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
            bendingCost = 0.5;
        }
    }

    int absDiffX = abs(next.m_x - this->current_targeted_pin.m_x);
    int absDiffY = abs(next.m_y - this->current_targeted_pin.m_y);
    int minDiff = min(absDiffX, absDiffY);
    int maxDiff = max(absDiffX, absDiffY);
    return (float)minDiff * GlobalParam::gDiagonalCost + maxDiff - minDiff - bendingCost;
}

float BoardGrid::getEstimatedCostWithLayers(const Location &l) {
    float estCost = max(abs(l.m_x - this->current_targeted_pin.m_x), abs(l.m_y - this->current_targeted_pin.m_y));
    if (currentTargetedPinWithLayers.size() == 1) {
        estCost += GlobalParam::gLayerChangeCost * abs(current_targeted_pin.m_z - l.m_z);
    }
    return estCost;
}

void BoardGrid::getNeighbors(const Location &l, std::vector<std::pair<float, Location>> &ns) /*const*/ {
    // Expansion testing
    // int clr = this->current_clearance;
    // this->current_clearance = 0;

    // left
    if (l.m_x - 1 > -1) {
        Location left{l.m_x - 1, l.m_y, l.m_z};
        float leftCost = 1.0;
        leftCost += sized_trace_cost_at(left, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(leftCost, left));
    }

    // right
    if (l.m_x + 1 < this->w) {
        Location right{l.m_x + 1, l.m_y, l.m_z};
        float rightCost = 1.0;
        rightCost += sized_trace_cost_at(right, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(rightCost, right));
    }

    // forward
    if (l.m_y + 1 < this->h) {
        Location forward{l.m_x, l.m_y + 1, l.m_z};
        float forwardCost = 1.0;
        forwardCost += sized_trace_cost_at(forward, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(forwardCost, forward));
    }

    // back
    if (l.m_y - 1 > -1) {
        Location backward{l.m_x, l.m_y - 1, l.m_z};
        float backwardCost = 1.0;
        backwardCost += sized_trace_cost_at(backward, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(backwardCost, backward));
    }

    // up
    if (l.m_z + 1 < this->l) {
        Location up{l.m_x, l.m_y, l.m_z + 1};
        float upCost;
        if (sizedViaExpandableAndCost(l, current_half_via_diameter + current_clearance, upCost)) {
            upCost += GlobalParam::gLayerChangeCost;
            ns.push_back(std::pair<float, Location>(upCost, up));
        }
    }

    // down
    if (l.m_z - 1 > -1) {
        Location down{l.m_x, l.m_y, l.m_z - 1};
        float downCost;
        if (sizedViaExpandableAndCost(l, current_half_via_diameter + current_clearance, downCost)) {
            downCost += GlobalParam::gLayerChangeCost;
            ns.push_back(std::pair<float, Location>(downCost, down));
        }
    }

    // lf
    if (l.m_x - 1 > -1 && l.m_y + 1 < this->h) {
        Location lf{l.m_x - 1, l.m_y + 1, l.m_z};
        float lfCost = GlobalParam::gDiagonalCost;
        lfCost += sized_trace_cost_at(lf, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(lfCost, lf));
    }

    // lb
    if (l.m_x - 1 > -1 && l.m_y - 1 > -1) {
        Location lb{l.m_x - 1, l.m_y - 1, l.m_z};
        float lbCost = GlobalParam::gDiagonalCost;
        lbCost += sized_trace_cost_at(lb, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(lbCost, lb));
    }

    // rf
    if (l.m_x + 1 < this->w && l.m_y + 1 < this->h) {
        Location rf{l.m_x + 1, l.m_y + 1, l.m_z};
        float rfCost = GlobalParam::gDiagonalCost;
        rfCost += sized_trace_cost_at(rf, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(rfCost, rf));
    }

    // rb
    if (l.m_x + 1 < this->w && l.m_y - 1 > -1) {
        Location rb{l.m_x + 1, l.m_y - 1, l.m_z};
        float rbCost = GlobalParam::gDiagonalCost;
        rbCost += sized_trace_cost_at(rb, current_half_trace_width + current_clearance);
        ns.push_back(std::pair<float, Location>(rbCost, rb));
    }

    // Expansion testing
    // this->current_clearance = clr;
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

void BoardGrid::printMatPlot() {
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
        std::string outFileName = "layer" + std::to_string(l) + "_baseCost.py";
        outFileName =
            util::appendDirectory(GlobalParam::gOutputFolder, outFileName);
        std::ofstream ofs(outFileName, std::ofstream::out);
        std::cout << "outFileName: " << outFileName << std::endl;

        ofs << std::fixed << std::setprecision(5);
        ofs << "import numpy as np\n";
        ofs << "import matplotlib.pyplot as plt\n";
        ofs << "plt.close()\n";
        ofs << "viridis = plt.get_cmap('nipy_spectral', " << int(maxCost) << ")\n";
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
        ofs << "plt.pcolormesh(data, cmap=viridis, vmin=data.min(), "
               "vmax=data.max())\n";
        ofs << "plt.title('"
            << "layer" << std::to_string(l) << " basecost"
            << "')\n";
        ofs << "plt.colorbar()\n";
        ofs << "plt.axis('equal')\n";
        ofs << "plt.gca().invert_yaxis()\n";
        ofs << "plt.show()\n";
    }

    for (int l = 0; l < this->l; ++l) {
        std::string outFileName = "layer" + std::to_string(l) + "_viaCost.py";
        outFileName =
            util::appendDirectory(GlobalParam::gOutputFolder, outFileName);
        std::ofstream ofs(outFileName, std::ofstream::out);
        std::cout << "outFileName: " << outFileName << std::endl;

        ofs << std::fixed << std::setprecision(5);
        ofs << "import numpy as np\n";
        ofs << "import matplotlib.pyplot as plt\n";
        ofs << "plt.close()\n";
        ofs << "viridis = plt.get_cmap('nipy_spectral', " << int(maxCost) << ")\n";
        ofs << "data = np.array([[";

        for (int r = 0; r < this->h; ++r) {
            // for (int r = this->h - 1; r > -1; --r) {
            for (int c = 0; c < this->w; ++c) {
                ofs << this->via_cost_at(Location(c, r, l)) << " ";
                if (c < this->w - 1) {
                    ofs << ", ";
                } else {
                    ofs << "]";
                }
            }

            if (r < this->h - 1) {
                // if (r > 0) {
                ofs << ", [";
            }
        }

        ofs << "])\n";
        ofs << "plt.pcolormesh(data, cmap=viridis, vmin=data.min(), "
               "vmax=data.max())\n";
        ofs << "plt.title('"
            << "layer" << std::to_string(l) << " viacost"
            << "')\n";
        ofs << "plt.colorbar()\n";
        ofs << "plt.axis('equal')\n";
        ofs << "plt.gca().invert_yaxis()\n";
        ofs << "plt.show()\n";
    }
}

void BoardGrid::pprint() {
    char colors[11] = " .,:=+*#%@";
    // long
    // colors[36]    = {' ','.',':','░','#','▒','▓','█'};
    int num_colors = 10;

    float max_val = 0.0;
    for (int i = 0; i < this->size; i += 1) {
        if (this->grid[i].baseCost > max_val) max_val = this->grid[i].baseCost;
    }

    for (int l = 0; l < this->l; l += 1) {
        std::cout << std::endl
                  << "Layer: " << l << std::endl;

        for (int r = 0; r < this->h; r += 1) {
            for (int c = 0; c < this->w; c += 1) {
                int current_color = this->base_cost_at(Location(c, r, l)) /
                                    max_val * num_colors;
                std::cout << colors[current_color] << " ";
                // std::cout << current_color << " ";

                // std::cout << std::setw(6) << std::setprecision(2) <<
                // std::right << this->at(c, r, l);
            }
            std::cout << std::endl;
        }
    }
}

float BoardGrid::sized_via_cost_at(const Location &l, const int viaRadius) const {
    // TODO: THROUGH HOLE only, should be extended to uVia and blind/burried
    // vias
    int radius = viaRadius;
    float cost = 0.0;
    for (int z = 0; z < this->l; z += 1) {
        for (int y = -radius; y <= radius; y += 1) {
            for (int x = -radius; x <= radius; x += 1) {
                Location current_l = Location(l.m_x + x, l.m_y + y, z);
                if (!validate_location(current_l)) {
                    // std::cerr << 'Invalid location: ' << current_l <<
                    // std::endl;
                    // TODO: cost to model the clearance to boundary
                    cost += 1000;
                    continue;
                }
                cost += this->via_cost_at(current_l);
                cost += this->base_cost_at(current_l);
            }
        }
    }
    return cost;
}

bool BoardGrid::sizedViaExpandableAndCost(const Location &l, const int viaRadius, float &cost) const {
    int radius = viaRadius;
    cost = 0.0;
    for (int z = 0; z < this->l; ++z) {
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                Location current_l = Location(l.m_x + x, l.m_y + y, z);
                if (!validate_location(current_l)) {
                    // TODO: cost to model the clearance to boundary
                    cost += 1000;
                    continue;
                }
                if (this->isViaForbidden(current_l)) {
                    return false;
                }
                cost += this->via_cost_at(current_l);
                cost += this->base_cost_at(current_l);
            }
        }
    }
    return true;
}

float BoardGrid::sized_trace_cost_at(const Location &l, int traceRadius) const {
    int radius = traceRadius;
    float cost = 0.0;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            Location current_l = Location(l.m_x + x, l.m_y + y, l.m_z);
            if (!validate_location(current_l)) {
                // TODO: cost to model the clearance to boundary
                cost += 100000;
                continue;
            }
            cost += this->base_cost_at(current_l);
            cost += this->via_cost_at(current_l);
        }
    }
    return cost;
}

void BoardGrid::print_came_from(
    const std::unordered_map<Location, Location> &came_from,
    const Location &end) {
    // Location end(ex, ey, ez);

    std::cout << "Came froms: " << std::endl;
    std::cout << "\tsize: " << came_from.size() << std::endl;

    for (int l = 0; l < this->l; l += 1) {
        std::cout << std::endl
                  << "Layer: " << l << std::endl;

        for (int r = 0; r < this->h; r += 1) {
            for (int c = 0; c < this->w; c += 1) {
                Location current(c, r, l);

                if (current == end) {
                    std::cout << "# ";
                    continue;  // goal
                }

                if (came_from.find(current) == came_from.end()) {
                    std::cout << ". ";  // not found
                    continue;           // unexplored
                }

                Location cf = came_from.find(current)->second;

                if (current.m_x > cf.m_x) {
                    std::cout << "> ";
                } else if (current.m_x < cf.m_x) {
                    std::cout << "< ";
                } else if (current.m_y > cf.m_y) {
                    std::cout << "V ";
                } else if (current.m_y < cf.m_y) {
                    std::cout << "^ ";
                } else if (current.m_z > cf.m_z) {
                    std::cout << "X ";
                } else if (current.m_z < cf.m_z) {
                    std::cout << "O ";
                } else if ((current.m_x == cf.m_x) && (current.m_y == cf.m_y) &&
                           (current.m_z == cf.m_z)) {
                    std::cout << "* ";  // start
                }
            }
            std::cout << std::endl;
        }
    }
}

void BoardGrid::add_via_cost(const Location &l, const int layer, const float cost, const int viaRadius) {
    int radius = viaRadius;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
#ifdef BOUND_CHECKS
            assert(((l.m_x + x) + (l.m_y + y) * this->w + (layer) * this->w * this->h) < this->size);
#endif
            this->grid[(l.m_x + x) + (l.m_y + y) * this->w + (layer) * this->w * this->h].viaCost += cost;
        }
    }
}

void BoardGrid::add_via_cost(const Location &l, const int layer, const float cost, const std::vector<Point_2D<int>> viaShapeToGrids) {
    for (auto &relativePt : viaShapeToGrids) {
#ifdef BOUND_CHECKS
        assert(((l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (layer) * this->w * this->h) < this->size);
#endif
        this->grid[(l.m_x + relativePt.x()) + (l.m_y + relativePt.y()) * this->w + (layer) * this->w * this->h].viaCost += cost;
    }
}

void BoardGrid::remove_route_from_base_cost(const MultipinRoute &route) {
    add_route_to_base_cost(route, current_half_trace_width, -GlobalParam::gTraceBasicCost, current_half_via_diameter, -GlobalParam::gViaInsertionCost);
    // int traceHalfWidth = this->current_half_trace_width + this->current_clearance;
    // int viaRadius = this->current_half_via_diameter + this->current_clearance;
    // add_route_to_base_cost(route, traceHalfWidth, -GlobalParam::gTraceBasicCost, viaRadius, -GlobalParam::gViaInsertionCost);
}

void BoardGrid::add_route_to_base_cost(const MultipinRoute &route) {
    add_route_to_base_cost(route, current_half_trace_width, GlobalParam::gTraceBasicCost, current_half_via_diameter, GlobalParam::gViaInsertionCost);
    // int traceHalfWidth = this->current_half_trace_width + this->current_clearance;
    // int viaRadius = this->current_half_via_diameter + this->current_clearance;
    // add_route_to_base_cost(route, traceHalfWidth, GlobalParam::gTraceBasicCost, viaRadius, GlobalParam::gViaInsertionCost);
}

void BoardGrid::add_route_to_base_cost(const MultipinRoute &route, const int traceRadius, const float traceCost, const int viaRadius, const float viaCost) {
    if (route.features.empty())
        return;

    //cout << __FUNCTION__ << "(): traceCost: " << traceCost << ", viaCost: " << viaCost << std::endl;

    // Add costs for traces
    for (auto &l : route.features) {
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
        auto &prevLoc = route.features.at(i - 1);
        auto &curLoc = route.features.at(i);

        if ((prevLoc.m_z != curLoc.m_z) && (prevLoc.m_x == curLoc.m_x) && (prevLoc.m_y == curLoc.m_y)) {
            for (int z = 0; z < this->l; ++z) {
                this->add_via_cost(prevLoc, z, viaCost, viaRadius);
            }
        }
    }
}

void BoardGrid::came_from_to_features(
    const std::unordered_map<Location, Location> &came_from,
    const Location &end, std::vector<Location> &features) const {
    // std::vector<Location> features;
    // features.insert(end);
    std::cout << "Starting came_from_to_features " << std::endl;

    if (!this->validate_location(end))
        std::cout << "Bad end for came_from_to_features" << std::endl;

    features.push_back(end);
    Location current = end;

    while (came_from.find(current) != came_from.end()) {  // while not the start
        Location next = came_from.find(current)->second;
        // if (features.find(next) != features.end()) { // next already in
        // features, loops or start break;
        // }
        if (next == current) {
            break;
        }
        // features.insert(next);
        features.push_back(next);
        current = next;
    }

    // ?????????
    // for (Location l : features)
    // {
    // 	//std::cout << "\t" << l << std::endl;
    // 	if (l.m_z > 2)
    // 	{
    // 		exit(-1);
    // 	}
    // }

    std::cout << "Finished came_from_to_features " << std::endl;

    // return features;
}

void BoardGrid::came_from_to_features(const Location &end,
                                      std::vector<Location> &features) const {
    std::cout << "Starting came_from_to_features ID" << std::endl;

    if (!this->validate_location(end))
        std::cout << "Bad end for came_from_to_features ID" << std::endl;

    features.push_back(end);
    Location current = end;
    int currentId = this->locationToId(current);
    int nextId = this->getCameFromId(currentId);

    while (nextId != -1) {
        if (nextId == currentId) {
            break;
        }

        Location next;
        this->idToLocation(nextId, next);

        features.push_back(next);
        currentId = nextId;
        nextId = this->getCameFromId(currentId);
    }

    std::cout << "Finished came_from_to_features ID" << std::endl;
}

std::vector<Location> BoardGrid::came_from_to_features(
    const std::unordered_map<Location, Location> &came_from,
    const Location &end) const {
    std::vector<Location> features;
    // features.insert(end);
    this->came_from_to_features(came_from, end, features);
    return features;
}

void BoardGrid::print_route(
    const std::unordered_map<Location, Location> &came_from,
    const Location &end) {
    // Location end(ex, ey, ez);

    // std::cout << "end: " << end << std::endl;

    // std::cout << "came_from: " << std::endl;
    // for (std::pair<Location, Location> l : came_from) {
    // 	std::cout << l.first << ", " << l.second << std::endl;
    // }

    std::vector<Location> features;
    features = this->came_from_to_features(came_from, end);
    // features.insert(end);
    // Location current = end;

    // std::cout << "current: " << current << std::endl;

    // std::cout << "Finding features" << std::endl;
    // while (came_from.find(current) != came_from.end()) { // while not the
    // start

    // 	Location next = came_from.find(current)->second;
    // 	// std::cout << "next: " << next << std::endl;

    // 	if (features.find(next) != features.end()) { // next already in
    // features, loops or start 		break;
    // 	}
    // 	features.insert(next);
    // 	// std::cout << "Got next features: " << next <<
    // 	 std::endl;
    // 	current = next;
    // }

    std::cout << "Printing features" << std::endl;
    for (int l = 0; l < this->l; l += 1) {
        std::cout << std::endl
                  << "Layer: " << l << std::endl;

        for (int r = 0; r < this->h; r += 1) {
            for (int c = 0; c < this->w; c += 1) {
                Location current(c, r, l);

                if (std::find(features.begin(), features.end(), current) !=
                    features.end()) {
                    // if (features.find(current) != features.end()) {
                    std::cout << "# ";  // contains feature
                } else {
                    std::cout << ". ";
                }
            }
            std::cout << std::endl;
        }
    }
}

void BoardGrid::print_features(std::vector<Location> features) {
    std::cout << "Printing features" << std::endl;
    for (int l = 0; l < this->l; l += 1) {
        std::cout << std::endl
                  << "Layer: " << l << std::endl;

        for (int r = 0; r < this->h; r += 1) {
            for (int c = 0; c < this->w; c += 1) {
                Location current(c, r, l);

                if (std::find(features.begin(), features.end(), current) !=
                    features.end()) {
                    // if (features.find(current) != features.end()) {
                    std::cout << "# ";  // contains feature
                } else {
                    std::cout << ". ";
                }
            }
            std::cout << std::endl;
        }
    }
}

void BoardGrid::add_route(MultipinRoute &route) {
    //int cost = GlobalParam::gTraceBasicCost;
    int via_size = 7;
    int num_pins = route.pins.size();

    if (num_pins <= 0) {
        return;
    } else if (num_pins == 1) {
        return;
    } else {
        route.features.push_back(route.pins[0]);

        for (Location pin : route.pins) {
            // std::unordered_map<Location, Location> came_from;
            this->dijkstrasWithGridCameFrom(route.features, via_size);

            std::vector<Location> new_features;
            // this->came_from_to_features(came_from, pin, new_features);
            this->came_from_to_features(pin, new_features);

            for (Location f : new_features) {
                route.features.push_back(f);
            }
        }
        // this->print_features(route.features);
        // TODO
        // this->add_route_to_base_cost(route, traceWidth, cost, via_size);
        this->add_route_to_base_cost(route);
    }
}

void BoardGrid::addRoute(MultipinRoute &route) {
    int num_pins = route.pins.size();

    // TODO
    // clear came from in the GridCell
    // check if traceWidth/clearance/viaDiameter persist

    if (num_pins <= 1) return;

    route.features.push_back(route.pins[0]);

    for (size_t i = 1; i < route.pins.size(); ++i)
    // for (size_t i = 0; i < route.pins.size(); ++i) //Original incorrect
    // implementation
    {
        // For early break
        this->setTargetedPin(route.pins[i]);
        // For cost estimation (cares about x and y only)
        current_targeted_pin = route.pins[i];

        // this->dijkstrasWithGridCameFrom(route.features, via_size);
        // via size is half_width
        Location finalEnd{0, 0, 0};
        this->aStarWithGridCameFrom(route.features, finalEnd, route.currentRouteCost);

        std::vector<Location> new_features;
        this->came_from_to_features(route.pins[i], new_features);

        for (Location f : new_features) {
            route.features.push_back(f);
        }

        // For early break
        this->clearTargetedPin(route.pins[i]);
        // For cost estimation
        current_targeted_pin = Location{0, 0, 0};
    }
    // this->print_features(route.features);
    // TODO
    this->add_route_to_base_cost(route);
}

void BoardGrid::addRouteWithGridPins(MultipinRoute &route) {
    // TODO
    // check if traceWidth/clearance/viaDiameter persist

    std::cout << __FUNCTION__ << "() route.gridPins.size: " << route.mGridPins.size() << std::endl;

    if (route.mGridPins.size() <= 1) return;

    // Clear and initialize
    this->clearAllCameFromId();
    route.currentRouteCost = 0.0;

    for (size_t i = 1; i < route.mGridPins.size(); ++i) {
        // For early break
        this->setTargetedPins(route.mGridPins.at(i).pinWithLayers);
        // For cost estimation (cares about x and y only)
        current_targeted_pin = route.mGridPins.at(i).pinWithLayers.front();
        //currentTargetedPinWithLayers = route.gridPins.at(i).pinWithLayers;

        // via size is half_width
        Location finalEnd{0, 0, 0};
        float routeCost = 0.0;
        // this->dijkstrasWithGridCameFrom(route.features, current_half_via_diameter);
        if (route.features.empty()) {
            this->aStarWithGridCameFrom(route.mGridPins.front().pinWithLayers, finalEnd, routeCost);
        } else {
            this->aStarWithGridCameFrom(route.features, finalEnd, routeCost);
        }
        route.currentRouteCost += routeCost;

        std::vector<Location> new_features;
        // TODO Fix this, when THROUGH PAD as a start?
        this->came_from_to_features(finalEnd, new_features);

        for (Location f : new_features) {
            route.features.push_back(f);
        }

        // For early break
        this->clearTargetedPins(route.mGridPins.at(i).pinWithLayers);
        // For cost estimation
        current_targeted_pin = Location{0, 0, 0};
        //currentTargetedPinWithLayers.clear();
    }
    // this->print_features(route.features);
    // TODO
    this->add_route_to_base_cost(route);
}

void BoardGrid::ripup_route(MultipinRoute &route) {
    std::cout << "Doing ripup" << std::endl;
    // Basic checking on the routed features
    for (Location l : route.features) {
        if (l.m_x > this->w || l.m_y > this->h || l.m_z > this->l) {
            std::cout << "Bad route to ripup: " << l << std::endl;
            return;
        }
    }
    this->remove_route_from_base_cost(route);
    route.features.clear();
    std::cout << "Finished ripup" << std::endl;
}

void BoardGrid::set_current_rules(const int gridNetclassId) {
    auto &gridNetclass = this->getGridNetclass(gridNetclassId);
    this->set_current_rules(gridNetclass.getClearance(), gridNetclass.getTraceWidth(), gridNetclass.getViaDia());
}

void BoardGrid::set_current_rules(const int clr, const int trWid, const int viaDia) {
    this->current_trace_width = trWid;
    this->current_half_trace_width = (int)floor((double)trWid / 2.0);
    this->current_clearance = clr;
    this->current_via_diameter = viaDia;
    this->current_half_via_diameter = (int)floor((double)viaDia / 2.0);

    cout << __FUNCTION__ << "() curTraceWid: " << this->current_trace_width
         << ", curHalfTraceWid: " << this->current_half_trace_width
         << ", curClearance: " << this->current_clearance
         << ", curViaDiameter: " << this->current_via_diameter
         << ", curHalfViaDiameter: " << this->current_half_via_diameter << std::endl;
}

void BoardGrid::addGridNetclass(const GridNetclass &gridNetclass) {
    this->mGridNetclasses.push_back(gridNetclass);
}

GridNetclass &BoardGrid::getGridNetclass(const int gridNetclassId) {
    if (gridNetclassId < 0 || gridNetclassId > this->mGridNetclasses.size()) {
        std::cerr << "Illegal grid netclass id: " << gridNetclassId << std::endl;
        return this->mGridNetclasses.front();
    } else {
        return this->mGridNetclasses.at(gridNetclassId);
    }
}

bool BoardGrid::validate_location(const Location &l) const {
    if (l.m_x >= this->w || l.m_x < 0) return false;
    if (l.m_y >= this->h || l.m_y < 0) return false;
    if (l.m_z >= this->l || l.m_z < 0) return false;
    return true;
}
