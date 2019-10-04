// BoardGrid.cpp
#include "BoardGrid.h"

void BoardGrid::initilization(int w, int h, int l) {
    this->w = w;
    this->h = h;
    this->l = l;
    this->size = w * h * l;

    assert(this->base_cost == nullptr);
    assert(this->working_cost == nullptr);
    assert(this->via_cost == nullptr);

    this->base_cost = new float[this->size];
    this->working_cost = new float[this->size];
    this->via_cost = new float[this->size];
    this->grid = new GridCell[this->size];

    assert(this->base_cost != nullptr);
    assert(this->working_cost != nullptr);
    assert(this->via_cost != nullptr);
}

void BoardGrid::base_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        // this->base_cost[i] = value;
        this->grid[i].baseCost = value;
    }
}

void BoardGrid::working_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        // this->working_cost[i] = value;
        this->grid[i].workingCost = value;
    }
}

void BoardGrid::via_cost_fill(float value) {
    for (int i = 0; i < this->size; ++i) {
        // this->working_cost[i] = value;
        this->grid[i].viaCost = value;
    }
}

float BoardGrid::cost_to_occupy(const Location &l) const {
    return this->base_cost_at(l) + this->via_cost_at(l);
}

float BoardGrid::base_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    // return this->base_cost[l.m_x + l.m_y * this->w + l.m_z * this->w *
    // this->h];
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h]
        .baseCost;
}

float BoardGrid::via_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    // return this->via_cost[l.m_x + l.m_y * this->w + l.m_z * this->w *
    // this->h];
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h]
        .viaCost;
}

float BoardGrid::working_cost_at(const Location &l) const {
#ifdef BOUND_CHECKS
    assert((l.m_x + l.m_y * this->w + l.m_z * this->w * this->h) < this->size);
#endif
    // return this->working_cost[l.m_x + l.m_y * this->w + l.m_z * this->w *
    // this->h];
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h]
        .workingCost;
}

void BoardGrid::base_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // this->base_cost[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h] =
    // value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost =
        value;
}

void BoardGrid::base_cost_add(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // this->base_cost[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h] +=
    // value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].baseCost +=
        value;
}

void BoardGrid::working_cost_set(float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // this->working_cost[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h] =
    // value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h]
        .workingCost = value;
}

void BoardGrid::setCameFromId(const Location &l, const int id) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].cameFromId =
        id;
}

int BoardGrid::getCameFromId(const Location &l) const {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    return this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h]
        .cameFromId;
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
    // this->via_cost[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h] =
    // value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost =
        value;
}

void BoardGrid::via_cost_add(const float value, const Location &l) {
#ifdef BOUND_CHECKS
    assert(l.m_x + l.m_y * this->w + l.m_z * this->w * this->h < this->size);
#endif
    // this->via_cost[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h] +=
    // value;
    this->grid[l.m_x + l.m_y * this->w + l.m_z * this->w * this->h].viaCost +=
        value;
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

// void BoardGrid::breadth_first_search(const Location &start, const Location
// &end)
// {
// 	// Location start(sx, sy, sz);
// 	// Location end(ex, ey, ez);

// 	std::queue<Location> frontier;
// 	frontier.push(start);

// 	std::unordered_set<Location> visited;
// 	visited.insert(start);

// 	while (!frontier.empty())
// 	{
// 		Location current = frontier.front();
// 		frontier.pop();

// 		std::cout << "Visiting " << current << std::endl;

// 		for (std::pair<float, Location> next : this->neighbors(current))
// 		{
// 			if (
// 				(next.second.m_x < 0) || (next.second.m_x >=
// this->w)
// || 				(next.second.m_y < 0) || (next.second.m_y >= this->h)
// || 				(next.second.m_z < 0) || (next.second.m_z >=
// this->l))
// 			{
// 				continue; // continue if out of bounds
// 			}
// 			if (visited.find(next.second) == visited.end())
// 			{ // next not in visited
// 				frontier.push(next.second);
// 				visited.insert(next.second);
// 			}
// 		}
// 	}
// }

// std::unordered_map<Location, Location>
// BoardGrid::breadth_first_search_with_came_from(const Location &start, const
// Location &end)
// {
// 	// Location start(sx, sy, sz);
// 	// Location end(ex, ey, ez);

// 	std::cout << "Starting breadth_first_search_with_came_from start: " <<
// start << " end: " << end << std::endl;

// 	std::queue<Location> frontier;
// 	frontier.push(start);

// 	// std::unordered_set<Location> visited; // we can keep track of visited
// set with the came from map
// 	// visited.insert(start);

// 	std::unordered_map<Location, Location> came_from;
// 	came_from[start] = start;

// 	while (!frontier.empty())
// 	{
// 		Location current = frontier.front();
// 		frontier.pop();

// 		std::cout << "Visiting " << current << std::endl;

// 		if (current == end)
// 		{
// 			std::cout << "Found goal, exiting early" << std::endl;
// 			break;
// 		}

// 		for (std::pair<float, Location> next : this->neighbors(current))
// 		{
// 			if (
// 				(next.second.m_x < 0) || (next.second.m_x >=
// this->w)
// || 				(next.second.m_y < 0) || (next.second.m_y >= this->h)
// || 				(next.second.m_z < 0) || (next.second.m_z >=
// this->l))
// 			{
// 				continue; // continue if out of bounds
// 			}
// 			// if (visited.find(next) == visited.end()) { // next not
// in visited
// 			// if (came_from.find(next.second) == came_from.end()) {
// //
// next not in visited 			if (came_from.find(next.second) ==
// came_from.end()) 			{ // next not in visited
// frontier.push(next.second);
// 				// visited.insert(next);
// 				came_from[next.second] = current;
// 			}
// 		}
// 	}

// 	return came_from;
// }

// std::unordered_map<Location, Location>
// BoardGrid::dijkstras_with_came_from(const Location &start)
// {
// 	// Location start(sx, sy, sz);
// 	// Location end(ex, ey, ez);

// 	//std::cout << "Starting dijkstras_with_came_from start: " << start << "
// end: " << end << std::endl; 	std::cout << "Starting dijkstras_with_came_from
// ==TwoPin== start: " << start << std::endl;

// 	// this->working_cost_fill(0.1);
// 	this->working_cost_fill(std::numeric_limits<float>::infinity());
// 	this->working_cost_set(0.0, start);

// 	LocationQueue<Location, float> frontier;
// 	frontier.push(start, 0.0);

// 	std::unordered_map<Location, Location> came_from;
// 	came_from[start] = start;

// 	while (!frontier.empty())
// 	{
// 		Location current = frontier.front();
// 		frontier.pop();

// 		//std::cout << "Visiting " << current << ", frontierSize: "<<
// frontier.size() << std::endl;

// 		// if (current == end) {
// 		// std::cout << "Found goal, exiting early" << std::endl;
// 		// break;
// 		// }

// 		//std::cout << "Got " << this->neighbors(current).size() << "
// neighbors" << std::endl; 		std::array<std::pair<float, Location>,
// 10>
// &&aNeighbors = this->neighbors(current);

// 		//for (std::pair<float, Location> next :
// this->neighbors(current)) { 		for (std::pair<float, Location> next :
// aNeighbors)
// 		{
// 			if (
// 				(next.second.m_x < 0) || (next.second.m_x >=
// this->w)
// || 				(next.second.m_y < 0) || (next.second.m_y >= this->h)
// || 				(next.second.m_z < 0) || (next.second.m_z >=
// this->l))
// 			{
// 				// std::cout << "\tOut of bounds neighbor " <
// next.second
// << std::endl; 				continue; // continue if out of
// bounds
// 			}

// 			//std::cout << "\tWorking on neighbor " << next.second
// << std::endl;

// 			// this->at(Location l) give the cost of moving to l
// 			// this->working_cost_at(Location l) gives the current
// best cost to get to l 			float new_cost =
// this->working_cost_at(current) + this->base_cost_at(next.second) +
// next.first;

// 			// std::cout << "\t\tnew_cost: " << new_cost <<
// std::endl;
// 			// std::cout << "\t\t\twc: " <<
// this->working_cost_at(current)
// <<  " bc: " << this->base_cost_at(next.second) << " n.f: " << next.first <<
// std::endl;

// 			// std::cout << "\t\told cost: " <<
// this->working_cost_at(next.second) << std::endl;

// 			if (new_cost < this->working_cost_at(next.second))
// 			{
// 				this->working_cost_set(new_cost, next.second);
// 				came_from[next.second] = current;
// 				frontier.push(next.second, new_cost);
// 			}
// 		}
// 	}

// 	return came_from;
// }

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

void BoardGrid::aStarWithGridCameFrom(const std::vector<Location> &route,
                                      Location &finalEnd) {
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
        this->working_cost_set(cost, start);
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
            if ((next.second.m_x < 0) || (next.second.m_x >= this->w) ||
                (next.second.m_y < 0) || (next.second.m_y >= this->h) ||
                (next.second.m_z < 0) || (next.second.m_z >= this->l)) {
                continue;  // continue if out of bounds
            }

            // TODO: this->via_cost_at(next.second)?
            float new_cost = this->working_cost_at(current) + this->base_cost_at(next.second) /*+ this->via_cost_at(next.second)*/ + next.first;
            // A*
            new_cost += getEstimatedCost(next.second);

            if (new_cost < this->working_cost_at(next.second)) {
                // std::cerr << "setting working cost" << std::endl;
                this->working_cost_set(new_cost, next.second);
                this->setCameFromId(next.second, this->locationToId(current));

                frontier.push(next.second, new_cost);
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
                std::cout << "=> Find the target with cost at " << bestCostWhenReachTarget << std::endl;
                return;
            }
        }
    }
    //For Dijkstra to output
    std::cout << "=> Find the target with cost at " << bestCostWhenReachTarget << std::endl;
}

float BoardGrid::getEstimatedCost(const Location &l) {
    return max(abs(l.m_x - this->current_targeted_pin.m_x), abs(l.m_y - this->current_targeted_pin.m_y));
    // int absDiffX = abs(l.m_x - this->current_targeted_pin.m_x);
    // int absDiffY = abs(l.m_y - this->current_targeted_pin.m_y);
    // int minDiff = min(absDiffX, absDiffY);
    // int maxDiff = max(absDiffX, absDiffY);
    // return minDiff * GlobalParam::gDiagonalCost + maxDiff - minDiff;
}

void BoardGrid::getNeighbors(const Location &l, std::vector<std::pair<float, Location>> &ns) const {
    // left
    Location left{l.m_x - 1, l.m_y, l.m_z};
    float leftCost = 1.0;
    leftCost += sized_trace_cost_at(left, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(leftCost, left));

    // right
    Location right{l.m_x + 1, l.m_y, l.m_z};
    float rightCost = 1.0;
    rightCost += sized_trace_cost_at(right, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(rightCost, right));

    // forward
    Location forward{l.m_x, l.m_y + 1, l.m_z};
    float forwardCost = 1.0;
    forwardCost += sized_trace_cost_at(forward, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(forwardCost, forward));

    // back
    Location backward{l.m_x, l.m_y - 1, l.m_z};
    float backwardCost = 1.0;
    backwardCost += sized_trace_cost_at(backward, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(backwardCost, backward));

    // up
    // Location up{l.m_x, l.m_y, l.m_z + 1};
    // float upCost = GlobalParam::gLayerChangeCost;
    // upCost += this->sized_via_cost_at(l, current_half_via_diameter + current_clearance);
    // ns.push_back(std::pair<float, Location>(upCost, up));

    Location up{l.m_x, l.m_y, l.m_z + 1};
    float upCost = GlobalParam::gLayerChangeCost;
    if (sizedViaExpandableAndCost(l, current_half_via_diameter + current_clearance, upCost)) {
        ns.push_back(std::pair<float, Location>(upCost, up));
    }

    // down
    // Location down{l.m_x, l.m_y, l.m_z - 1};
    // float downCost = GlobalParam::gLayerChangeCost;
    // downCost += this->sized_via_cost_at(l, current_half_via_diameter + current_clearance);
    // ns.push_back(std::pair<float, Location>(downCost, down));

    Location down{l.m_x, l.m_y, l.m_z - 1};
    float downCost = GlobalParam::gLayerChangeCost;
    if (sizedViaExpandableAndCost(l, current_half_via_diameter + current_clearance, downCost)) {
        ns.push_back(std::pair<float, Location>(downCost, down));
    }

    // lf
    Location lf{l.m_x - 1, l.m_y + 1, l.m_z};
    float lfCost = GlobalParam::gDiagonalCost;
    lfCost += sized_trace_cost_at(lf, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(lfCost, lf));

    // lb
    Location lb{l.m_x - 1, l.m_y - 1, l.m_z};
    float lbCost = GlobalParam::gDiagonalCost;
    lbCost += sized_trace_cost_at(lb, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(lbCost, lb));

    // rf
    Location rf{l.m_x + 1, l.m_y + 1, l.m_z};
    float rfCost = GlobalParam::gDiagonalCost;
    rfCost += sized_trace_cost_at(rf, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(rfCost, rf));

    // rb
    Location rb{l.m_x + 1, l.m_y - 1, l.m_z};
    float rbCost = GlobalParam::gDiagonalCost;
    rbCost += sized_trace_cost_at(rb, current_half_trace_width + current_clearance);
    ns.push_back(std::pair<float, Location>(rbCost, rb));
}

void BoardGrid::printGnuPlot() {
    float max_val = 0.0;
    for (int i = 0; i < this->size; i += 1) {
        if (this->base_cost[i] > max_val) max_val = this->base_cost[i];
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
        if (this->base_cost[i] > maxCost) {
            maxCost = this->base_cost[i];
        } else if (this->base_cost[i] < minCost) {
            minCost = this->base_cost[i];
        }
    }

    std::cout << "printGnuPlot()::Max Cost: " << maxCost
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
        ofs << "viridis = plt.get_cmap('viridis', 12)\n";
        ofs << "data = np.array([[";

        // for (int r = 0; r < this->h; ++r)
        for (int r = this->h - 1; r > -1; --r) {
            for (int c = 0; c < this->w; ++c) {
                ofs << this->base_cost_at(Location(c, r, l)) << " ";
                if (c < this->w - 1) {
                    ofs << ", ";
                } else {
                    ofs << "]";
                }
            }

            // if (r < this->h - 1)
            if (r > 0) {
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
        ofs << "viridis = plt.get_cmap('viridis', 12)\n";
        ofs << "data = np.array([[";

        // for (int r = 0; r < this->h; ++r)
        for (int r = this->h - 1; r > -1; --r) {
            for (int c = 0; c < this->w; ++c) {
                ofs << this->via_cost_at(Location(c, r, l)) << " ";
                if (c < this->w - 1) {
                    ofs << ", ";
                } else {
                    ofs << "]";
                }
            }

            // if (r < this->h - 1)
            if (r > 0) {
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
        if (this->base_cost[i] > max_val) max_val = this->base_cost[i];
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
        for (int y = -radius; y < radius; y += 1) {
            for (int x = -radius; x < radius; x += 1) {
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
    //cost = 0.0;
    for (int z = 0; z < this->l; ++z) {
        for (int y = -radius; y < radius; ++y) {
            for (int x = -radius; x < radius; ++x) {
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
    for (int y = -radius; y < radius; y += 1) {
        for (int x = -radius; x < radius; x += 1) {
            Location current_l = Location(l.m_x + x, l.m_y + y, l.m_z);
            if (!validate_location(current_l)) {
                // TODO: cost to model the clearance to boundary
                cost += 100000;
                continue;
            }
            cost += this->base_cost_at(current_l);
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

void BoardGrid::add_via_cost(const Location &l, const int layer,
                             const int viaRadius) {
    int radius = viaRadius;
    float cost = GlobalParam::gViaInsertionCost;
    for (int y = -radius; y < radius; y += 1) {
        for (int x = -radius; x <= radius; x += 1) {
#ifdef BOUND_CHECKS
            assert(((l.m_x + x) + (l.m_y + y) * this->w +
                    (layer) * this->w * this->h) < this->size);
#endif
            this->grid[(l.m_x + x) + (l.m_y + y) * this->w +
                       (layer) * this->w * this->h]
                .viaCost += cost;
        }
    }
}

void BoardGrid::remove_via_cost(const Location &l, const int layer,
                                const int viaRadius) {
    int radius = viaRadius;
    float cost = GlobalParam::gViaInsertionCost;
    for (int y = -radius; y < radius; y += 1) {
        for (int x = -radius; x <= radius; x += 1) {
#ifdef BOUND_CHECKS
            assert(((l.m_x + x) + (l.m_y + y) * this->w +
                    (layer) * this->w * this->h) < this->size);
#endif
            this->grid[(l.m_x + x) + (l.m_y + y) * this->w +
                       (layer) * this->w * this->h]
                .viaCost -= cost;
        }
    }
}

void BoardGrid::add_route_to_base_cost(const MultipinRoute &route, int radius,
                                       float cost, int via_size) {
    // std::vector<Location> features = route.features;
    Location last_location = route.features[0];
    for (Location l : route.features) {
        // std::cout << "setting cost for feature " << l << std::endl;
        int layer = l.m_z;

        // Add costs for vias
        // TODO:: Handle THROUGH VIA only
        if ((l.m_z != last_location.m_z) && (l.m_x == last_location.m_x) &&
            (l.m_y == last_location.m_y)) {
            for (int z = 0; z < this->l; z += 1) {
                this->add_via_cost(l, z, current_half_via_diameter);
            }
        }

        for (int current_radius = 0; current_radius <= radius;
             current_radius += 1) {
            float current_cost = cost;
            if (current_cost <= 0) break;

            for (int r = l.m_y - current_radius; r <= l.m_y + current_radius;
                 r += 1) {
                if (r < 0) continue;
                if (r >= this->h) continue;
                for (int c = l.m_x - current_radius;
                     c <= l.m_x + current_radius; c += 1) {
                    if (c < 0) continue;
                    if (c >= this->w) continue;
                    // std::cout << "\tsetting cost at " << Location(c, r,
                    // layer) << std::endl;
                    this->base_cost_set(
                        this->base_cost_at(Location(c, r, layer)) +
                            current_cost,
                        Location(c, r, layer));
                }
            }
        }
    }
}

// void BoardGrid::remove_route_from_base_cost(const Route &route, int radius,
// float cost)
// {
// 	// std::vector<Location> features = route.features;
// 	std::cout << "Starting remove_route_from_base_cost" << std::endl;
// 	for (Location l : route.features)
// 	{
// 		// std::cout << "setting cost for feature " << l << std::endl;
// 		int layer = l.m_z;

// 		if (layer > 2)
// 		{
// 			std::cout << "Bad layer: " << l << std::endl;
// 			exit(-1);
// 		}

// 		for (int current_radius = 0; current_radius <= radius;
// current_radius += 1)
// 		{
// 			float current_cost = cost - current_radius;
// 			if (current_cost <= 0)
// 				break;

// 			for (int r = l.m_y - current_radius; r <= l.m_y +
// current_radius; r += 1)
// 			{
// 				if (r < 0)
// 					continue;
// 				if (r >= this->h)
// 					continue;
// 				for (int c = l.m_x - current_radius; c <= l.m_x
// + current_radius; c += 1)
// 				{
// 					if (c < 0)
// 						continue;
// 					if (c >= this->w)
// 						continue;
// 					std::cout << "\tSetting cost at " << Location(c,
// r, layer) << std::endl;
// this->base_cost_set(this->base_cost_at(Location(c, r, layer)) - current_cost,
// Location(c, r, layer)); 					std::cout <<
// "\tFinised setting cost" << std::endl;
// 				}
// 			}
// 		}
// 	}
// 	std::cout << "Finished remove_route_from_base_cost" << std::endl;
// }

void BoardGrid::remove_route_from_base_cost(const MultipinRoute &route,
                                            int radius, float cost) {
    // std::vector<Location> features = route.features;
    std::cout << "Starting remove_route_from_base_cost" << std::endl;
    for (Location l : route.features) {
        // std::cout << "setting cost for feature " << l << std::endl;
        int layer = l.m_z;

        if (layer > 2) {
            std::cout << "Bad layer: " << l << std::endl;
            exit(-1);
        }

        for (int current_radius = 0; current_radius <= radius;
             current_radius += 1) {
            float current_cost = cost - current_radius;
            if (current_cost <= 0) break;

            for (int r = l.m_y - current_radius; r <= l.m_y + current_radius;
                 r += 1) {
                if (r < 0) continue;
                if (r >= this->h) continue;
                for (int c = l.m_x - current_radius;
                     c <= l.m_x + current_radius; c += 1) {
                    if (c < 0) continue;
                    if (c >= this->w) continue;
                    std::cout << "\tSetting cost at " << Location(c, r, layer)
                              << std::endl;
                    this->base_cost_set(
                        this->base_cost_at(Location(c, r, layer)) -
                            current_cost,
                        Location(c, r, layer));
                    std::cout << "\tFinised setting cost" << std::endl;
                }
            }
        }
    }
    std::cout << "Finished remove_route_from_base_cost" << std::endl;
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

// void BoardGrid::add_route(Route &route)
// {
// 	std::cout << "add_route: Start: " << route.start << ", End: " <<
// route.end << std::endl; 	route.came_from =
// this->dijkstras_with_came_from(route.start); 	route.features.clear();
// 	this->came_from_to_features(route.came_from, route.end, route.features);
// 	// std::cout << "Net: " << route.features.size() << std::endl;

// 	this->add_route_to_base_cost(route, 1, 10);

// 	// Check if out of boundary
// 	for (Location l : route.features)
// 	{
// 		if (l.m_x > this->w || l.m_y > this->h || l.m_z > this->l)
// 		{
// 			std::cout << "Bad route added: " << l << std::endl;
// 			exit(-1);
// 		}
// 	}
// }

void BoardGrid::add_route(MultipinRoute &route) {
    int cost = 10;
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
        this->add_route_to_base_cost(route, current_half_trace_width, cost,
                                     via_size);
    }
}

void BoardGrid::addRoute(MultipinRoute &route) {
    int cost = 10;
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
        this->aStarWithGridCameFrom(route.features, finalEnd);

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
    this->add_route_to_base_cost(route, current_half_trace_width, cost, current_half_via_diameter);
}

void BoardGrid::addRouteWithGridPins(MultipinRoute &route) {
    int cost = 10;

    // TODO
    // check if traceWidth/clearance/viaDiameter persist

    std::cout << __FUNCTION__ << "() route.gridPins.size: " << route.gridPins.size() << std::endl;

    if (route.gridPins.size() <= 1) return;

    this->clearAllCameFromId();

    for (size_t i = 1; i < route.gridPins.size(); ++i) {
        // For early break
        this->setTargetedPins(route.gridPins.at(i).pinWithLayers);
        // For cost estimation (cares about x and y only)
        current_targeted_pin = route.gridPins.at(i).pinWithLayers.front();

        // via size is half_width
        Location finalEnd{0, 0, 0};
        // this->dijkstrasWithGridCameFrom(route.features, current_half_via_diameter);
        if (route.features.empty()) {
            //TODO::
            this->aStarWithGridCameFrom(route.gridPins.front().pinWithLayers, finalEnd);
        } else {
            this->aStarWithGridCameFrom(route.features, finalEnd);
        }

        std::vector<Location> new_features;
        // TODO Fix this
        this->came_from_to_features(finalEnd, new_features);

        for (Location f : new_features) {
            route.features.push_back(f);
        }

        // For early break
        this->clearTargetedPins(route.gridPins.at(i).pinWithLayers);
        // For cost estimation
        current_targeted_pin = Location{0, 0, 0};
    }
    // this->print_features(route.features);
    // TODO
    this->add_route_to_base_cost(route, current_half_trace_width, cost, current_half_via_diameter);
}

// void BoardGrid::ripup_route(Route &route)
// {
// 	std::cout << "Doing ripup" << std::endl;
// 	for (Location l : route.features)
// 	{
// 		if (l.m_x > this->w || l.m_y > this->h || l.m_z > this->l)
// 		{
// 			std::cout << "Bad route to ripup: " << l << std::endl;
// 			exit(-1);
// 		}
// 	}
// 	this->remove_route_from_base_cost(route, 1, 10);
// 	std::cout << "Clearing features" << std::endl;

// 	route.features.clear();
// 	std::cout << "Finished ripup" << std::endl;
// }

void BoardGrid::ripup_route(MultipinRoute &route) {
    std::cout << "Doing ripup" << std::endl;
    for (Location l : route.features) {
        if (l.m_x > this->w || l.m_y > this->h || l.m_z > this->l) {
            std::cout << "Bad route to ripup: " << l << std::endl;
            exit(-1);
        }
    }
    this->remove_route_from_base_cost(route, 1, 10);
    std::cout << "Clearing features" << std::endl;

    route.features.clear();
    std::cout << "Finished ripup" << std::endl;
}

void BoardGrid::set_current_rules(const int clr, const int trWid, int viaDia) {
    current_trace_width = trWid;
    current_half_trace_width = ceil((double)trWid / 2.0);
    current_clearance = clr;
    current_via_diameter = viaDia;
    current_half_via_diameter = ceil((double)viaDia / 2.0);
}

bool BoardGrid::validate_location(const Location &l) const {
    if (l.m_x >= this->w || l.m_x < 0) return false;
    if (l.m_y >= this->h || l.m_y < 0) return false;
    if (l.m_z >= this->l || l.m_z < 0) return false;
    return true;
}
