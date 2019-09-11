// BoardGrid.cpp
#include "BoardGrid.h"

bool Location::operator==(const Location &other) const
{
	return (this->x == other.x) && (this->y == other.y) && (this->z == other.z);
}

std::ostream &operator<<(std::ostream &os, Location const &l)
{
	return os << "Location(" << l.x << ", " << l.y << ", " << l.z << ")";
}

void BoardGrid::base_cost_fill(float value)
{
	for (int i = 0; i < this->size; i++)
	{
		this->base_cost[i] = value;
	}
}

void BoardGrid::working_cost_fill(float value)
{
	for (int i = 0; i < this->size; i++)
	{
		this->working_cost[i] = value;
	}
}

float BoardGrid::base_cost_at(const Location &l) const
{
	return this->base_cost[l.x + l.y * this->w + l.z * this->w * this->h];
}

float BoardGrid::working_cost_at(const Location &l) const
{
	return this->working_cost[l.x + l.y * this->w + l.z * this->w * this->h];
}

void BoardGrid::base_cost_set(float value, const Location &l)
{
	this->base_cost[l.x + l.y * this->w + l.z * this->w * this->h] = value;
}

void BoardGrid::working_cost_set(float value, const Location &l)
{
	this->working_cost[l.x + l.y * this->w + l.z * this->w * this->h] = value;
}

// void BoardGrid::breadth_first_search(const Location &start, const Location &end)
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
// 				(next.second.x < 0) || (next.second.x >= this->w) ||
// 				(next.second.y < 0) || (next.second.y >= this->h) ||
// 				(next.second.z < 0) || (next.second.z >= this->l))
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

// std::unordered_map<Location, Location> BoardGrid::breadth_first_search_with_came_from(const Location &start, const Location &end)
// {
// 	// Location start(sx, sy, sz);
// 	// Location end(ex, ey, ez);

// 	std::cout << "Starting breadth_first_search_with_came_from start: " << start << " end: " << end << std::endl;

// 	std::queue<Location> frontier;
// 	frontier.push(start);

// 	// std::unordered_set<Location> visited; // we can keep track of visited set with the came from map
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
// 				(next.second.x < 0) || (next.second.x >= this->w) ||
// 				(next.second.y < 0) || (next.second.y >= this->h) ||
// 				(next.second.z < 0) || (next.second.z >= this->l))
// 			{
// 				continue; // continue if out of bounds
// 			}
// 			// if (visited.find(next) == visited.end()) { // next not in visited
// 			// if (came_from.find(next.second) == came_from.end()) { // next not in visited
// 			if (came_from.find(next.second) == came_from.end())
// 			{ // next not in visited
// 				frontier.push(next.second);
// 				// visited.insert(next);
// 				came_from[next.second] = current;
// 			}
// 		}
// 	}

// 	return came_from;
// }

// std::unordered_map<Location, Location> BoardGrid::dijkstras_with_came_from(const Location &start)
// {
// 	// Location start(sx, sy, sz);
// 	// Location end(ex, ey, ez);

// 	//std::cout << "Starting dijkstras_with_came_from start: " << start << " end: " << end << std::endl;
// 	std::cout << "Starting dijkstras_with_came_from ==TwoPin== start: " << start << std::endl;

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

// 		//std::cout << "Visiting " << current << ", frontierSize: "<< frontier.size() << std::endl;

// 		// if (current == end) {
// 		// std::cout << "Found goal, exiting early" << std::endl;
// 		// break;
// 		// }

// 		//std::cout << "Got " << this->neighbors(current).size() << " neighbors" << std::endl;
// 		std::array<std::pair<float, Location>, 10> &&aNeighbors = this->neighbors(current);

// 		//for (std::pair<float, Location> next : this->neighbors(current)) {
// 		for (std::pair<float, Location> next : aNeighbors)
// 		{
// 			if (
// 				(next.second.x < 0) || (next.second.x >= this->w) ||
// 				(next.second.y < 0) || (next.second.y >= this->h) ||
// 				(next.second.z < 0) || (next.second.z >= this->l))
// 			{
// 				// std::cout << "\tOut of bounds neighbor " < next.second << std::endl;
// 				continue; // continue if out of bounds
// 			}

// 			//std::cout << "\tWorking on neighbor " << next.second << std::endl;

// 			// this->at(Location l) give the cost of moving to l
// 			// this->working_cost_at(Location l) gives the current best cost to get to l
// 			float new_cost = this->working_cost_at(current) + this->base_cost_at(next.second) + next.first;

// 			// std::cout << "\t\tnew_cost: " << new_cost << std::endl;
// 			// std::cout << "\t\t\twc: " << this->working_cost_at(current) <<  " bc: " << this->base_cost_at(next.second) << " n.f: " << next.first << std::endl;

// 			// std::cout << "\t\told cost: " << this->working_cost_at(next.second) << std::endl;

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
	const std::vector<Location> &route)
{
	std::cout << "Starting dijkstras_with_came_from ==Multipin== nets: route.size() = " << route.size() << std::endl;

	// For path to multiple points
	// Searches from the multiple points to every other point
	this->working_cost_fill(std::numeric_limits<float>::infinity());

	LocationQueue<Location, float> frontier;		  // search frontier
	std::unordered_map<Location, Location> came_from; // cheapest neighbor
	for (Location start : route)
	{
		this->working_cost_set(0.0, start);
		frontier.push(start, 0.0);
		came_from[start] = start;
	}

	std::cout << "came_from.size() = " << came_from.size() << std::endl;

	while (!frontier.empty())
	{
		Location current = frontier.front();
		frontier.pop();

		//std::cout << "Visiting " << current << ", frontierSize: "<< frontier.size() << std::endl;

		for (std::pair<float, Location> next : this->neighbors(current))
		{
			if (
				(next.second.x < 0) || (next.second.x >= this->w) ||
				(next.second.y < 0) || (next.second.y >= this->h) ||
				(next.second.z < 0) || (next.second.z >= this->l))
			{
				continue; // continue if out of bounds
			}
			float new_cost = this->working_cost_at(current) + this->base_cost_at(next.second) + next.first;
			if (new_cost < this->working_cost_at(next.second))
			{
				this->working_cost_set(new_cost, next.second);
				came_from[next.second] = current;
				frontier.push(next.second, new_cost);
			}
		}
	}
	return came_from;
}

std::array<std::pair<float, Location>, 10> BoardGrid::neighbors(const Location &l) const
{
	std::array<std::pair<float, Location>, 10> ns;

	// left
	ns[0].first = 1.0;
	ns[0].second.x = l.x + 1;
	ns[0].second.y = l.y;
	ns[0].second.z = l.z;
	// right
	ns[1].first = 1.0;
	ns[1].second.x = l.x - 1;
	ns[1].second.y = l.y;
	ns[1].second.z = l.z;

	// forward
	ns[2].first = 1.0;
	ns[2].second.x = l.x;
	ns[2].second.y = l.y + 1;
	ns[2].second.z = l.z;
	// back
	ns[3].first = 1.0;
	ns[3].second.x = l.x;
	ns[3].second.y = l.y - 1;
	ns[3].second.z = l.z;

	// up
	ns[4].first = LAYER_CHANGE_COST;
	ns[4].second.x = l.x;
	ns[4].second.y = l.y;
	ns[4].second.z = l.z + 1;
	// down
	ns[5].first = LAYER_CHANGE_COST;
	ns[5].second.x = l.x;
	ns[5].second.y = l.y;
	ns[5].second.z = l.z - 1;

	//lf
	ns[6].first = DIAGONAL_COST;
	ns[6].second.x = l.x - 1;
	ns[6].second.y = l.y + 1;
	ns[6].second.z = l.z;

	//lb
	ns[7].first = DIAGONAL_COST;
	ns[7].second.x = l.x - 1;
	ns[7].second.y = l.y - 1;
	ns[7].second.z = l.z;

	//rf
	ns[8].first = DIAGONAL_COST;
	ns[8].second.x = l.x + 1;
	ns[8].second.y = l.y + 1;
	ns[8].second.z = l.z;

	//rb
	ns[9].first = DIAGONAL_COST;
	ns[9].second.x = l.x + 1;
	ns[9].second.y = l.y - 1;
	ns[9].second.z = l.z;

	return ns;
}

void BoardGrid::pprint()
{
	char colors[11] = " .,:=+*#%@";
	// long
	// colors[36]    = {' ','.',':','░','#','▒','▓','█'};
	int num_colors = 10;

	float max_val = 0.0;
	for (int i = 0; i < this->size; i += 1)
	{
		if (this->base_cost[i] > max_val)
			max_val = this->base_cost[i];
	}

	for (int l = 0; l < this->l; l += 1)
	{
		std::cout << std::endl
				  << "Layer: " << l << std::endl;

		for (int r = 0; r < this->h; r += 1)
		{
			for (int c = 0; c < this->w; c += 1)
			{
				int current_color = this->base_cost_at(Location(c, r, l)) / max_val * num_colors;
				std::cout << colors[current_color] << " ";
				// std::cout << current_color << " ";

				// std::cout << std::setw(6) << std::setprecision(2) << std::right << this->at(c, r, l);
			}
			std::cout << std::endl;
		}
	}
}


float BoardGrid::cost_of_via_at(const Location &l){
	return 10.0;
}


void BoardGrid::print_came_from(const std::unordered_map<Location, Location> &came_from, const Location &end)
{
	// Location end(ex, ey, ez);

	std::cout << "Came froms: " << std::endl;
	std::cout << "\tsize: " << came_from.size() << std::endl;

	for (int l = 0; l < this->l; l += 1)
	{
		std::cout << std::endl
				  << "Layer: " << l << std::endl;

		for (int r = 0; r < this->h; r += 1)
		{
			for (int c = 0; c < this->w; c += 1)
			{

				Location current(c, r, l);

				if (current == end)
				{
					std::cout << "# ";
					continue; // goal
				}

				if (came_from.find(current) == came_from.end())
				{
					std::cout << ". "; // not found
					continue;		   // unexplored
				}

				Location cf = came_from.find(current)->second;

				if (current.x > cf.x)
				{
					std::cout << "> ";
				}
				else if (current.x < cf.x)
				{
					std::cout << "< ";
				}
				else if (current.y > cf.y)
				{
					std::cout << "V ";
				}
				else if (current.y < cf.y)
				{
					std::cout << "^ ";
				}
				else if (current.z > cf.z)
				{
					std::cout << "X ";
				}
				else if (current.z < cf.z)
				{
					std::cout << "O ";
				}
				else if ((current.x == cf.x) && (current.y == cf.y) && (current.z == cf.z))
				{
					std::cout << "* "; // start
				}
			}
			std::cout << std::endl;
		}
	}
}

// void BoardGrid::add_route_to_base_cost(const Route &route, int radius, float cost)
// {
// 	// std::vector<Location> features = route.features;
// 	for (Location l : route.features)
// 	{
// 		// std::cout << "setting cost for feature " << l << std::endl;
// 		int layer = l.z;

// 		for (int current_radius = 0; current_radius <= radius; current_radius += 1)
// 		{
// 			float current_cost = cost - current_radius;
// 			if (current_cost <= 0)
// 				break;

// 			for (int r = l.y - current_radius; r <= l.y + current_radius; r += 1)
// 			{
// 				if (r < 0)
// 					continue;
// 				if (r >= this->h)
// 					continue;
// 				for (int c = l.x - current_radius; c <= l.x + current_radius; c += 1)
// 				{
// 					if (c < 0)
// 						continue;
// 					if (c >= this->w)
// 						continue;
// 					// std::cout << "\tsetting cost at " << Location(c, r, layer) << std::endl;
// 					this->base_cost_set(this->base_cost_at(Location(c, r, layer)) +
// 											current_cost,
// 										Location(c, r, layer));
// 				}
// 			}
// 		}
// 	}
// }

void BoardGrid::add_via_cost(const Location &l) {
	int radius = 10;
	float cost = 10.0;
	for (int z = 0; z < this->l; z += 1) {
		for (int y = -radius; y < radius; y += 1) {
			for (int x = -radius; x <= radius; x += 1){
				this->via_cost[(l.x+x) + (l.y+y) * this->w + (l.z+z) * this->w * this->h] += cost;
			}
		}
	}
}

void BoardGrid::remove_via_cost(const Location &l) {
	int radius = 10;
	float cost = 10.0;
	for (int z = 0; z < this->l; z += 1) {
		for (int y = -radius; y < radius; y += 1) {
			for (int x = -radius; x <= radius; x += 1){
				this->via_cost[(l.x+x) + (l.y+y) * this->w + (l.z+z) * this->w * this->h] -= cost;
			}
		}
	}
}

void BoardGrid::add_route_to_base_cost(const MultipinRoute &route, int radius, float cost)
{
	// std::vector<Location> features = route.features;
	Location last_location = route.features[0];
	for (Location l : route.features)
	{
		// std::cout << "setting cost for feature " << l << std::endl;
		int layer = l.z;
		if ((l.z != last_location.z) && (l.x == last_location.x) && (l.y == last_location.y)) {
			this->add_via_cost(l);
		}

		for (int current_radius = 0; current_radius <= radius; current_radius += 1)
		{
			float current_cost = cost - current_radius;
			if (current_cost <= 0)
				break;

			for (int r = l.y - current_radius; r <= l.y + current_radius; r += 1)
			{
				if (r < 0)
					continue;
				if (r >= this->h)
					continue;
				for (int c = l.x - current_radius; c <= l.x + current_radius; c += 1)
				{
					if (c < 0)
						continue;
					if (c >= this->w)
						continue;
					// std::cout << "\tsetting cost at " << Location(c, r, layer) << std::endl;
					this->base_cost_set(
						this->base_cost_at(Location(c, r, layer)) + current_cost,
						Location(c, r, layer)
					);
				}
			}
		}
	}
}

// void BoardGrid::remove_route_from_base_cost(const Route &route, int radius, float cost)
// {
// 	// std::vector<Location> features = route.features;
// 	std::cout << "Starting remove_route_from_base_cost" << std::endl;
// 	for (Location l : route.features)
// 	{
// 		// std::cout << "setting cost for feature " << l << std::endl;
// 		int layer = l.z;

// 		if (layer > 2)
// 		{
// 			std::cout << "Bad layer: " << l << std::endl;
// 			exit(-1);
// 		}

// 		for (int current_radius = 0; current_radius <= radius; current_radius += 1)
// 		{
// 			float current_cost = cost - current_radius;
// 			if (current_cost <= 0)
// 				break;

// 			for (int r = l.y - current_radius; r <= l.y + current_radius; r += 1)
// 			{
// 				if (r < 0)
// 					continue;
// 				if (r >= this->h)
// 					continue;
// 				for (int c = l.x - current_radius; c <= l.x + current_radius; c += 1)
// 				{
// 					if (c < 0)
// 						continue;
// 					if (c >= this->w)
// 						continue;
// 					std::cout << "\tSetting cost at " << Location(c, r, layer) << std::endl;
// 					this->base_cost_set(this->base_cost_at(Location(c, r, layer)) - current_cost, Location(c, r, layer));
// 					std::cout << "\tFinised setting cost" << std::endl;
// 				}
// 			}
// 		}
// 	}
// 	std::cout << "Finished remove_route_from_base_cost" << std::endl;
// }

void BoardGrid::remove_route_from_base_cost(const MultipinRoute &route, int radius, float cost)
{
	// std::vector<Location> features = route.features;
	std::cout << "Starting remove_route_from_base_cost" << std::endl;
	for (Location l : route.features)
	{
		// std::cout << "setting cost for feature " << l << std::endl;
		int layer = l.z;

		if (layer > 2)
		{
			std::cout << "Bad layer: " << l << std::endl;
			exit(-1);
		}

		for (int current_radius = 0; current_radius <= radius; current_radius += 1)
		{
			float current_cost = cost - current_radius;
			if (current_cost <= 0)
				break;

			for (int r = l.y - current_radius; r <= l.y + current_radius; r += 1)
			{
				if (r < 0)
					continue;
				if (r >= this->h)
					continue;
				for (int c = l.x - current_radius; c <= l.x + current_radius; c += 1)
				{
					if (c < 0)
						continue;
					if (c >= this->w)
						continue;
					std::cout << "\tSetting cost at " << Location(c, r, layer) << std::endl;
					this->base_cost_set(this->base_cost_at(Location(c, r, layer)) - current_cost, Location(c, r, layer));
					std::cout << "\tFinised setting cost" << std::endl;
				}
			}
		}
	}
	std::cout << "Finished remove_route_from_base_cost" << std::endl;
}

void BoardGrid::came_from_to_features(
	const std::unordered_map<Location, Location> &came_from,
	const Location &end,
	std::vector<Location> &features) const
{

	// std::vector<Location> features;
	// features.insert(end);
	std::cout << "Starting came_from_to_features " << std::endl;

	if (!this->validate_location(end))
		std::cout << "Bad end for came_from_to_features" << std::endl;

	features.push_back(end);
	Location current = end;

	while (came_from.find(current) != came_from.end())
	{ // while not the start
		Location next = came_from.find(current)->second;
		// if (features.find(next) != features.end()) { // next already in features, loops or start
		// break;
		// }
		if (next == current)
		{
			break;
		}
		// features.insert(next);
		features.push_back(next);
		current = next;
	}

	for (Location l : features)
	{
		//std::cout << "\t" << l << std::endl;
		if (l.z > 2)
		{
			exit(-1);
		}
	}

	std::cout << "Finished came_from_to_features " << std::endl;

	// return features;
}

std::vector<Location> BoardGrid::came_from_to_features(
	const std::unordered_map<Location, Location> &came_from,
	const Location &end) const
{
	std::vector<Location> features;
	// features.insert(end);
	this->came_from_to_features(came_from, end, features);
	return features;
}

void BoardGrid::print_route(const std::unordered_map<Location, Location> &came_from, const Location &end)
{
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
	// while (came_from.find(current) != came_from.end()) { // while not the start

	// 	Location next = came_from.find(current)->second;
	// 	// std::cout << "next: " << next << std::endl;

	// 	if (features.find(next) != features.end()) { // next already in features, loops or start
	// 		break;
	// 	}
	// 	features.insert(next);
	// 	// std::cout << "Got next features: " << next <<
	// 	 std::endl;
	// 	current = next;
	// }

	std::cout << "Printing features" << std::endl;
	for (int l = 0; l < this->l; l += 1)
	{
		std::cout << std::endl
				  << "Layer: " << l << std::endl;

		for (int r = 0; r < this->h; r += 1)
		{
			for (int c = 0; c < this->w; c += 1)
			{

				Location current(c, r, l);

				if (std::find(features.begin(), features.end(), current) != features.end())
				{
					// if (features.find(current) != features.end()) {
					std::cout << "# "; // contains feature
				}
				else
				{
					std::cout << ". ";
				}
			}
			std::cout << std::endl;
		}
	}
}

void BoardGrid::print_features(std::vector<Location> features)
{
	std::cout << "Printing features" << std::endl;
	for (int l = 0; l < this->l; l += 1)
	{
		std::cout << std::endl
				  << "Layer: " << l << std::endl;

		for (int r = 0; r < this->h; r += 1)
		{
			for (int c = 0; c < this->w; c += 1)
			{

				Location current(c, r, l);

				if (std::find(features.begin(), features.end(), current) != features.end())
				{
					// if (features.find(current) != features.end()) {
					std::cout << "# "; // contains feature
				}
				else
				{
					std::cout << ". ";
				}
			}
			std::cout << std::endl;
		}
	}
}

// void BoardGrid::add_route(Route &route)
// {
// 	std::cout << "add_route: Start: " << route.start << ", End: " << route.end << std::endl;
// 	route.came_from = this->dijkstras_with_came_from(route.start);
// 	route.features.clear();
// 	this->came_from_to_features(route.came_from, route.end, route.features);
// 	// std::cout << "Net: " << route.features.size() << std::endl;

// 	this->add_route_to_base_cost(route, 1, 10);

// 	// Check if out of boundary
// 	for (Location l : route.features)
// 	{
// 		if (l.x > this->w || l.y > this->h || l.z > this->l)
// 		{
// 			std::cout << "Bad route added: " << l << std::endl;
// 			exit(-1);
// 		}
// 	}
// }

void BoardGrid::add_route(MultipinRoute &route)
{
	int num_pins = route.pins.size();

	if (num_pins <= 0)
	{
		return;
	}
	else if (num_pins == 1)
	{
		return;
	}
	else
	{
		route.features.push_back(route.pins[0]);

		for (Location pin : route.pins)
		{
			std::unordered_map<Location, Location> came_from = this->dijkstras_with_came_from(route.features);
			//this->print_came_from(came_from, pin);
			std::vector<Location> new_features = this->came_from_to_features(came_from, pin);
			for (Location f : new_features)
			{
				route.features.push_back(f);
			}
		}
		//this->print_features(route.features);
		this->add_route_to_base_cost(route, 1, 10);
	}
}

// void BoardGrid::ripup_route(Route &route)
// {
// 	std::cout << "Doing ripup" << std::endl;
// 	for (Location l : route.features)
// 	{
// 		if (l.x > this->w || l.y > this->h || l.z > this->l)
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

void BoardGrid::ripup_route(MultipinRoute &route)
{
	std::cout << "Doing ripup" << std::endl;
	for (Location l : route.features)
	{
		if (l.x > this->w || l.y > this->h || l.z > this->l)
		{
			std::cout << "Bad route to ripup: " << l << std::endl;
			exit(-1);
		}
	}
	this->remove_route_from_base_cost(route, 1, 10);
	std::cout << "Clearing features" << std::endl;

	route.features.clear();
	std::cout << "Finished ripup" << std::endl;
}

bool BoardGrid::validate_location(const Location &l) const
{
	if (l.x >= this->w)
		return false;
	if (l.y >= this->h)
		return false;
	if (l.z >= this->l)
		return false;
	return true;
}
