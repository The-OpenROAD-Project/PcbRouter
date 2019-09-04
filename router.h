#ifndef ROUTER_H
#define ROUTER_H

#include "mymath.h"
//#include "layer.h"
#include <map>
#include <unordered_map>
#include <set>
#include <cmath>
#include <string>

class pcb;
class net;

struct pin
{
  std::string m_name;
  std::string m_comp_name;
  std::string m_instance_name;
};

struct pad
{
	bool operator==(const pad &t) const {
		return std::tie(m_radius, m_gap, m_pos, m_shape)
				== std::tie(t.m_radius, t.m_gap, t.m_pos, t.m_shape); }
	bool operator<(const pad &t) const { return m_pos < t.m_pos; }
	double m_radius;
	double m_gap;
	point_3d m_pos;
	points_2d m_shape;
};
typedef std::vector<pad> pads;

//grid node and collections
struct node
{
	bool operator<(const node &n) const {
		return std::tie(m_x, m_y, m_z) < std::tie(n.m_x, n.m_y, n.m_z); }
	bool operator==(const node &n) const {
		return std::tie(m_x, m_y, m_z) == std::tie(n.m_x, n.m_y, n.m_z); }
	int manhattan_distance(const node &n) const
	{
		auto dx = m_x - n.m_x;
		auto dy = m_y - n.m_y;
		auto dz = m_z - n.m_z;
		return std::abs(dx) + std::abs(dy) + std::abs(dz);
	}
	int euclidian_distance(const node &n) const
	{
		auto dx = m_x - n.m_x;
		auto dy = m_y - n.m_y;
		auto dz = m_z - n.m_z;
		return int(sqrt(dx * dx + dy * dy + dz * dz));
	}
	node mid(const node &n) const
	{
		return node{(m_x + n.m_x)/2, (m_y + n.m_y)/2, (m_z + n.m_z)/2};
	}
	int m_x;
	int m_y;
	int m_z;
};
typedef std::vector<node> nodes;
typedef std::vector<nodes> nodess;
typedef std::set<node> node_set;

namespace std
{
	template <>
	struct hash<node>
	{
		auto operator()(const node& n) const
		{
			return (std::hash<int>()(n.m_x)
					^ std::hash<int>()(n.m_y)
					^ std::hash<int>()(n.m_z));
		}
	};
}

//netlist structures
typedef std::vector<point_3d> path;
typedef std::vector<path> paths;

struct track
{
	std::string m_id;
	double m_track_radius;
	double m_via_radius;
	double m_gap;
	pads m_pads;
	paths m_paths;
};
typedef std::vector<track> tracks;

//sortable node
struct sort_node
{
	int m_mark;
	node m_node;
};
typedef std::vector<sort_node> sort_nodes;

typedef std::vector<net> nets;
typedef double (*dfunc_t)(const point_3d &, const point_3d &);

//net object
class net
{
  public:
  net(const double clearance = 0.0, 
      const double trace_width = 0.0,
      const double via_dia = 0.0,
      const double via_drill = 0.0,
      const double uvia_dia = 0.0,
      const double uvia_drill = 0.0)
    :m_clearance(clearance), m_trace_width(trace_width), m_via_dia(via_dia), m_via_drill(via_drill), m_uvia_dia(uvia_dia), m_uvia_drill(uvia_drill){}
  std::vector<pin> pins;
  double getClearance() {return m_clearance;}
  bool setId (int &id) { m_id = id; return true;}
  bool getId (int *id) { if(!id) return false; *id = m_id; return true;}
  int getId () { return m_id; }
/*public:
	net(const track &t, pcb *pcb);
	bool route();
	void print_net();
	void remove();
	void shuffle_topology();

	int m_area;
	double m_radius;
	paths m_wires;
	nodess m_paths;
	std::vector<layers::line> m_paths_collision_lines;
	pads m_pads;
	std::vector<layers::line> m_pad_collision_lines;
	std::vector<nodes> m_pad_end_nodes;
	std::vector<layers::line> m_wire_collision_lines;
	node_set m_wire_nodes;
	layer::aabb m_bbox;

private:
	void add_pad_collision_lines();
	void sub_pad_collision_lines();
	void add_wire_collision_lines();
	void sub_wire_collision_lines();
	std::vector<layers::line> paths_collision_lines() const;
	void add_paths_collision_lines();
	void sub_paths_collision_lines();
	nodess optimise_paths(const nodess &paths);
	std::pair<nodes, bool> backtrack_path(const node_set &vis, const node &end,
		 								double radius, double via, double gap);
*/
  private:
	pcb *m_pcb;
	int m_id;
	//double m_via;
        double m_clearance;
        double m_trace_width;
        double m_via_dia;
        double m_via_drill;
        double m_uvia_dia;
        double m_uvia_drill;
	//double m_gap;
};

//pcb class
/*class pcb
{
public:
	//dimensions of pcb board in grid points/layers
	struct dims
	{
		double m_width;
		double m_height;
		double m_depth;
	};

	pcb(const dims &dims, const nodess &rfvs, const nodess &rpvs,
		int res, int verb, int quant, int viascost);
	~pcb();
	auto get_node(const node &n);
	void add_track(track &t);
	bool route(double timeout);
	int cost();
	void increase_quantization();
	void print_pcb();
	void print_netlist();
	void print_stats();
	point_3d node_to_point(const node &n);
	node point_to_node(const point_3d &p);
	point_3d node_to_pad_point(const node &n);
	node pad_point_to_node(const point_3d &p);
	nodes &all_not_shorting(const nodes &gather, const node &n, double radius, double gap);
	nodes &all_not_shorting_via(const nodes &gather, const node &n, double radius, double gap);
	nodes &all_nearer_sorted(const nodess &vec, const node &n);
	void mark_distances(double radius, double via, double gap,
		const node_set &starts, const nodes &ends, const node &mid, double mid_scale);
	void unmark_distances();

	int m_resolution;
	int m_quantization;
	int m_depth;
	int m_viascost;
	std::map<node, point_3d> m_deform;
	layers m_layers;
	layers m_via_layers;
	nodess m_routing_flood_vectors;
	nodess m_routing_path_vectors;

private:
	void set_node(const node &n, unsigned int value);
	sort_nodes &all_marked(const nodess &vec, const node &n);
	nodes &all_not_marked(const nodess &vec, const node &n);
	void reset_areas();
	void shuffle_netlist();
	int hoist_net(int n);
	void remove_netlist();

	int m_width;
	int m_height;
	int m_stride;
	int m_verbosity;
	nets m_netlist;
	std::vector<int> m_nodes;
};*/

#endif
