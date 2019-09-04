#include "kicadPcbDataBase.h"


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
  {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

auto shape_to_cords(const points_2d &shape, double a1, double a2)
{
  auto cords = points_2d{};
  auto rads = fmod(a1+a2, 2*M_PI);
  auto s = sin(rads);
  auto c = cos(rads);
  for (auto &p : shape)
  {
    auto px = double(c*p.m_x - s*p.m_y);
    auto py = double(s*p.m_x + c*p.m_y);
    cords.push_back(point_2d{px, py});
  }
  return cords;
}

//read input till given byte appears
auto read_until(std::istream &in, char c)
{
  char input;
  while (in.get(input))
  {
    if (input == c) return false;
  }
  return true;
}

//read whitespace
auto read_whitespace(std::istream &in)
{
  for (;;)
  {
    auto b = in.peek();
    if (b != '\t' && b != '\n' && b != '\r' && b != ' ') break;
    char c;
    in.get(c);
  }
}

auto read_node_name(std::istream &in)
{
  std::string s;
  for (;;)
  {
    auto b = in.peek();
    if (b == '\t' || b == '\n' || b == '\r' || b == ' ' || b == ')') break;
    char c;
    in.get(c);
    s.push_back(c);
  }
  return s;
}

auto read_string(std::istream &in)
{
  std::string s;
  for (;;)
  {
    auto b = in.peek();
    if (b == '\t' || b == '\n' || b == '\r' || b == ')' || b == ' ') break;
    char c;
    in.get(c);
    s.push_back(c);
  }
  return tree{s, {}};
}

auto read_quoted_string(std::istream &in)
{
  std::string s;
  s.push_back('"');
  auto a = in.peek();
  for (;;)
  {
    auto b = in.peek();
    if (b == '"' && a != '\\') break;
    char c;
    in.get(c);
    s.push_back(c);
    a = b;
  }
  s.push_back('"');
  return tree{s, {}};
}

void ss_reset(std::stringstream &ss, const std::string &s)
{
  ss.str(s);
  ss.clear();
}

void get_value(std::stringstream &ss, std::vector<tree>::iterator t, int &x)
{
  ss_reset(ss, t->m_value);
  ss >> x;
}

void get_value(std::stringstream &ss, std::vector<tree>::iterator t, double &x)
{
  ss_reset(ss, t->m_value);
  ss >> x;
}

void get_2d(std::stringstream &ss, std::vector<tree>::iterator t, double &x, double &y)
{
  get_value(ss, t, x);
  get_value(ss, t + 1, y);
}

void get_rect(std::stringstream &ss, std::vector<tree>::iterator t, double &x1, double &y1, double &x2, double &y2)
{
  get_2d(ss, t, x1, y1);
  get_2d(ss, t + 2, x2, y2);
}
void kicadPcbDataBase::printKicadPcb(const tree &t, int indent)
{

  if (!t.m_value.empty())
  {
    for (auto i = 1; i < indent; ++i) std::cout << " "; 
    if (!t.m_branches.empty()) std::cout << "(";
    std::cout << t.m_value;
    //	if (t.m_branches.empty() && isYoungest) std::cout << ")";
  }
  //if (t.m_branches.empty()) std::cout << ")\n";
  //else 
  if (!t.m_branches.empty()) {
    for (auto &ct : t.m_branches) {
      printTree(ct, indent+1);
    }
    std::cout << ")" << std::endl;
  }
}


tree kicadPcbDataBase::readTree(std::istream &in)
{
  read_until(in, '(');
  read_whitespace(in);
  auto t = tree{read_node_name(in), {}};
  for (;;)
  {
    read_whitespace(in);
    auto b = in.peek();
    char c;
    if (b == EOF) {
      break;
    }

    if (b == ')')
    {
      in.get(c);
      break;
    }
    if (b == '(')
    {
      t.m_branches.push_back(readTree(in));
      continue;
    }
    if (b == '"')
    {
      in.get(c);
      t.m_branches.push_back(read_quoted_string(in));
      in.get(c);
      continue;
    }
    t.m_branches.push_back(read_string(in));
  }
  return t;
}

void kicadPcbDataBase::printTree(const tree &t, int indent)
{
  if (!t.m_value.empty())
  {
    for (auto i = 1; i < indent; ++i) std::cout << "  "; 
    std::cout << t.m_value << "\n";
  }
  for (auto &ct : t.m_branches) {
    printTree(ct, indent+1);
  }
}


bool kicadPcbDataBase::parseKicadPcb()
{

  std::ifstream arg_infile;
  arg_infile.open(m_fileName, std::ifstream::in);
  if (!arg_infile.is_open()) return false;
  std::istream &in = arg_infile;

  //create tree from input
  auto tree = readTree(in);
  //printTree(tree, 0);


  std::stringstream ss;

  for (auto &&sub_node : tree.m_branches) {
    //layer part
    if (sub_node.m_value == "layers") {

      for (auto &&layer_node : sub_node.m_branches)
      {
        auto layer_index = 0;
        auto layer_name = layer_node.m_branches[1].m_value;
        get_value(ss, begin(layer_node.m_branches), layer_index);
        layer_to_index_map[layer_name] = layer_index;
      }
    } 
    //net
    else if (sub_node.m_value == "net") {
      auto net_index = 0;
      auto net_name = sub_node.m_branches[1].m_value;
      get_value(ss, begin(sub_node.m_branches), net_index);
      index_to_net_map[net_index] = net_name;
    }
    //net class
    else if (sub_node.m_value == "net_class") {
      for (auto &&net_class_node : sub_node.m_branches)
      {
        double m_clearance, m_trace_width, m_via_dia, m_via_drill, m_uvia_dia, m_uvia_drill;
        if (net_class_node.m_value == "clearance")
          get_value(ss, begin(net_class_node.m_branches), m_clearance);
        else if (net_class_node.m_value == "trace_width")
          get_value(ss, begin(net_class_node.m_branches), m_trace_width);
        else if (net_class_node.m_value == "via_dia")
          get_value(ss, begin(net_class_node.m_branches), m_via_dia);
        else if (net_class_node.m_value == "via_drill")
          get_value(ss, begin(net_class_node.m_branches), m_via_drill);
        else if (net_class_node.m_value == "uvia_dia")
          get_value(ss, begin(net_class_node.m_branches), m_uvia_dia);
        else if (net_class_node.m_value == "uvia_drill")
          get_value(ss, begin(net_class_node.m_branches), m_uvia_drill);
        else if (net_class_node.m_value == "add_net") {
          auto the_net = net(m_clearance, m_trace_width, m_via_dia, m_via_drill, m_uvia_dia, m_uvia_drill);
          auto net_name = net_class_node.m_branches[0].m_value;
          name_to_net_map[net_name] = the_net;
        }
      }
    }
    else if (sub_node.m_value == "module") {
      //padstack?? hole??
      if (sub_node.m_branches[0].m_value == "\"\"" ) {



      }
      //instance and component
      else {

        auto name = split(sub_node.m_branches[0].m_value, ':');
        auto component_name = name[1]; 

        auto layer = sub_node.m_branches[1].m_branches[0].m_value;
        auto the_instance = instance{};
        the_instance.m_comp = component_name;

        get_2d(ss, begin(sub_node.m_branches[4].m_branches), the_instance.m_x, the_instance.m_y);
        //std::cout << node.m_branches[0].m_value << " " << node.m_branches[1].m_value << std::endl;
        if (int(sub_node.m_branches[4].m_branches.size()) == 3)
          get_value(ss,begin(sub_node.m_branches[4].m_branches)+2, the_instance.m_angle);
        else the_instance.m_angle = 0;

        comp_it = name_to_component_map.find(component_name);

        auto the_comp = component{component_name, std::map<std::string, padstack>{}};

        for (auto &&module_node : sub_node.m_branches) {

          if (module_node.m_value == "fp_text" && module_node.m_branches[0].m_value == "reference") {
            the_instance.m_name = module_node.m_branches[1].m_value;
          }
          if(comp_it == name_to_component_map.end()) { 
            if (module_node.m_value == "fp_line") {
              auto the_line = line{};
              get_2d(ss, begin(module_node.m_branches[0].m_branches), the_line.m_start.m_x, the_line.m_start.m_y);
              get_2d(ss, begin(module_node.m_branches[1].m_branches), the_line.m_end.m_x, the_line.m_end.m_y);
              get_value(ss, begin(module_node.m_branches[3].m_branches), the_line.m_width);
              the_comp.m_lines.push_back(the_line);
            }
            else if (module_node.m_value == "fp_poly") {
              auto the_poly = poly{};
              for (auto &&cor_node : module_node.m_branches[0].m_branches) {
                auto the_point = point_2d{};
                get_2d(ss, begin(cor_node.m_branches), the_point.m_x, the_point.m_y);
                the_poly.m_shape.push_back(the_point);
              }
              get_value(ss, begin(module_node.m_branches[2].m_branches), the_poly.m_width);
              the_comp.m_polys.push_back(the_poly);
            }
            else if (module_node.m_value == "fp_circle") {
              auto the_circle = circle{};
              get_2d(ss, begin(module_node.m_branches[0].m_branches), the_circle.m_center.m_x, the_circle.m_center.m_y);
              get_2d(ss, begin(module_node.m_branches[1].m_branches), the_circle.m_end.m_x, the_circle.m_end.m_y);
              get_value(ss, begin(module_node.m_branches[3].m_branches), the_circle.m_width);
              the_comp.m_circles.push_back(the_circle);
            }
            else if (module_node.m_value == "fp_arc") {
              auto the_arc = arc{};
              get_2d(ss, begin(module_node.m_branches[0].m_branches), the_arc.m_start.m_x, the_arc.m_start.m_y);
              get_2d(ss, begin(module_node.m_branches[1].m_branches), the_arc.m_end.m_x, the_arc.m_end.m_y);
              get_value(ss, begin(module_node.m_branches[2].m_branches), the_arc.m_angle);
              get_value(ss, begin(module_node.m_branches[4].m_branches), the_arc.m_width);
              the_comp.m_arcs.push_back(the_arc);
            }

            else if (module_node.m_value == "pad") {
              // if(module_node.m_branches[1].m_value != "smd") continue;
              auto the_pin = padstack{};
              auto the_point = point_2d{};
              auto form = module_node.m_branches[2].m_value;
              auto type = module_node.m_branches[1].m_value;
              the_pin.m_name = module_node.m_branches[0].m_value;
              the_pin.setForm(form);
              the_pin.setType(type);

              get_2d(ss, begin(module_node.m_branches[3].m_branches), the_pin.m_x, the_pin.m_y);
              get_2d(ss, begin(module_node.m_branches[4].m_branches), the_point.m_x, the_point.m_y);
              the_pin.m_size = the_point;
              if ((int)module_node.m_branches[3].m_branches.size() == 3) {
                get_value(ss, begin(module_node.m_branches[3].m_branches)+2, the_pin.m_angle);
                the_pin.m_angle = the_pin.m_angle - the_instance.m_angle;
              }
              else the_pin.m_angle = 0;
              the_comp.m_pin_map[the_pin.m_name] = the_pin;

              for (auto &&layer_node : module_node.m_branches[5].m_branches) {
                the_pin.m_layers.push_back(layer_node.m_value);
              }
            }
            name_to_component_map[component_name] = the_comp;
          }
        }

        //Find the connection of the pad
        for (auto &&pad_node : sub_node.m_branches)
        {
          if (pad_node.m_value == "pad") {
            auto pin_name = pad_node.m_branches[0].m_value;
            int net_index = 0;
            std::string net_name = "";
            for (auto &&net_node : pad_node.m_branches) {
              if (net_node.m_value == "net") {
                net_name = net_node.m_branches[1].m_value;
                get_value(ss, begin(net_node.m_branches), net_index);

                auto the_pin = pin{pin_name, component_name, the_instance.m_name};
                the_instance.m_pin_net_map[pin_name] = net_index;
                auto &&the_net = name_to_net_map[net_name];
                the_net.setId(net_index);
                the_net.pins.push_back(the_pin);

              }
            }
          }
        }
        name_to_instance_map[the_instance.m_name] = the_instance;
      }
    }
    //TODO: calculate outline  
    else if (sub_node.m_value == "gr_line")
    {
    }

    else if (sub_node.m_value == "segment")
    {
      auto x = 0.0, y = 0.0, z = 0.0;
      auto width = 0.0;
      auto net = 0;
      auto segment = path{};

      z = layer_to_index_map[sub_node.m_branches[4].m_branches[0].m_value];
      get_2d(ss, begin(sub_node.m_branches[0].m_branches), x, y);
      segment.push_back(point_3d(x, y, z));
      get_2d(ss, begin(sub_node.m_branches[1].m_branches), x, y);
      segment.push_back(point_3d(x, y, z));
      get_value(ss, begin(sub_node.m_branches[3].m_branches), width);
      get_value(ss, begin(sub_node.m_branches[4].m_branches), net);
      net_to_segments_map[net].emplace_back(std::move(segment));
    }

    else if (sub_node.m_value == "via")
    {
        auto x = 0.0, y = 0.0;
        auto net = 0;
        get_2d(ss, begin(sub_node.m_branches[0].m_branches), x, y);
        get_value(ss, begin(sub_node.m_branches[4].m_branches), net);
        net_to_segments_map[net].emplace_back(
            path{point_3d{x, y, 0}, point_3d{x, y, double(layer_to_index_map.size() - 1)}});
    }
  }



  return true;
}

void kicadPcbDataBase::printComp() 
{

  std::cout << "###########COMP############" << std::endl;
  for (comp_it = name_to_component_map.begin(); comp_it != name_to_component_map.end(); ++comp_it) {
    std::cout << comp_it->first <<  " " << "====================== " << std::endl;
    auto comp = comp_it->second;
    for(size_t i = 0; i < comp.m_lines.size(); ++i) {
      std::cout << "\tstart: (" << comp.m_lines[i].m_start.m_x << "," << comp.m_lines[i].m_start.m_y << ")" << std::endl;
      std::cout << "\tend: (" << comp.m_lines[i].m_end.m_x << "," << comp.m_lines[i].m_end.m_y << ")" << std::endl;
      std::cout << "\twidth: " << comp.m_lines[i].m_width << std::endl;
    }
    for (pad_it = comp.m_pin_map.begin(); pad_it != comp.m_pin_map.end(); ++pad_it) {
      auto pad = pad_it->second;
      std::cout << "\tpad: " << pad.m_name << " (" << pad.m_x << "," << pad.m_y << ") " << pad.m_angle << std::endl;
      std::cout << "\t\tsize: " << (int)pad.m_form << " (" << pad.m_size.m_x << "," << pad.m_size.m_y << ")" << std::endl;
    }
  }
}

void kicadPcbDataBase::printInst()
{

  std::cout << "###########INST############" << std::endl;
  for (inst_it = name_to_instance_map.begin(); inst_it != name_to_instance_map.end(); ++inst_it) {
    auto inst = inst_it->second;
    std::cout << inst_it->first <<  " " << inst.m_comp << "====================== " << std::endl;
    for (pin_it = inst.m_pin_net_map.begin(); pin_it != inst.m_pin_net_map.end(); ++pin_it) {
      std::cout << "\tpin: " << pin_it->first << " net: " << pin_it->second << std::endl;
    }
  }
}


void kicadPcbDataBase::printNet()
{
  std::cout << "#################NET###############" << std::endl;
  for (net_it = name_to_net_map.begin(); net_it != name_to_net_map.end(); ++net_it) {
    auto net = net_it->second;
    std::cout << net_it->first << " " << net.getClearance() << " " << net.pins.size() << std::endl;
    for (size_t i = 0; i < net.pins.size(); ++i) {
      auto pin = net.pins[i];
      std::cout << "\tinst name: " << pin.m_instance_name << " pin name: " << pin.m_name << " comp name: " << pin.m_comp_name << std::endl;

    }
  }
}

// Warning! Doesn't count component rotation
void kicadPcbDataBase::printPcbRouterInfo()
{
  std::cout << std::endl << "#################Routing Input###############" << std::endl;
  for (net_it = name_to_net_map.begin(); net_it != name_to_net_map.end(); ++net_it) {
    auto &&net = net_it->second;
    std::cout << net_it->first << ", clearance: " << net.getClearance() << ", size: " << net.pins.size() << ", netId: " << net.getId() << std::endl;
    for (size_t i = 0; i < net.pins.size(); ++i) {
      auto &&pin = net.pins[i];
      auto &&inst = name_to_instance_map[pin.m_instance_name];
      std::string compName = inst.m_comp;
      auto &&comp = name_to_component_map[compName];
      auto &&pad = comp.m_pin_map[pin.m_name];
      point_2d pinLocation;

      getPinPosition(pin.m_instance_name, pin.m_name, &pinLocation);

      std::cout << "\tinst name: " << pin.m_instance_name << ", " << inst.m_name << " (" << inst.m_x << " " << inst.m_y << ")" << ", Rot: " << inst.m_angle
                << " pin name: " << pin.m_name << " Relative:(" << pad.m_x << " " << pad.m_y << ")" << ", Rot: " << pad.m_angle << ", Absolute Pin Loc:(" << pinLocation.m_x << ", " << pinLocation.m_y << ")"
                << " comp name: " << pin.m_comp_name << ", " << comp.m_name << std::endl;

    }
  }
}

bool kicadPcbDataBase::getPcbRouterInfo(std::vector<std::set<std::pair<double, double>>> *routerInfo)
{
  int numNet = name_to_net_map.size();
  // netId = 0 is default 
  routerInfo->resize(numNet+1);
  //int netCounter = 0;

  for (net_it = name_to_net_map.begin(); net_it != name_to_net_map.end(); ++net_it) {
    auto &&net = net_it->second;
    assert(net.getId()<=numNet);

    for (size_t i = 0; i < net.pins.size(); ++i) {
      auto &&pin = net.pins[i];
      point_2d pinLocation;
      getPinPosition(pin.m_instance_name, pin.m_name, &pinLocation);
      routerInfo->at(net.getId()).insert(std::pair<double, double>(pinLocation.m_x, pinLocation.m_y));
    }
  }
  return true;
}

bool kicadPcbDataBase::getNumOfInst(int* num)
{
  if(!num) return false;
  *num = (int)name_to_instance_map.size();
  return true;
}


bool kicadPcbDataBase::getInst(int &id, instance* inst)
{
  if(!inst) return false;
  inst_it = name_to_instance_map.begin();
  inst_it = next(inst_it, id);
  *inst = inst_it->second;
  return true;
}

//TODO: consider pin shape;
bool kicadPcbDataBase::getPinPosition(std::string & instName, std::string & pinName, point_2d *pos)
{
  if(!pos) return false;

  auto &&inst = name_to_instance_map[instName];
  std::string compName = inst.m_comp;
  auto &&comp = name_to_component_map[compName];
  auto &&pad = comp.m_pin_map[pinName];
  double padX = pad.m_x, padY = pad.m_y, padAngle = pad.m_angle;
  auto instAngle = inst.m_angle * (-M_PI / 180.0);

  auto s = sin((float)instAngle);
  auto c = cos((float)instAngle);
  pos->m_x = double((c*padX - s*padY) + inst.m_x);
  pos->m_y = double((s*padX + c*padY) + inst.m_y);

  return true;
}

bool kicadPcbDataBase::getInstBBox(std::string &instName, point_2d *bBox)
{
  if(!bBox) return false;
  auto &&inst = name_to_instance_map[instName];
  auto &&comp = name_to_component_map[inst.m_comp];
  auto angle = inst.m_angle;
  auto minx = double(1000000.0);
  auto miny = double(1000000.0);
  auto maxx = double(-1000000.0);
  auto maxy = double(-1000000.0);
  for (size_t i = 0; i < comp.m_lines.size(); ++i) {
    auto start = comp.m_lines[i].m_start;
    auto end = comp.m_lines[i].m_end;
    auto width = comp.m_lines[i].m_width/2;
    minx = std::min(start.m_x-width, minx);
    maxx = std::max(start.m_x+width, maxx);
    minx = std::min(end.m_x-width, minx);
    maxx = std::max(end.m_x+width, maxx);

    miny = std::min(start.m_y-width, miny);
    maxy = std::max(start.m_y+width, maxy);
    miny = std::min(end.m_y-width, miny);
    maxy = std::max(end.m_y+width, maxy);

  }

  for (size_t i = 0; i < comp.m_circles.size(); ++i) {
    auto center = comp.m_circles[i].m_center;
    auto end = comp.m_circles[i].m_end;
    auto width = comp.m_circles[i].m_width/2;
    minx = std::min(center.m_x-end.m_x-width, minx);
    maxx = std::max(center.m_x+width+end.m_x, maxx);

    miny = std::min(center.m_y-end.m_y-width, miny);
    maxy = std::max(center.m_y+end.m_y+width, maxy);
  }

  for (size_t i = 0; i < comp.m_polys.size(); ++i) {
    for (size_t j = 0; j < comp.m_polys[i].m_shape.size(); ++j) {
      auto point = comp.m_polys[i].m_shape[j];
      auto width = comp.m_polys[i].m_width/2;
      minx = std::min(point.m_x-width, minx);
      maxx = std::max(point.m_x+width, maxx);
      miny = std::min(point.m_y-width, miny);
      maxy = std::max(point.m_y+width, maxy);
    }
  }


  for (size_t i = 0; i < comp.m_arcs.size(); ++i) {
    auto start = comp.m_arcs[i].m_start;
    auto end = comp.m_arcs[i].m_end;
    auto width = comp.m_arcs[i].m_width/2;
    minx = std::min(start.m_x-width, minx);
    maxx = std::max(start.m_x+width, maxx);
    minx = std::min(end.m_x-width, minx);
    maxx = std::max(end.m_x+width, maxx);

    miny = std::min(start.m_y-width, miny);
    maxy = std::max(start.m_y+width, maxy);
    miny = std::min(end.m_y-width, miny);
    maxy = std::max(end.m_y+width, maxy);
  }

  for (pad_it = comp.m_pin_map.begin(); pad_it != comp.m_pin_map.end(); ++pad_it) {
    auto &&pad = pad_it->second;
    auto pad_x = pad.m_x;
    auto pad_y = pad.m_y;
    auto size = pad.m_size;
    auto pad_angle = pad.m_angle;

    if(pad_angle == 0 || pad_angle == 180) {
      minx = std::min(pad_x-size.m_x/2, minx);
      maxx = std::max(pad_x+size.m_x/2, maxx);

      miny = std::min(pad_y-size.m_y/2, miny);
      maxy = std::max(pad_y+size.m_y/2, maxy);
    }

    if(pad_angle == 90 || pad_angle == 270) {
      miny = std::min(pad_x-size.m_x/2, miny);
      maxy = std::max(pad_x+size.m_x/2, maxy);

      minx = std::min(pad_y-size.m_y/2, minx);
      maxx = std::max(pad_y+size.m_y/2, maxx);
    }
  }

  double width = abs(maxx-minx);
  double height = abs(maxy-miny);
  if(angle == 90 || angle == 270) {
    bBox->m_x = height;
    bBox->m_y = width;
  } else {
    bBox->m_x = width;
    bBox->m_y = height;
  }

  return true;
}

bool kicadPcbDataBase::getPad(std::string &compName, std::string &padName, padstack *pad)
{
  if(!pad) return false;
  auto &&comp = name_to_component_map[compName];
  *pad = comp.m_pin_map[padName];
  return true;
}
