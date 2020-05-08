#ifndef PCBROUTER_BOOST_H
#define PCBROUTER_BOOST_H

#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/indexable.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/mpl/string.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using point_int_t = bg::model::point<int, 2, bg::cs::cartesian>;
using point_double_t = bg::model::point<double, 2, bg::cs::cartesian>;
using box_int_t = bg::model::box<point_int_t>;
using polygon_int_t = bg::model::polygon<point_int_t>;
using polygon_double_t = bg::model::polygon<point_double_t>;
using segment_double_t = bg::model::segment<point_double_t>;
using linestring_double_t = bg::model::linestring<point_double_t>;
using box_double_t = bg::model::box<point_double_t>;

#endif