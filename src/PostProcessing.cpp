#include "PostProcessing.h"

void PostProcessing::removeAcuteAngleBetweenGridPinsAndPaths(const vector<GridPin> gridPins, vector<GridPath> gridPaths) {
    for (const auto &gPin : gridPins) {
        std::cout << "Pin's Expanded Polygon: " << boost::geometry::wkt(gPin.getExpandedPinPolygon()) << std::endl;
        int pinCenterX = gPin.getPinWithLayers().front().x();
        int pinCenterY = gPin.getPinWithLayers().front().y();

        for (const auto &gPath : gridPaths) {
            // auto startPt = gPath.getSegments().front();
            // auto endPt = gPath.getSegments().back();

            // if (startPt.x() == pinCenterX && startPt.y() == pinCenterY) {
            // } else if (endPt.x() == pinCenterX && endPt.y() == pinCenterY) {
            // }

            if (gPath.getSegments().size() < 2) {
                continue;
            }
            auto &segs = gPath.getSegments();

            //*
            // See if the starting segment is within/crosses the polygon
            auto firstPtIte = segs.begin();
            auto secondPtIte = ++segs.begin();
            linestring_double_t bgFirstLs{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};

            if (bg::intersects(bgFirstLs, gPin.getExpandedPinPolygon())) {
                // Jump to the segment that crosses the polygon outline
                for (; secondPtIte != segs.end();) {
                    if (bg::crosses(bgFirstLs, gPin.getExpandedPinPolygon())) {
                        break;
                    }
                    ++firstPtIte;
                    ++secondPtIte;
                    bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};
                }

                // Is a segment and on the same layer to the pin
                if (firstPtIte->z() == secondPtIte->z() && gPin.isPinLayer(firstPtIte->z())) {
                    // Get the intersection point by bg::intersection
                    std::vector<point_double_t> intersectPts;
                    bg::intersection(bgFirstLs, gPin.getExpandedPinPolygon(), intersectPts);
                    for (const auto &pt : intersectPts) {
                        std::cout << "Interset from head: Pt: " << bg::get<0>(pt) << ", " << bg::get<1>(pt) << std::endl;
                    }

                    // Several cases that needs to check if is a drc
                    if (gPin.getPinShape() == GridPin::PinShape::RECT) {
                        // Orthogonal segments
                        if ((firstPtIte->x() == secondPtIte->x() && firstPtIte->y() != secondPtIte->y()) ||
                            (firstPtIte->x() != secondPtIte->x() && firstPtIte->y() == secondPtIte->y())) {
                        }
                        // Diagonal segments
                        if (firstPtIte->x() != secondPtIte->x() && firstPtIte->y() != secondPtIte->y()) {
                        }
                    } else if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                    }
                }
            }

            // See if the ending segment is within/crosses the polygon
            auto lastPtIte = prev(segs.end());
            auto secondLastPtIte = prev(lastPtIte);
            linestring_double_t bgLastLs{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};

            if (bg::intersects(bgLastLs, gPin.getExpandedPinPolygon())) {
                // Jump to the segment that crosses the polygon outline
                for (; lastPtIte != segs.begin();) {
                    if (bg::crosses(bgLastLs, gPin.getExpandedPinPolygon())) {
                        break;
                    }
                    --lastPtIte;
                    --secondLastPtIte;
                    bgLastLs = linestring_double_t{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};
                }

                // Get the intersection point by bg::intersection
                std::vector<point_double_t> intersectPts;
                bg::intersection(bgLastLs, gPin.getExpandedPinPolygon(), intersectPts);

                for (const auto &pt : intersectPts) {
                    std::cout << "Interset from tail: Pt: " << bg::get<0>(pt) << ", " << bg::get<1>(pt) << std::endl;
                }
            }
            //*/

            /*
            auto pointIte = gPath.getSegments().begin();
            auto nextPointIte = ++gPath.getSegments().begin();

            for (; nextPointIte != gPath.getSegments().end();) {
                // Cares about segments only (and ignore vias)
                if (pointIte->z() == nextPointIte->z()) {
                    // See Boost's crosses definition!
                    // https://www.boost.org/doc/libs/1_73_0/libs/geometry/doc/html/geometry/reference/algorithms/crosses/crosses_2.html

                    // Hint: Use crosses for checking if segments/linestring is crossing the polygon (doesn't include touch)
                    // Hint: Use intersects for checking if segments/linestring is inside/crossing the polygon
                    linestring_double_t bgLs{point_double_t(pointIte->x(), pointIte->y()), point_double_t(nextPointIte->x(), nextPointIte->y())};
                    std::cout << "Segment: " << *pointIte << ", " << *nextPointIte << std::endl;

                    if (bg::crosses(bgLs, gPin.getExpandedPinPolygon())) {
                        std::vector<point_double_t> intersectPts;
                        bg::intersection(bgLs, gPin.getExpandedPinPolygon(), intersectPts);

                        for (const auto &pt : intersectPts) {
                            std::cout << "Pt: " << bg::get<0>(pt) << ", " << bg::get<1>(pt) << std::endl;
                        }
                    }
                }

                ++pointIte;
                ++nextPointIte;
            }
            //*/
        }
    }
}
