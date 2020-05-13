#include "PostProcessing.h"

void PostProcessing::removeAcuteAngleBetweenGridPinsAndPaths(const vector<GridPin> &gridPins, vector<GridPath> &gridPaths, const double gridWireWidth) {
    std::cout << "==========Start of all gridPaths Segments==========" << std::endl;
    for (const auto &gPath : gridPaths) {
        gPath.printSegments();
    }
    std::cout << "==========End of all gridPaths Segments==========" << std::endl;

    unordered_set<int> processedFromHead;
    unordered_set<int> processedFromTail;
    this->mGridWireWidth = gridWireWidth;

    for (const auto &gPin : gridPins) {
        std::cout << "Pin's Polygon: " << boost::geometry::wkt(gPin.getPinPolygon()) << std::endl;
        std::cout << "Pin's Expanded Polygon: " << boost::geometry::wkt(gPin.getExpandedPinPolygon()) << std::endl;
        std::cout << "Pin's LL: " << gPin.getPinLL() << ", UR: " << gPin.getPinUR() << std::endl;
        std::cout << "Pin's Contracted LL: " << gPin.getContractedPinLL() << ", Contracted UR: " << gPin.getContractedPinUR() << std::endl;
        std::cout << "Pin's Expanded LL: " << gPin.getExpandedPinLL() << ", Expanded UR: " << gPin.getExpandedPinUR() << std::endl;

        for (int i = 0; i < gridPaths.size(); ++i) {
            auto &gPath = gridPaths.at(i);
            if (gPath.getSegments().size() < 2) {
                continue;
            }
            auto &segs = gPath.setSegments();

            // Make sure process only once
            if (processedFromHead.find(i) == processedFromHead.end()) {
                // See if the starting segment is within/crosses the polygon
                // 1. its a segment but not via
                // 2. start point is within pin polygon (return false when point on the polygon boundary)
                // 3. (Skip) starting segments intersects pin polygon <= redundant /*bg::intersects(bgFirstLs, gPin.getExpandedPinPolygon()) &&*/
                auto firstPtIte = segs.begin();
                auto secondPtIte = ++segs.begin();

                if (firstPtIte->z() == secondPtIte->z() &&
                    gPin.isConnectedToPin(*firstPtIte)
                    /*&& bg::within(point_double_t(firstPtIte->x(), firstPtIte->y()), gPin.getExpandedPinPolygon())*/) {
                    // 3-Steps
                    // 1. Identify a pad entry violation happens to the PinPolygon
                    // 2. if is viloation, find a intersection point to the ExpandedPinPolygon
                    // 3. Based on intersection point and pin center, obtain the updated segments

                    // Jump to the segment that crosses the polygon outline
                    for (; secondPtIte != segs.end() && firstPtIte != segs.end(); ++firstPtIte, ++secondPtIte) {
                        linestring_double_t bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};
                        // TODO: Change crosses to two points within or not to do early break
                        // if two points are all outside of the pin polygon
                        if (bg::crosses(bgFirstLs, gPin.getPinPolygon()) &&
                            firstPtIte->z() == secondPtIte->z() &&
                            gPin.isPinLayer(firstPtIte->z()) &&
                            isAcuteAngleBetweenPadAndSegment(gPin, *firstPtIte, *secondPtIte, bgFirstLs)) {
                            // Has an acute angle that need to be fixed
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                // Need to find intersection at the expanded polygon
                                for (; secondPtIte != segs.end(); ++firstPtIte, ++secondPtIte) {
                                    bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};
                                    if (bg::crosses(bgFirstLs, gPin.getExpandedPinPolygon()) &&
                                        firstPtIte->z() == secondPtIte->z() &&
                                        gPin.isPinLayer(firstPtIte->z())) {
                                        break;
                                    }
                                }
                            }
                            // Calculate the update segments
                            list<Location> updatedSegments;
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                expCirclePadFindIntersectPtAndGetNewSegments(gPin, bgFirstLs, *firstPtIte, *secondPtIte, updatedSegments);
                            } else if (gPin.getPinShape() == GridPin::PinShape::RECT) {
                                rectPadFindIntersectPtAndGetNewSegments(gPin, bgFirstLs, *firstPtIte, *secondPtIte, updatedSegments);

                                if (updatedSegments.empty()) {
                                    // Need to find intersection at the expanded polygon
                                    for (; secondPtIte != segs.end(); ++firstPtIte, ++secondPtIte) {
                                        bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};
                                        if (bg::crosses(bgFirstLs, gPin.getExpandedPinPolygon()) &&
                                            firstPtIte->z() == secondPtIte->z() &&
                                            gPin.isPinLayer(firstPtIte->z())) {
                                            expRectPadFindIntersectPtAndGetNewSegments(gPin, bgFirstLs, *firstPtIte, *secondPtIte, updatedSegments);
                                            break;
                                        }
                                    }
                                }
                            }

                            // Remove acute angle segments
                            segs.erase(segs.begin(), secondPtIte);
                            // Update the segments without acute angle
                            while (!updatedSegments.empty()) {
                                segs.emplace_front(updatedSegments.front());
                                updatedSegments.pop_front();
                            }
                            processedFromHead.insert(i);
                            break;
                        }
                    }
                }
            }

            /*
            // Make sure process only once
            // Fix the lost connection due to fixing the acute angle violation
            if (processedFromHead.find(i) == processedFromHead.end()) {
                auto firstPtIte = segs.begin();
                auto secondPtIte = ++segs.begin();
                if (firstPtIte->z() == secondPtIte->z() &&
                    !gPin.isConnectedToPin(*firstPtIte) &&
                    gPin.isPinLayer(firstPtIte->z()) &&
                    bg::within(point_double_t(firstPtIte->x(), firstPtIte->y()), gPin.getExpandedPinPolygon())) {
                    // Jump to the segment that crosses the polygon outline
                    for (; secondPtIte != segs.end() && firstPtIte != segs.end(); ++firstPtIte, ++secondPtIte) {
                        linestring_double_t bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};

                        // Encounter an via break;
                        if (firstPtIte->z() != secondPtIte->z()) {
                            break;
                        }

                        if ((bg::crosses(bgFirstLs, gPin.getPinPolygon()) || bg::crosses(bgFirstLs, gPin.getExpandedPinPolygon())) &&
                            gPin.isPinLayer(firstPtIte->z())) {
                            // Because the segment it connected might be fixed, so needs fixed as well
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                // Need to find intersection at the expanded polygon
                                for (; secondPtIte != segs.end(); ++firstPtIte, ++secondPtIte) {
                                    bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};
                                    if (bg::crosses(bgFirstLs, gPin.getExpandedPinPolygon()) &&
                                        firstPtIte->z() == secondPtIte->z() &&
                                        gPin.isPinLayer(firstPtIte->z())) {
                                        break;
                                    }
                                }
                            }
                            // Calculate the update segments
                            list<Location> updatedSegments;
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                expCirclePadFindIntersectPtAndGetNewSegments(gPin, bgFirstLs, *firstPtIte, *secondPtIte, updatedSegments);
                            } else if (gPin.getPinShape() == GridPin::PinShape::RECT) {
                                rectPadFindIntersectPtAndGetNewSegments(gPin, bgFirstLs, *firstPtIte, *secondPtIte, updatedSegments);

                                if (updatedSegments.empty()) {
                                    // Need to find intersection at the expanded polygon
                                    for (; secondPtIte != segs.end(); ++firstPtIte, ++secondPtIte) {
                                        bgFirstLs = linestring_double_t{point_double_t(firstPtIte->x(), firstPtIte->y()), point_double_t(secondPtIte->x(), secondPtIte->y())};
                                        if (bg::crosses(bgFirstLs, gPin.getExpandedPinPolygon()) &&
                                            firstPtIte->z() == secondPtIte->z() &&
                                            gPin.isPinLayer(firstPtIte->z())) {
                                            expRectPadFindIntersectPtAndGetNewSegments(gPin, bgFirstLs, *firstPtIte, *secondPtIte, updatedSegments);
                                            break;
                                        }
                                    }
                                }
                            }

                            // Remove acute angle segments
                            segs.erase(segs.begin(), secondPtIte);
                            // Update the segments without acute angle
                            while (!updatedSegments.empty()) {
                                segs.emplace_front(updatedSegments.front());
                                updatedSegments.pop_front();
                            }
                            processedFromHead.insert(i);
                            break;
                        }
                    }
                }
            }
            //*/

            if (processedFromTail.find(i) == processedFromTail.end()) {
                auto lastPtIte = prev(segs.end());
                auto secondLastPtIte = prev(lastPtIte);

                if (lastPtIte->z() == secondLastPtIte->z() &&
                    gPin.isConnectedToPin(*lastPtIte) /*&&
                    bg::within(point_double_t(lastPtIte->x(), lastPtIte->y()), gPin.getExpandedPinPolygon())*/
                ) {
                    // Jump to the segment that crosses the polygon outline
                    for (; lastPtIte != segs.begin(); --lastPtIte, --secondLastPtIte) {
                        linestring_double_t bgLastLs{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};
                        if (bg::crosses(bgLastLs, gPin.getPinPolygon()) &&
                            lastPtIte->z() == secondLastPtIte->z() &&
                            gPin.isPinLayer(lastPtIte->z()) &&
                            isAcuteAngleBetweenPadAndSegment(gPin, *lastPtIte, *secondLastPtIte, bgLastLs)) {
                            // Has an acute angle that need to be fixed
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                // Need to find intersection at the expanded polygon
                                for (; lastPtIte != segs.begin(); --lastPtIte, --secondLastPtIte) {
                                    bgLastLs = linestring_double_t{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};
                                    if (bg::crosses(bgLastLs, gPin.getExpandedPinPolygon()) &&
                                        lastPtIte->z() == secondLastPtIte->z() &&
                                        gPin.isPinLayer(lastPtIte->z())) {
                                        break;
                                    }
                                }
                            }

                            // Calculate the update segments
                            list<Location> updatedSegments;
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                expCirclePadFindIntersectPtAndGetNewSegments(gPin, bgLastLs, *lastPtIte, *secondLastPtIte, updatedSegments);
                            } else if (gPin.getPinShape() == GridPin::PinShape::RECT) {
                                rectPadFindIntersectPtAndGetNewSegments(gPin, bgLastLs, *lastPtIte, *secondLastPtIte, updatedSegments);

                                if (updatedSegments.empty()) {
                                    // Need to find intersection at the expanded polygon
                                    for (; lastPtIte != segs.begin(); --lastPtIte, --secondLastPtIte) {
                                        bgLastLs = linestring_double_t{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};
                                        if (bg::crosses(bgLastLs, gPin.getExpandedPinPolygon()) &&
                                            lastPtIte->z() == secondLastPtIte->z() &&
                                            gPin.isPinLayer(lastPtIte->z())) {
                                            expRectPadFindIntersectPtAndGetNewSegments(gPin, bgLastLs, *lastPtIte, *secondLastPtIte, updatedSegments);
                                            break;
                                        }
                                    }
                                }
                            }

                            // Remove acute angle segments
                            segs.erase(lastPtIte, segs.end());
                            // Update the segments without acute angle
                            while (!updatedSegments.empty()) {
                                segs.emplace_back(updatedSegments.front());
                                updatedSegments.pop_front();
                            }
                            processedFromTail.insert(i);
                            break;
                        }
                    }
                }
            }

            //*
            // Fix the lost connection due to fixing the acute angle violation
            if (processedFromTail.find(i) == processedFromTail.end()) {
                auto lastPtIte = prev(segs.end());
                auto secondLastPtIte = prev(lastPtIte);

                if (lastPtIte->z() == secondLastPtIte->z() &&
                    !gPin.isConnectedToPin(*lastPtIte) &&
                    gPin.isPinLayer(lastPtIte->z()) &&
                    bg::within(point_double_t(lastPtIte->x(), lastPtIte->y()), gPin.getExpandedPinPolygon())) {
                    // Jump to the segment that crosses the polygon outline
                    for (; lastPtIte != segs.begin(); --lastPtIte, --secondLastPtIte) {
                        linestring_double_t bgLastLs{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};

                        // Encounter an via break;
                        if (lastPtIte->z() != secondLastPtIte->z()) {
                            break;
                        }

                        if ((bg::crosses(bgLastLs, gPin.getPinPolygon()) || bg::crosses(bgLastLs, gPin.getExpandedPinPolygon())) &&
                            gPin.isPinLayer(lastPtIte->z())) {
                            // Because the segment it connected might be fixed, so needs fixed as well
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                // Need to find intersection at the expanded polygon
                                for (; lastPtIte != segs.begin(); --lastPtIte, --secondLastPtIte) {
                                    bgLastLs = linestring_double_t{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};
                                    if (bg::crosses(bgLastLs, gPin.getExpandedPinPolygon()) &&
                                        lastPtIte->z() == secondLastPtIte->z() &&
                                        gPin.isPinLayer(lastPtIte->z())) {
                                        break;
                                    }
                                }
                            }

                            // Calculate the update segments
                            list<Location> updatedSegments;
                            if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
                                expCirclePadFindIntersectPtAndGetNewSegments(gPin, bgLastLs, *lastPtIte, *secondLastPtIte, updatedSegments);
                            } else if (gPin.getPinShape() == GridPin::PinShape::RECT) {
                                rectPadFindIntersectPtAndGetNewSegments(gPin, bgLastLs, *lastPtIte, *secondLastPtIte, updatedSegments);

                                if (updatedSegments.empty()) {
                                    // Need to find intersection at the expanded polygon
                                    for (; lastPtIte != segs.begin(); --lastPtIte, --secondLastPtIte) {
                                        bgLastLs = linestring_double_t{point_double_t(lastPtIte->x(), lastPtIte->y()), point_double_t(secondLastPtIte->x(), secondLastPtIte->y())};
                                        if (bg::crosses(bgLastLs, gPin.getExpandedPinPolygon()) &&
                                            lastPtIte->z() == secondLastPtIte->z() &&
                                            gPin.isPinLayer(lastPtIte->z())) {
                                            expRectPadFindIntersectPtAndGetNewSegments(gPin, bgLastLs, *lastPtIte, *secondLastPtIte, updatedSegments);
                                            break;
                                        }
                                    }
                                }
                            }

                            if (!updatedSegments.empty()) {
                                // Remove acute angle segments
                                segs.erase(lastPtIte, segs.end());
                                // Update the segments without acute angle
                                while (!updatedSegments.empty()) {
                                    segs.emplace_back(updatedSegments.front());
                                    updatedSegments.pop_front();
                                }
                                processedFromTail.insert(i);
                            }
                            break;
                        }
                    }
                }
            }
            //*/
        }
    }
}

bool PostProcessing::isIntersectionPointOnTheBoxCorner(const point_double_t &point, const polygon_double_t &poly, const double wireWidth) {
    //See if minimum distance to the corner is within half wire width
    double minDis = numeric_limits<double>::max();
    for (auto it = boost::begin(bg::exterior_ring(poly)); it != boost::end(bg::exterior_ring(poly)); ++it) {
        minDis = min(minDis, bg::distance(point, *it));
    }
    return minDis < (wireWidth / 2.0) * GlobalParam::gSqrt2;
}

bool PostProcessing::isAcuteAngleBetweenPadAndSegment(const GridPin &gPin, const Location &inPt, const Location &outPt, const linestring_double_t &bgLs) {
    // Several cases that needs to check if is a drc
    if (gPin.getPinShape() == GridPin::PinShape::RECT) {
        // Orthogonal segments
        if ((inPt.x() == outPt.x() && inPt.y() != outPt.y()) ||
            (inPt.x() != outPt.x() && inPt.y() == outPt.y())) {
            return false;
        }
        // Diagonal segments
        if (inPt.x() != outPt.x() && inPt.y() != outPt.y()) {
            std::vector<point_double_t> intersectPts;
            bg::intersection(bgLs, gPin.getPinPolygon(), intersectPts);

            if (!intersectPts.empty() && !isIntersectionPointOnTheBoxCorner(intersectPts.front(), gPin.getPinPolygon(), this->mGridWireWidth)) {
                std::cout << __FUNCTION__ << "(): Is Acute Angle" << std::endl;
                return true;
            } else {
                return false;
            }
        }
    } else if (gPin.getPinShape() == GridPin::PinShape::CIRCLE) {
        if (inPt.x() == gPin.getPinCenter().x() && inPt.y() == gPin.getPinCenter().y()) {
            return false;
        } else {
            return true;
        }
    }
    return true;
}

void PostProcessing::rectPadFindIntersectPtAndGetNewSegments(const GridPin &gPin, const linestring_double_t &bgSeg,
                                                             const Location &inPt, const Location &outPt, list<Location> &ret) {
    // Get the intersection point by bg::intersection
    std::vector<point_double_t> intersectPts;
    bg::intersection(bgSeg, gPin.getPinPolygon(), intersectPts);
    for (const auto &pt : intersectPts) {
        std::cout << "Rect: Seg: (" << bg::get<0>(bgSeg.front()) << ", " << bg::get<1>(bgSeg.front())
                  << "), (" << bg::get<0>(bgSeg.back()) << ", " << bg::get<1>(bgSeg.back())
                  << "), Intersetion Pt: " << bg::get<0>(pt) << ", " << bg::get<1>(pt) << std::endl;
    }

    if (intersectPts.empty()) {
        return;
    }
    if (intersectPts.size() > 1) {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::WARNING) {
            std::cout << __FUNCTION__ << "(): have multiple intersection points. Will use the first point to update segments." << std::endl;
        }
    }

    int intersectX = round(bg::get<0>(intersectPts.front()));
    int intersectY = round(bg::get<1>(intersectPts.front()));

    if (intersectX == gPin.getPinLL().x() || intersectX == gPin.getExpandedPinUR().x()) {
        // Intersection pt at left/right
        // Two horizontal lines
        linestring_double_t bgTopLs = linestring_double_t{point_double_t(gPin.getExpandedPinLL().x(), gPin.getContractedPinUR().y()), point_double_t(gPin.getExpandedPinUR().x(), gPin.getContractedPinUR().y())};
        linestring_double_t bgBottomLs = linestring_double_t{point_double_t(gPin.getExpandedPinLL().x(), gPin.getContractedPinLL().y()), point_double_t(gPin.getExpandedPinUR().x(), gPin.getContractedPinLL().y())};
        std::vector<point_double_t> intersectToTop;
        bg::intersection(bgSeg, bgTopLs, intersectToTop);
        std::vector<point_double_t> intersectToBottom;
        bg::intersection(bgSeg, bgBottomLs, intersectToBottom);

        if (!intersectToTop.empty()) {
            ret.emplace_back(Location{(int)round(bg::get<0>(intersectToTop.front())), (int)round(bg::get<1>(intersectToTop.front())), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinUR().y(), inPt.z()});
            // Pin Center
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getPinCenter().y(), inPt.z()});
        } else if (!intersectToBottom.empty()) {
            ret.emplace_back(Location{(int)round(bg::get<0>(intersectToBottom.front())), (int)round(bg::get<1>(intersectToBottom.front())), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinLL().y(), inPt.z()});
            // Pin Center
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getPinCenter().y(), inPt.z()});
        }
    } else if (intersectY == gPin.getExpandedPinLL().y() || intersectY == gPin.getExpandedPinUR().y()) {
        //Intersection pt at bottom/top
        // Two vertical lines
        linestring_double_t bgLeftLs = linestring_double_t{point_double_t(gPin.getContractedPinLL().x(), gPin.getExpandedPinLL().y()), point_double_t(gPin.getContractedPinLL().x(), gPin.getExpandedPinUR().y())};
        linestring_double_t bgRightLs = linestring_double_t{point_double_t(gPin.getContractedPinUR().x(), gPin.getExpandedPinLL().y()), point_double_t(gPin.getContractedPinUR().x(), gPin.getExpandedPinUR().y())};
        std::vector<point_double_t> intersectToLeft;
        bg::intersection(bgSeg, bgLeftLs, intersectToLeft);
        std::vector<point_double_t> intersectToRight;
        bg::intersection(bgSeg, bgRightLs, intersectToRight);

        if (!intersectToLeft.empty()) {
            ret.emplace_back(Location{(int)round(bg::get<0>(intersectToLeft.front())), (int)round(bg::get<1>(intersectToLeft.front())), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinLL().x(), gPin.getPinCenter().y(), inPt.z()});
            // Pin Center
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getPinCenter().y(), inPt.z()});
        } else if (!intersectToRight.empty()) {
            ret.emplace_back(Location{(int)round(bg::get<0>(intersectToRight.front())), (int)round(bg::get<1>(intersectToRight.front())), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinUR().x(), gPin.getPinCenter().y(), inPt.z()});
            // Pin Center
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getPinCenter().y(), inPt.z()});
        }
    }
}

void PostProcessing::expRectPadFindIntersectPtAndGetNewSegments(const GridPin &gPin, const linestring_double_t &bgSeg,
                                                                const Location &inPt, const Location &outPt, list<Location> &ret) {
    // Get the intersection point by bg::intersection
    std::vector<point_double_t> intersectPts;
    bg::intersection(bgSeg, gPin.getExpandedPinPolygon(), intersectPts);
    for (const auto &pt : intersectPts) {
        std::cout << "ExpRect: Seg: (" << bg::get<0>(bgSeg.front()) << ", " << bg::get<1>(bgSeg.front())
                  << "), (" << bg::get<0>(bgSeg.back()) << ", " << bg::get<1>(bgSeg.back())
                  << "), Intersetion Pt: " << bg::get<0>(pt) << ", " << bg::get<1>(pt) << std::endl;
    }

    if (intersectPts.empty()) {
        return;
    }
    if (intersectPts.size() > 1) {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::WARNING) {
            std::cout << __FUNCTION__ << "(): have multiple intersection points. Will use the first point to update segments." << std::endl;
        }
    }

    int intersectX = round(bg::get<0>(intersectPts.front()));
    int intersectY = round(bg::get<1>(intersectPts.front()));

    // Intersection pt
    ret.emplace_back(Location{intersectX, intersectY, inPt.z()});

    if (intersectX == gPin.getExpandedPinLL().x() &&
        intersectY != gPin.getPinCenter().y()) {
        //Intersection pt at left and not aligned to center
        if (intersectY < gPin.getContractedPinLL().y()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{gPin.getContractedPinLL().x() + (intersectY - gPin.getContractedPinLL().y()), intersectY, inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinLL().x() + (gPin.getPinCenter().y() - gPin.getContractedPinLL().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinLL().x(), gPin.getContractedPinLL().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinLL().y(), inPt.z()});
        } else if (intersectY > gPin.getContractedPinUR().y()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{gPin.getContractedPinLL().x() - (intersectY - gPin.getContractedPinUR().y()), intersectY, inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinLL().x() - (gPin.getPinCenter().y() - gPin.getContractedPinUR().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinLL().x(), gPin.getContractedPinUR().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinUR().y(), inPt.z()});
        } else {
            // 90-degree turn
            ret.emplace_back(Location{gPin.getPinCenter().x(), intersectY, inPt.z()});
        }
    } else if (intersectY == gPin.getExpandedPinLL().y() &&
               intersectX != gPin.getPinCenter().x()) {
        //Intersection pt at bottom and not aligned to center
        if (intersectX < gPin.getContractedPinLL().x()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{intersectX, gPin.getContractedPinLL().y() + (intersectX - gPin.getContractedPinLL().x()), inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinLL().x() + (gPin.getPinCenter().y() - gPin.getContractedPinLL().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinLL().x(), gPin.getContractedPinLL().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinLL().y(), inPt.z()});
        } else if (intersectX > gPin.getContractedPinUR().x()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{intersectX, gPin.getContractedPinLL().y() - (intersectX - gPin.getContractedPinUR().x()), inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinUR().x() - (gPin.getPinCenter().y() - gPin.getContractedPinLL().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinUR().x(), gPin.getContractedPinLL().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinLL().y(), inPt.z()});
        } else {
            // 90-degree turn
            ret.emplace_back(Location{intersectX, gPin.getPinCenter().y(), inPt.z()});
        }
    } else if (intersectX == gPin.getExpandedPinUR().x() &&
               intersectY != gPin.getPinCenter().y()) {
        //Intersection pt at right and not aligned to center
        if (intersectY < gPin.getContractedPinLL().y()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{gPin.getContractedPinUR().x() - (intersectY - gPin.getContractedPinLL().y()), intersectY, inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinUR().x() - (gPin.getPinCenter().y() - gPin.getContractedPinLL().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinUR().x(), gPin.getContractedPinLL().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinLL().y(), inPt.z()});
        } else if (intersectY > gPin.getContractedPinUR().y()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{gPin.getContractedPinUR().x() + (intersectY - gPin.getContractedPinUR().y()), intersectY, inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinUR().x() + (gPin.getPinCenter().y() - gPin.getContractedPinUR().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinUR().x(), gPin.getContractedPinUR().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinUR().y(), inPt.z()});
        } else {
            // 90-degree turn
            ret.emplace_back(Location{gPin.getPinCenter().x(), intersectY, inPt.z()});
        }
    } else if (intersectY == gPin.getExpandedPinUR().y() &&
               intersectX != gPin.getPinCenter().x()) {
        //Intersection pt at top and not aligned to center => make a 90-degree turn
        if (intersectX < gPin.getContractedPinLL().x()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{intersectX, gPin.getContractedPinUR().y() - (intersectX - gPin.getContractedPinLL().x()), inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinLL().x() - (gPin.getPinCenter().y() - gPin.getContractedPinUR().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinLL().x(), gPin.getContractedPinUR().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinUR().y(), inPt.z()});
        } else if (intersectX > gPin.getContractedPinUR().x()) {
            // Need to enter the pad by its corner
            ret.emplace_back(Location{intersectX, gPin.getContractedPinUR().y() + (intersectX - gPin.getContractedPinUR().x()), inPt.z()});
            // ret.emplace_back(Location{gPin.getContractedPinUR().x() + (gPin.getPinCenter().y() - gPin.getContractedPinUR().y()), gPin.getPinCenter().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getContractedPinUR().x(), gPin.getContractedPinUR().y(), inPt.z()});
            ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getContractedPinUR().y(), inPt.z()});
        } else {
            // 90-degree turn
            ret.emplace_back(Location{intersectX, gPin.getPinCenter().y(), inPt.z()});
        }
    }
    // Pin Center
    ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getPinCenter().y(), inPt.z()});
}

void PostProcessing::expCirclePadFindIntersectPtAndGetNewSegments(const GridPin &gPin, const linestring_double_t &bgSeg,
                                                                  const Location &inPt, const Location &outPt, list<Location> &ret) {
    // Get the intersection point by bg::intersection
    std::vector<point_double_t> intersectPts;
    bg::intersection(bgSeg, gPin.getExpandedPinPolygon(), intersectPts);
    for (const auto &pt : intersectPts) {
        std::cout << "Seg: (" << bg::get<0>(bgSeg.front()) << ", " << bg::get<1>(bgSeg.front())
                  << "), (" << bg::get<0>(bgSeg.back()) << ", " << bg::get<1>(bgSeg.back())
                  << "), Intersetion Pt: " << bg::get<0>(pt) << ", " << bg::get<1>(pt) << std::endl;
    }

    if (intersectPts.empty()) {
        return;
    }
    if (intersectPts.size() > 1) {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::WARNING) {
            std::cout << __FUNCTION__ << "(): have multiple intersection points. Will use the first point to update segments." << std::endl;
        }
    }

    // Any angle segment from intersection point to the pin center
    ret.emplace_back(Location{(int)round(bg::get<0>(intersectPts.front())), (int)round(bg::get<1>(intersectPts.front())), inPt.z()});
    ret.emplace_back(Location{gPin.getPinCenter().x(), gPin.getPinCenter().y(), inPt.z()});

    // if (GlobalParam::gVerboseLevel <= VerboseLevel::CRITICAL) {
    //     std::cout << __FUNCTION__ << "(): Cannot handle this kind of pin shape type." << std::endl;
    // }
}