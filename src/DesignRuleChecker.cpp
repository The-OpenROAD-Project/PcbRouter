#include "DesignRuleChecker.h"

int DesignRuleChecker::checkAcuteAngleViolationBetweenTracesAndPads() {
    std::cout << "Starting " << __FUNCTION__ << "()..." << std::endl;
    std::cout << std::fixed << std::setprecision((int)round(std::log(mInputPrecision)));

    int numViolations = 0;

    // Iterate nets
    for (auto &net : mDb.getNets()) {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::DEBUG) {
            std::cout << "\nNet: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;
        }
        auto &pins = net.getPins();

        for (auto &pin : pins) {
            // DB elements
            auto &comp = mDb.getComponent(pin.getCompId());
            auto &inst = mDb.getInstance(pin.getInstId());
            auto &pad = comp.getPadstack(pin.getPadstackId());

            // Handle GridPin's pinPolygon, which should be expanded by clearance
            Point_2D<double> polyPadSize = pad.getSize();
            Point_2D<double> pinDbLocation;
            mDb.getPinPosition(pad, inst, &pinDbLocation);

            // Get a exact locations expanded polygon in db's coordinates
            std::vector<Point_2D<double>> exactDbLocPadPoly;
            if (pad.getPadShape() == padShape::CIRCLE || pad.getPadShape() == padShape::OVAL) {
                // WARNING!! shape_to_coords's pos can be origin only!!! Otherwise the rotate function will be wrong
                exactDbLocPadPoly = shape_to_coords(polyPadSize, point_2d{0, 0}, padShape::CIRCLE, inst.getAngle(), pad.getAngle(), pad.getRoundRectRatio(), 32);
            } else {
                exactDbLocPadPoly = shape_to_coords(polyPadSize, point_2d{0, 0}, padShape::RECT, inst.getAngle(), pad.getAngle(), pad.getRoundRectRatio(), 32);
            }

            // Shift to exact location
            for (auto &&pt : exactDbLocPadPoly) {
                pt.m_x += pinDbLocation.m_x;
                pt.m_y += pinDbLocation.m_y;
            }

            // Transform this into Boost's polygon in db's coordinates
            polygon_double_t exactLocGridPadShapePoly;
            for (const auto &pt : exactDbLocPadPoly) {
                bg::append(exactLocGridPadShapePoly.outer(), point_double_t(pt.x(), pt.y()));
            }
            bg::correct(exactLocGridPadShapePoly);
            // box_double_t pinBoundingBox;
            // bg::envelope(exactLocGridPadShapePoly, pinBoundingBox);
            // std::cout << "Pin's Polygon: " << boost::geometry::wkt(exactLocGridPadShapePoly) << std::endl;

            //Iterate all the segments
            for (auto &seg : net.getSegments()) {
                if (!isPadstackAndSegmentHaveSameLayer(inst, pad, seg)) {
                    continue;
                }

                points_2d &points = seg.getPos();
                if (points.size() < 2) {
                    std::cout << __FUNCTION__ << "(): Invalid # segment's points." << std::endl;
                    continue;
                }

                linestring_double_t bgLs{point_double_t(points[0].x(), points[0].y()), point_double_t(points[1].x(), points[1].y())};

                // std::cout << "Seg: (" << bg::get<0>(bgLs.front()) << ", " << bg::get<1>(bgLs.front())
                //           << "), (" << bg::get<0>(bgLs.back()) << ", " << bg::get<1>(bgLs.back()) << ")" << std::endl;

                if (bg::crosses(bgLs, exactLocGridPadShapePoly)) {
                    Point_2D<long long> pt1{(long long)round(points[0].x() * mInputPrecision), (long long)round(points[0].y() * mInputPrecision)};
                    Point_2D<long long> pt2{(long long)round(points[1].x() * mInputPrecision), (long long)round(points[1].y() * mInputPrecision)};
                    if (pad.getPadShape() == padShape::RECT) {
                        if (!isOrthogonalSegment(pt1, pt2)) {
                            // Get intersection point
                            std::vector<point_double_t> intersectPts;
                            bg::intersection(bgLs, exactLocGridPadShapePoly, intersectPts);

                            if (!intersectPts.empty()) {
                                if (!isIntersectionPointOnTheBoxCorner(intersectPts.front(), exactLocGridPadShapePoly, seg.getWidth(), mAcuteAngleTol)) {
                                    std::cout << __FUNCTION__ << "(): Violation between segment: ("
                                              << points[0].x() << ", " << points[0].y() << "), ("
                                              << points[1].x() << ", " << points[1].y() << "), width: "
                                              << seg.getWidth() << ", and rectangle pad center at "
                                              << pinDbLocation.x() << ", " << pinDbLocation.y() << std::endl;
                                    ++numViolations;
                                }
                            }
                        }
                    } else if (pad.getPadShape() == padShape::CIRCLE) {
                        point_double_t center{pinDbLocation.x(), pinDbLocation.y()};
                        if (!isSegmentTouchAPoint(bgLs, center, seg.getWidth())) {
                            std::cout << __FUNCTION__ << "(): Violation between segment: ("
                                      << points[0].x() << ", " << points[0].y() << "), ("
                                      << points[1].x() << ", " << points[1].y() << ") and circle pad center at "
                                      << pinDbLocation.x() << ", " << pinDbLocation.y() << std::endl;
                            ++numViolations;
                        }
                    }
                }
            }
        }
    }

    std::cout << "Total # acute angle violations: " << numViolations << std::endl;
    std::cout << "End of " << __FUNCTION__ << "()..." << std::endl;
    return numViolations;
}

int DesignRuleChecker::checkTJunctionViolation() {
    std::cout << "Starting " << __FUNCTION__ << "()..." << std::endl;
    std::cout << std::fixed << std::setprecision((int)round(std::log(mInputPrecision)));

    int numViolations = 0;
    std::vector<TJunctionPatch> patches;

    // Iterate nets
    for (auto &net : mDb.getNets()) {
        // if (net.getId() != 27) continue;

        if (GlobalParam::gVerboseLevel <= VerboseLevel::DEBUG) {
            std::cout << "\nNet: " << net.getName() << ", netId: " << net.getId() << ", netDegree: " << net.getPins().size() << "..." << std::endl;
        }

        for (int segId = 0; segId < net.getSegments().size(); ++segId) {
            for (int segId2 = segId + 1; segId2 < net.getSegments().size(); ++segId2) {
                auto &seg1 = net.getSegment(segId);
                auto &seg2 = net.getSegment(segId2);

                if (seg1.getLayer() != seg2.getLayer()) {
                    continue;
                }

                points_2d &ptsSeg1 = seg1.getPos();
                points_2d &ptsSeg2 = seg2.getPos();
                if (ptsSeg1.size() < 2 || ptsSeg2.size() < 2) {
                    std::cout << __FUNCTION__ << "(): Invalid # segment's points." << std::endl;
                    continue;
                }
                linestring_double_t bgLs1{point_double_t(ptsSeg1[0].x(), ptsSeg1[0].y()), point_double_t(ptsSeg1[1].x(), ptsSeg1[1].y())};
                linestring_double_t bgLs2{point_double_t(ptsSeg2[0].x(), ptsSeg2[0].y()), point_double_t(ptsSeg2[1].x(), ptsSeg2[1].y())};

                if (areConnectedSegments(bgLs1, bgLs2)) {
                    continue;
                }

                // Check if is T-Junction
                ++numViolations;
                double patchSize = seg1.getWidth() * 2.0;
                polygon_double_t patchPoly;

                if (bg::distance(bgLs1.back(), bgLs2) < mTJunctionEpsilon) {
                    std::cout << __FUNCTION__ << "(): Violation at point: (" << ptsSeg1[1].x() << ", " << ptsSeg1[1].y() << ")" << std::endl;
                    addTJunctionPatch(bgLs1.back(), bgLs1, bgLs2, patchSize, patchPoly);
                } else if (bg::distance(bgLs1.front(), bgLs2) < mTJunctionEpsilon) {
                    std::cout << __FUNCTION__ << "(): Violation at point: (" << ptsSeg1[0].x() << ", " << ptsSeg1[0].y() << ")" << std::endl;
                    addTJunctionPatch(bgLs1.front(), bgLs1, bgLs2, patchSize, patchPoly);
                } else if (bg::distance(bgLs2.front(), bgLs1) < mTJunctionEpsilon) {
                    std::cout << __FUNCTION__ << "(): Violation at point: (" << ptsSeg2[0].x() << ", " << ptsSeg2[0].y() << ")" << std::endl;
                    addTJunctionPatch(bgLs2.front(), bgLs2, bgLs1, patchSize, patchPoly);
                } else if (bg::distance(bgLs2.back(), bgLs1) < mTJunctionEpsilon) {
                    std::cout << __FUNCTION__ << "(): Violation at point: (" << ptsSeg2[1].x() << ", " << ptsSeg2[1].y() << ")" << std::endl;
                    addTJunctionPatch(bgLs2.back(), bgLs2, bgLs1, patchSize, patchPoly);
                } else {
                    --numViolations;
                }

                if (!bg::is_empty(patchPoly)) {
                    patches.emplace_back(TJunctionPatch{patchPoly, seg1.getLayer(), seg1.getNetId()});
                }
            }
        }
    }

    printTJunctionPatchToKiCadZone(patches);

    std::cout << "Total # T-Junction violations: " << numViolations << std::endl;
    std::cout << "End of " << __FUNCTION__ << "()..." << std::endl;
    return numViolations;
}

void DesignRuleChecker::printTJunctionPatchToKiCadZone(std::vector<TJunctionPatch> &patches) {
    // Print all the patches in KiCadPcbFormat
    for (const auto &patch : patches) {
        std::cout << "( zone ( net 0 )";
        std::cout << "( polygon ( pts ";

        // ( xy 148.9585 131.8135 )
        for (auto it = boost::begin(bg::exterior_ring(patch.poly)); it != boost::end(bg::exterior_ring(patch.poly)); ++it) {
            auto x = bg::get<0>(*it);
            auto y = bg::get<1>(*it);
            std::cout << "( xy " << x << " " << y << ") ";
        }
        std::cout << ")";
        std::cout << ")";
        std::cout << ")";
        std::cout << std::endl;
    }
}

void DesignRuleChecker::addTJunctionPatch(const point_double_t &point, const linestring_double_t &bgLs1, const linestring_double_t &bgLs2, const double size, polygon_double_t &patchPoly) {
    std::cout << "Add T-Junction Patch at: " << std::endl;
    std::cout << "Seg1: (" << bg::get<0>(bgLs1.front()) << ", " << bg::get<1>(bgLs1.front())
              << "), (" << bg::get<0>(bgLs1.back()) << ", " << bg::get<1>(bgLs1.back()) << ")" << std::endl;
    std::cout << "Seg2: (" << bg::get<0>(bgLs2.front()) << ", " << bg::get<1>(bgLs2.front())
              << "), (" << bg::get<0>(bgLs2.back()) << ", " << bg::get<1>(bgLs2.back()) << ")" << std::endl;

    std::cout << "bg::distance(bgLs1.back(), bgLs2): " << bg::distance(bgLs1.back(), bgLs2) << std::endl;
    std::cout << "bg::distance(bgLs1.front(), bgLs2): " << bg::distance(bgLs1.front(), bgLs2) << std::endl;
    std::cout << "bg::distance(bgLs2.front(), bgLs1): " << bg::distance(bgLs2.front(), bgLs1) << std::endl;
    std::cout << "bg::distance(bgLs2.back(), bgLs1): " << bg::distance(bgLs2.back(), bgLs1) << std::endl;

    // First Point
    point_2d vec{bg::get<0>(bgLs2.front()) - bg::get<0>(point), bg::get<1>(bgLs2.front()) - bg::get<1>(point)};
    double vecLen = sqrt(vec.x() * vec.x() + vec.y() * vec.y());
    point_2d unitVec{vec.x() / vecLen, vec.y() / vecLen};
    double length = (size / 2.0);
    if (length < vecLen) {
        bg::append(patchPoly.outer(), point_double_t(bg::get<0>(point) + unitVec.x() * length, bg::get<1>(point) + unitVec.y() * length));
    } else {
        bg::append(patchPoly.outer(), bgLs2.front());
    }

    // Seoncd Point
    vec = point_2d{bg::get<0>(bgLs2.back()) - bg::get<0>(point), bg::get<1>(bgLs2.back()) - bg::get<1>(point)};
    vecLen = sqrt(vec.x() * vec.x() + vec.y() * vec.y());
    unitVec = point_2d{vec.x() / vecLen, vec.y() / vecLen};
    length = (size / 2.0);
    if (length < vecLen) {
        bg::append(patchPoly.outer(), point_double_t(bg::get<0>(point) + unitVec.x() * length, bg::get<1>(point) + unitVec.y() * length));
    } else {
        bg::append(patchPoly.outer(), bgLs2.back());
    }

    // Third Point
    point_double_t point2 = bg::distance(bgLs1.front(), point) > bg::distance(bgLs1.back(), point) ? bgLs1.front() : bgLs1.back();
    vec = point_2d{bg::get<0>(point2) - bg::get<0>(point), bg::get<1>(point2) - bg::get<1>(point)};
    vecLen = sqrt(vec.x() * vec.x() + vec.y() * vec.y());
    unitVec = point_2d{vec.x() / vecLen, vec.y() / vecLen};
    length = ((size / 2.0) * sqrt(3));
    if (length < vecLen) {
        bg::append(patchPoly.outer(), point_double_t(bg::get<0>(point) + unitVec.x() * length, bg::get<1>(point) + unitVec.y() * length));
    } else {
        bg::append(patchPoly.outer(), point2);
    }

    bg::correct(patchPoly);
}

bool DesignRuleChecker::areConnectedSegments(linestring_double_t &ls1, linestring_double_t &ls2) {
    double minDis = numeric_limits<double>::max();
    minDis = min(minDis, bg::distance(ls1.front(), ls2.front()));
    minDis = min(minDis, bg::distance(ls1.front(), ls2.back()));
    minDis = min(minDis, bg::distance(ls1.back(), ls2.back()));
    minDis = min(minDis, bg::distance(ls1.back(), ls2.front()));
    return minDis < this->mEpsilon;
}

bool DesignRuleChecker::isSegmentTouchAPoint(linestring_double_t &ls, point_double_t &point, double wireWidth) {
    double minDis = numeric_limits<double>::max();
    minDis = min(minDis, bg::distance(point, ls.front()));
    minDis = min(minDis, bg::distance(point, ls.back()));
    return minDis < (wireWidth / 2.0);
}

bool DesignRuleChecker::isIntersectionPointOnTheBoxCorner(point_double_t &point, polygon_double_t &poly, double wireWidth, double tol) {
    //See if minimum distance to the corner is within half wire width
    double minDis = numeric_limits<double>::max();
    for (auto it = boost::begin(bg::exterior_ring(poly)); it != boost::end(bg::exterior_ring(poly)); ++it) {
        minDis = min(minDis, bg::distance(point, *it));
    }
    return minDis < ((wireWidth / 2.0) * GlobalParam::gSqrt2 * (1 + tol));
    // return minDis < (wireWidth / 2.0) * GlobalParam::gSqrt2;
}

bool DesignRuleChecker::isOrthogonalSegment(Point_2D<long long> &pt1, Point_2D<long long> &pt2) {
    if ((pt1.x() == pt2.x() && pt1.y() != pt2.y()) ||
        (pt1.x() != pt2.x() && pt2.y() == pt1.y())) {
        return true;
    }
    return false;
}

bool DesignRuleChecker::isOrthogonalSegment(points_2d &points) {
    Point_2D<long long> pt1{(long long)round(points[0].x() * mInputPrecision), (long long)round(points[0].y() * mInputPrecision)};
    Point_2D<long long> pt2{(long long)round(points[1].x() * mInputPrecision), (long long)round(points[1].y() * mInputPrecision)};
    return isOrthogonalSegment(pt1, pt2);
}

bool DesignRuleChecker::isDiagonalSegment(Point_2D<long long> &pt1, Point_2D<long long> &pt2) {
    if (pt1.x() != pt2.x() && pt1.y() != pt2.y() && abs(pt1.x() - pt2.x()) == abs(pt1.y() - pt2.y())) {
        return true;
    }
    return false;
}

bool DesignRuleChecker::isDiagonalSegment(points_2d &points) {
    Point_2D<long long> pt1{(long long)round(points[0].x() * mInputPrecision), (long long)round(points[0].y() * mInputPrecision)};
    Point_2D<long long> pt2{(long long)round(points[1].x() * mInputPrecision), (long long)round(points[1].y() * mInputPrecision)};
    return isDiagonalSegment(pt1, pt2);
}

bool DesignRuleChecker::isPadstackAndSegmentHaveSameLayer(instance &inst, padstack &pad, Segment &seg) {
    auto pinLayers = mDb.getPinLayer(inst.getId(), pad.getId());
    for (const auto layerId : pinLayers) {
        if (layerId == mDb.getLayerId(seg.getLayer())) {
            return true;
        }
    }
    return false;
}