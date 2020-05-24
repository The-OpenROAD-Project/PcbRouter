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

    // Iterate nets
    for (auto &net : mDb.getNets()) {
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
                linestring_double_t bgLs2{point_double_t(ptsSeg1[0].x(), ptsSeg1[0].y()), point_double_t(ptsSeg1[1].x(), ptsSeg1[1].y())};
            }
        }
    }

    std::cout << "Total # acute angle violations: " << numViolations << std::endl;
    std::cout << "End of " << __FUNCTION__ << "()..." << std::endl;
    return numViolations;
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