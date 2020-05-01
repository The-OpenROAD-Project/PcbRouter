#include "GridDiffPairNet.h"

void GridDiffPairNet::setupDiffPairGridPins(const int startLayerId, const int endLayerId) {
    const std::vector<GridPin> &gridPins1 = this->mNet1.getGridPins();
    const std::vector<GridPin> &gridPins2 = this->mNet2.getGridPins();

    // if (gridPins1.size() != 2) {
    //     std::cout << __FUNCTION__ << "(): supports only two pin nets of differential nets."
    //               << " NetId: " << this->mNet1.getNetId() << " has net degree of " << gridPins1.size() << std::endl;
    // }
    // if (gridPins2.size() != 2) {
    //     std::cout << __FUNCTION__ << "(): supports only two pin nets of differential nets."
    //               << " NetId: " << this->mNet2.getNetId() << " has net degree of " << gridPins2.size() << std::endl;
    // }

    if (GlobalParam::gVerboseLevel <= VerboseLevel::DEBUG) {
        std::cout << std::endl
                  << __FUNCTION__ << "(): start finding differential pairs' median points..." << std::endl;
    }

    // For each GridPin1 find a nearest GridPin2
    for (unsigned int i = 0; i < gridPins1.size(); ++i) {
        const auto &gp1 = gridPins1.at(i);
        int minHPWL = std::numeric_limits<int>::max();
        int nearestPinId2 = -1;
        for (unsigned int j = 0; j < gridPins2.size(); ++j) {
            const auto &gp2 = gridPins2.at(j);
            Location gp1center = gp1.getPinWithLayers().front();
            Location gp2center = gp2.getPinWithLayers().front();

            int HPWL = abs(gp1center.m_x - gp2center.m_x) + abs(gp1center.m_y - gp2center.m_y);
            if (HPWL < minHPWL) {
                minHPWL = HPWL;
                nearestPinId2 = i;
            }
        }

        // Record the pair
        this->mGridPinPairsId.emplace_back(std::make_pair(i, nearestPinId2));
    }

    // Add GridPins as median (merging) points and as pseudo through-hole pin
    for (const auto &pinPairIds : this->mGridPinPairsId) {
        const auto &gp1 = gridPins1.at(pinPairIds.first);
        const auto &gp2 = gridPins2.at(pinPairIds.second);

        GridPin &gridPin = this->getNewGridPin();
        int halfDiffX = abs(gp1.getPinWithLayers().front().m_x - gp2.getPinWithLayers().front().m_x) / 2;
        int halfDiffY = abs(gp1.getPinWithLayers().front().m_y - gp2.getPinWithLayers().front().m_y) / 2;

        int medianPtX = (gp1.getPinWithLayers().front().m_x + gp2.getPinWithLayers().front().m_x) / 2;
        int medianPtY = (gp1.getPinWithLayers().front().m_y + gp2.getPinWithLayers().front().m_y) / 2;

        for (int layerId = startLayerId; layerId <= endLayerId; ++layerId) {
            gridPin.addPinWithLayer(Location(medianPtX, medianPtY, layerId));

            // Test for multiple pin locations
            gridPin.addPinWithLayer(Location(medianPtX + halfDiffY, medianPtY + halfDiffX, layerId));
            gridPin.addPinWithLayer(Location(medianPtX - halfDiffY, medianPtY - halfDiffX, layerId));
        }

        // Debugging
        if (GlobalParam::gVerboseLevel <= VerboseLevel::DEBUG) {
            std::cout << "GP1: " << gp1.getPinWithLayers().front() << ", GP2: " << gp2.getPinWithLayers().front() << std::endl;
            std::cout << "Median Point: (" << medianPtX << ", " << medianPtY << "), GridPins: " << std::endl;
            for (const auto &gp : gridPin.getPinWithLayers()) {
                std::cout << gp << std::endl;
            }
        }
    }
}

void GridDiffPairNet::postProcessingGridPaths() {
    this->mNet1.removeFirstGridPathRedudantLocations();
    this->mNet2.removeFirstGridPathRedudantLocations();
}

void GridDiffPairNet::separateGridPathsIntoTwo(const int traceClr, const int traceDiagClr, const int traceDiagOffset) {
    const auto &gridPaths = this->getGridPaths();
    if (gridPaths.size() > 1) {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::CRITICAL) {
            std::cout << __FUNCTION__ << "(): Support separating two-pin nets only." << std::endl;
        }
        return;
    }

    if (GlobalParam::gVerboseLevel <= VerboseLevel::DEBUG) {
        std::cout << __FUNCTION__ << "(): Grouped Nets: " << std::endl;
        for (const auto &gp : this->getGridPaths()) {
            gp.printSegments();
            gp.printLocations();
        }
    }

    for (const auto &gp : gridPaths) {
        this->separateGridPath(gp, traceClr, traceDiagClr, traceDiagOffset);
    }

    return;
}

void GridDiffPairNet::separateGridPath(const GridPath &path, const int traceClr, const int traceDiagClr, const int traceDiagOffset) {
    const auto &segs = path.getSegments();
    if (segs.empty())
        return;

    // TODO:: Decide which net is at the left/right of the thicker net
    // By exchanging the Net1/Net2 order
    auto &gpNetL = this->mNet2.getNewGridPath();
    auto &gpNetR = this->mNet1.getNewGridPath();

    auto &locsL = gpNetL.setSegments();
    auto &locsR = gpNetR.setSegments();

    auto prevPointIte = segs.begin();
    auto pointIte = ++segs.begin();
    auto nextPointIte = std::next(pointIte, 1);

    // Handle Middle Points
    for (; nextPointIte != segs.end();) {
        // Make sure we are at a bending point but not a via
        if (prevPointIte->z() == pointIte->z() && pointIte->z() == nextPointIte->z()) {
            Location middleL{*pointIte};
            Location middleR{*pointIte};

            // Horizontal to Diagonal (45-degree turn)
            if (pointIte->x() != prevPointIte->x() && pointIte->y() == prevPointIte->y() &&
                pointIte->x() != nextPointIte->x() && pointIte->y() != nextPointIte->y()) {
                this->locBetweenHorizontalAndDiagonal(*prevPointIte, *pointIte, *nextPointIte, traceClr, traceDiagOffset, middleL, middleR);
            }
            // Diagonal to Horizontal (45-degree turn)
            else if (pointIte->x() != nextPointIte->x() && pointIte->y() == nextPointIte->y() &&
                     pointIte->x() != prevPointIte->x() && pointIte->y() != prevPointIte->y()) {
                // Note: Swap next/prev iterators and middleL/R
                this->locBetweenHorizontalAndDiagonal(*nextPointIte, *pointIte, *prevPointIte, traceClr, traceDiagOffset, middleR, middleL);
            }
            // Vertical to Diagonal (45-degree turn)
            else if (pointIte->x() == prevPointIte->x() && pointIte->y() != prevPointIte->y() &&
                     pointIte->x() != nextPointIte->x() && pointIte->y() != nextPointIte->y()) {
                this->locBetweenVerticalAndDiagonal(*prevPointIte, *pointIte, *nextPointIte, traceClr, traceDiagOffset, middleL, middleR);
            }
            // Diagonal to Vertical (45-degree turn)
            else if (pointIte->x() == nextPointIte->x() && pointIte->y() != nextPointIte->y() &&
                     pointIte->x() != prevPointIte->x() && pointIte->y() != prevPointIte->y()) {
                // Note: Swap next/prev iterators and middleL/R
                this->locBetweenVerticalAndDiagonal(*nextPointIte, *pointIte, *prevPointIte, traceClr, traceDiagOffset, middleR, middleL);
            }
            // Diagonal to Diagonal (90-degree turn)
            else if (pointIte->x() != prevPointIte->x() && pointIte->y() != prevPointIte->y() &&
                     pointIte->x() != nextPointIte->x() && pointIte->y() != nextPointIte->y()) {
                this->locBetweenDiagonalAndDiagonal(*prevPointIte, *pointIte, *nextPointIte, traceClr, traceDiagOffset, middleL, middleR);
            }
            // Orthogonal to Orthogonal  (90-degree turn)
            else if ((pointIte->x() == prevPointIte->x() && pointIte->y() != prevPointIte->y() &&
                      pointIte->x() != nextPointIte->x() && pointIte->y() == nextPointIte->y()) ||
                     (pointIte->x() != prevPointIte->x() && pointIte->y() == prevPointIte->y() &&
                      pointIte->x() == nextPointIte->x() && pointIte->y() != nextPointIte->y())) {
                this->locBetweenOrthogonalAndOrthogonal(*prevPointIte, *pointIte, *nextPointIte, traceClr, traceDiagOffset, middleL, middleR);
            } else {
                if (GlobalParam::gVerboseLevel <= VerboseLevel::WARNING) {
                    std::cout << __FUNCTION__ << "(): Unexpected cases when separting middle points into two." << std::endl;
                }
            }

            locsL.emplace_back(middleL);
            locsR.emplace_back(middleR);
        }

        // Move to the next segment
        ++pointIte;
        ++nextPointIte;
        ++prevPointIte;
    }

    // Handle start/end points or special case without middle points
    if (!locsR.empty() && !locsL.empty()) {
        // Handle First Point
        Location startL{segs.front()};
        Location startR{segs.front()};
        // Note: Swap next/prev iterators and endL/R
        this->endLocByStartEndLocations(*(std::next(segs.begin())), segs.front(), locsR.front(), locsL.front(), traceClr, traceDiagOffset, startR, startL);
        locsL.emplace_front(startL);
        locsR.emplace_front(startR);

        // Handle Last Point
        Location endL{*(std::prev(segs.end()))};
        Location endR{*(std::prev(segs.end()))};
        this->endLocByStartEndLocations(*(std::prev(segs.end(), 2)), *(std::prev(segs.end())), locsL.back(), locsR.back(), traceClr, traceDiagOffset, endL, endR);
        locsL.emplace_back(endL);
        locsR.emplace_back(endR);
    } else {
        // No Middle points pushed....
        // Directly calculate start and end points
        const Location &start = segs.front();
        const Location &end = segs.back();

        this->locsForStartEndLocationsWithoutMiddlePoints(start, end, traceClr, traceDiagClr, locsL, locsR);
    }

    // // Handle Vias
    // if (segs.size() < 2) return;

    // pointIte = segs.begin();
    // nextPointIte = ++segs.begin();

    // for (; nextPointIte != segs.end();) {
    //     if (pointIte->x() == nextPointIte->x() && pointIte->y() == nextPointIte->y() &&
    //         pointIte->z() != nextPointIte->z()) {
    //     }

    //     // Move to the next segment
    //     ++pointIte;
    //     ++nextPointIte;
    // }
}

void GridDiffPairNet::locsForStartEndLocationsWithoutMiddlePoints(const Location &start, const Location &end,
                                                                  const int traceClr, const int traceDiagClr,
                                                                  std::list<Location> &locsL, std::list<Location> &locsR) {
    // Vertical  end
    //            ^
    //          start
    if (start.x() == end.x() && start.y() < end.y()) {
        locsL.emplace_back(Location{start.x() - traceClr, start.y(), start.z()});
        locsL.emplace_back(Location{end.x() - traceClr, end.y(), end.z()});
        locsR.emplace_back(Location{start.x() + traceClr, start.y(), start.z()});
        locsR.emplace_back(Location{end.x() + traceClr, end.y(), end.z()});
    }
    // Vertical  start
    //             v
    //            end
    else if (start.x() == end.x() && start.y() > end.y()) {
        locsL.emplace_back(Location{start.x() + traceClr, start.y(), start.z()});
        locsL.emplace_back(Location{end.x() + traceClr, end.y(), end.z()});
        locsR.emplace_back(Location{start.x() - traceClr, start.y(), start.z()});
        locsR.emplace_back(Location{end.x() - traceClr, end.y(), end.z()});
    }
    // Horizontal, start->end
    else if (start.x() < end.x() && start.y() == end.y()) {
        locsL.emplace_back(Location{start.x(), start.y() + traceClr, start.z()});
        locsL.emplace_back(Location{end.x(), end.y() + traceClr, end.z()});
        locsR.emplace_back(Location{start.x(), start.y() - traceClr, start.z()});
        locsR.emplace_back(Location{end.x(), end.y() - traceClr, end.z()});
    }
    // Horizontal, end<-start
    else if (start.x() > end.x() && start.y() == end.y()) {
        locsL.emplace_back(Location{start.x(), start.y() - traceClr, start.z()});
        locsL.emplace_back(Location{end.x(), end.y() - traceClr, end.z()});
        locsR.emplace_back(Location{start.x(), start.y() + traceClr, start.z()});
        locsR.emplace_back(Location{end.x(), end.y() + traceClr, end.z()});
    }
    // LL -> UR
    else if (start.x() < end.x() && start.y() < end.y()) {
        locsL.emplace_back(Location{start.x() - traceDiagClr, start.y() + traceDiagClr, start.z()});
        locsL.emplace_back(Location{end.x() - traceDiagClr, end.y() + traceDiagClr, end.z()});
        locsR.emplace_back(Location{start.x() + traceDiagClr, start.y() - traceDiagClr, start.z()});
        locsR.emplace_back(Location{end.x() + traceDiagClr, end.y() - traceDiagClr, end.z()});
    }
    // UR -> LL
    else if (start.x() > end.x() && start.y() > end.y()) {
        locsL.emplace_back(Location{start.x() + traceDiagClr, start.y() - traceDiagClr, start.z()});
        locsL.emplace_back(Location{end.x() + traceDiagClr, end.y() - traceDiagClr, end.z()});
        locsR.emplace_back(Location{start.x() - traceDiagClr, start.y() + traceDiagClr, start.z()});
        locsR.emplace_back(Location{end.x() - traceDiagClr, end.y() + traceDiagClr, end.z()});
    }
    // UL -> LR
    else if (start.x() < end.x() && start.y() > end.y()) {
        locsL.emplace_back(Location{start.x() + traceDiagClr, start.y() + traceDiagClr, start.z()});
        locsL.emplace_back(Location{end.x() + traceDiagClr, end.y() + traceDiagClr, end.z()});
        locsR.emplace_back(Location{start.x() - traceDiagClr, start.y() - traceDiagClr, start.z()});
        locsR.emplace_back(Location{end.x() - traceDiagClr, end.y() - traceDiagClr, end.z()});
    }
    // LR -> UL
    else if (start.x() > end.x() && start.y() < end.y()) {
        locsL.emplace_back(Location{start.x() - traceDiagClr, start.y() - traceDiagClr, start.z()});
        locsL.emplace_back(Location{end.x() - traceDiagClr, end.y() - traceDiagClr, end.z()});
        locsR.emplace_back(Location{start.x() + traceDiagClr, start.y() + traceDiagClr, start.z()});
        locsR.emplace_back(Location{end.x() + traceDiagClr, end.y() + traceDiagClr, end.z()});
    }
}

void GridDiffPairNet::endLocByStartEndLocations(const Location &start, const Location &end, const Location &startL, const Location &startR,
                                                const int traceClr, const int traceDiagOffset, Location &endL, Location &endR) {
    // 8 Cases
    // case 1:  L
    //          s - e
    //          R

    // case 2:      R
    //          e - s
    //              L

    // case 3:   e
    //           |
    //         L s R

    // case 4: R s L
    //           |
    //           e

    // case 5:    e
    //           /
    //        L s R

    // case 6:  R s L
    //           /
    //          e

    // case 7: e
    //          \
    //         L s R

    // case 8: R s L
    //            \
    //             e

    // Horizontal
    if (start.m_y == end.m_y && start.m_x != end.m_x) {
        endL.m_x = end.m_x;
        endR.m_x = end.m_x;

        if (start.m_x < end.m_x) {
            // case 1
            endL.m_y = end.m_y + traceClr;
            endR.m_y = end.m_y - traceClr;
        } else {
            // case 2
            endL.m_y = end.m_y - traceClr;
            endR.m_y = end.m_y + traceClr;
        }
    }
    // Vertical
    else if (start.m_y != end.m_y && start.m_x == end.m_x) {
        endL.m_y = end.m_y;
        endR.m_y = end.m_y;

        if (start.m_y < end.m_y) {
            // case 3
            endL.m_x = end.m_x - traceClr;
            endR.m_x = end.m_x + traceClr;
        } else {
            // case 4
            endL.m_x = end.m_x + traceClr;
            endR.m_x = end.m_x - traceClr;
        }
    }
    // LL -> UR, case 5
    // UR -> LL, case 6
    // LR -> UL, case 7
    // UL -> LR, case 8
    else if ((start.m_x < end.m_x && start.m_y < end.m_y) ||
             (start.m_x > end.m_x && start.m_y > end.m_y) ||
             (start.m_x > end.m_x && start.m_y < end.m_y) ||
             (start.m_x < end.m_x && start.m_y > end.m_y)) {
        endL.m_x = startL.m_x + (end.m_x - start.m_x);
        endL.m_y = startL.m_y + (end.m_y - start.m_y);
        endR.m_x = startR.m_x + (end.m_x - start.m_x);
        endR.m_y = startR.m_y + (end.m_y - start.m_y);
    }
}

void GridDiffPairNet::locBetweenOrthogonalAndOrthogonal(const Location &orth1, const Location &middle, const Location &orth2,
                                                        const int traceClr, const int traceDiagOffset,
                                                        Location &middleL, Location &middleR) {
    // 8 cases
    // case 1:
    //  <--
    //     |
    //   L | R

    // case 2:
    //      -->
    //     |
    //   L | R

    // case 3:
    //   R | L
    //     |
    //      -->

    // case 4:
    //   R | L
    //     |
    //  <--

    // case 5:
    // -->|
    //    |
    //  R v L

    // case 6:
    //    |<--
    //    |
    //  R v L

    // case 7:
    //  L ^ R
    //    |
    // -->|

    // case 8:
    //  L ^ R
    //    |
    //    |<--

    if (orth1.m_x == middle.m_x) {
        if (orth2.m_x < middle.m_x && orth2.m_y > orth1.m_y) {
            //case 1
            middleL.m_x -= traceClr;
            middleL.m_y -= traceClr;
            middleR.m_x += traceClr;
            middleR.m_y += traceClr;
        } else {
            //case 4
            middleL.m_x += traceClr;
            middleL.m_y -= traceClr;
            middleR.m_x -= traceClr;
            middleR.m_y += traceClr;
        }
        if (orth2.m_x > middle.m_x && orth2.m_y > orth1.m_y) {
            //case 2
            middleL.m_x -= traceClr;
            middleL.m_y += traceClr;
            middleR.m_x += traceClr;
            middleR.m_y -= traceClr;
        } else {
            //case 3
            middleL.m_x += traceClr;
            middleL.m_y += traceClr;
            middleR.m_x -= traceClr;
            middleR.m_y -= traceClr;
        }
    } else if (orth1.m_y == middle.m_y) {
        if (orth2.m_x < orth1.m_x && orth2.m_y > middle.m_y) {
            //case 8
            middleL.m_x -= traceClr;
            middleL.m_y -= traceClr;
            middleR.m_x += traceClr;
            middleR.m_y += traceClr;
        } else {
            //case 6
            middleL.m_x += traceClr;
            middleL.m_y -= traceClr;
            middleR.m_x -= traceClr;
            middleR.m_y += traceClr;
        }
        if (orth2.m_x > orth1.m_x && orth2.m_y > middle.m_y) {
            //case 7
            middleL.m_x -= traceClr;
            middleL.m_y += traceClr;
            middleR.m_x += traceClr;
            middleR.m_y -= traceClr;
        } else {
            //case 5
            middleL.m_x += traceClr;
            middleL.m_y += traceClr;
            middleR.m_x -= traceClr;
            middleR.m_y -= traceClr;
        }
    } else {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::WARNING) {
            std::cout << __FUNCTION__ << "(): Unexpected cases when separting two orthogonal segments's middle points." << std::endl;
        }
    }
}
void GridDiffPairNet::locBetweenDiagonalAndDiagonal(const Location &diag1, const Location &middle, const Location &diag2,
                                                    const int traceClr, const int traceDiagOffset,
                                                    Location &middleL, Location &middleR) {
    // 8 cases
    // Case 1:
    //  R \ L
    //     \
    //     /
    //    v

    // Case 2:
    //  R / L
    //   /
    //   \
    //    v

    // Case 3:
    //    \
    //     \
    //     /
    //  L ^ R

    // Case 4:
    //    /
    //   /
    //   \
    //  L ^ R

    // Case 5:
    //       /\
    //      /  \
    //   L / R  v

    // Case 6:
    // R \ L  ^
    //    \  /
    //     \/

    // Case 7:
    //       /\
    //      /  \
    //     /  L ^ R

    // Case 8:
    //   \  R v L
    //    \  /
    //     \/
    if (diag1.m_y > middle.m_y && middle.m_y > diag2.m_y) {
        //case 1
        //case 2
        middleL.m_x += (traceClr + traceDiagOffset);
        middleR.m_x -= (traceClr + traceDiagOffset);
    } else if (diag1.m_y < middle.m_y && middle.m_y < diag2.m_y) {
        //case 3
        //case 4
        middleL.m_x -= (traceClr + traceDiagOffset);
        middleR.m_x += (traceClr + traceDiagOffset);
    } else if (diag1.m_x < middle.m_x && middle.m_x < diag2.m_x) {
        //case 5
        //case 6
        middleL.m_y += (traceClr + traceDiagOffset);
        middleR.m_y -= (traceClr + traceDiagOffset);
    } else if (diag1.m_x > middle.m_x && middle.m_x > diag2.m_x) {
        //case 7
        //case 8
        middleL.m_y -= (traceClr + traceDiagOffset);
        middleR.m_y += (traceClr + traceDiagOffset);
    } else {
        if (GlobalParam::gVerboseLevel <= VerboseLevel::WARNING) {
            std::cout << __FUNCTION__ << "(): error cases of middle points between two diagonal segments." << std::endl;
        }
    }
}

void GridDiffPairNet::locBetweenHorizontalAndDiagonal(const Location &horizon, const Location &middle, const Location &diag,
                                                      const int traceClr, const int traceDiagOffset,
                                                      Location &middleL, Location &middleR) {
    // 4 cases
    // Case 1:
    //           /
    // L        /
    // ------->/
    // R

    // Case 2:
    // L
    // ------->\
    // R        \
    //           \

    // Case 3:
    //   \
    //    \         R
    //     \<--------
    //              L

    // Case 4:
    //            R
    //   /<--------
    //  /         L
    // /

    if (diag.m_x > middle.m_x) {
        middleL.m_y = middle.m_y + traceClr;
        middleR.m_y = middle.m_y - traceClr;

        if (diag.m_y > middle.m_y) {
            //Case 1
            middleL.m_x = middle.m_x - traceDiagOffset;
            middleR.m_x = middle.m_x + traceDiagOffset;
        } else {
            //Case 2
            middleL.m_x = middle.m_x + traceDiagOffset;
            middleR.m_x = middle.m_x - traceDiagOffset;
        }

    } else {
        middleL.m_y = middle.m_y - traceClr;
        middleR.m_y = middle.m_y + traceClr;

        if (diag.m_y > middle.m_y) {
            //Case 3
            middleL.m_x = middle.m_x - traceDiagOffset;
            middleR.m_x = middle.m_x + traceDiagOffset;
        } else {
            //Case 4
            middleL.m_x = middle.m_x + traceDiagOffset;
            middleR.m_x = middle.m_x - traceDiagOffset;
        }
    }
}

void GridDiffPairNet::locBetweenVerticalAndDiagonal(const Location &vertical, const Location &middle, const Location &diag,
                                                    const int traceClr, const int traceDiagOffset,
                                                    Location &middleL, Location &middleR) {
    // 4 cases
    // Case 1:
    //         /
    //        /
    //       ^
    //     L | R

    // Case 2:
    //      \
    //       \
    //       ^
    //     L | R

    // Case 3:
    //     R | L
    //       v
    //        \
    //         \

    // Case 4:
    //     R | L
    //       v
    //       /
    //      /

    if (diag.m_y > middle.m_y) {
        middleL.m_x = middle.m_x - traceClr;
        middleR.m_x = middle.m_x + traceClr;

        if (diag.m_x > middle.m_x) {
            //Case 1
            middleL.m_y = middle.m_y + traceDiagOffset;
            middleR.m_y = middle.m_y - traceDiagOffset;
        } else {
            //Case 2
            middleL.m_y = middle.m_y - traceDiagOffset;
            middleR.m_y = middle.m_y + traceDiagOffset;
        }

    } else {
        middleL.m_x = middle.m_x + traceClr;
        middleR.m_x = middle.m_x - traceClr;

        if (diag.m_x > middle.m_x) {
            //Case 3
            middleL.m_y = middle.m_y + traceDiagOffset;
            middleR.m_y = middle.m_y - traceDiagOffset;
        } else {
            //Case 4
            middleL.m_y = middle.m_y - traceDiagOffset;
            middleR.m_y = middle.m_y + traceDiagOffset;
        }
    }
}