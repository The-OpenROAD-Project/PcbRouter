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
        const auto &gp2 = gridPins2.at(nearestPinId2);

        // Add GridPins as median (merging) points and as pseudo through-hole pin
        GridPin &gridPin = this->getNewGridPin();
        int medianPtX = (gp1.getPinWithLayers().front().m_x + gp2.getPinWithLayers().front().m_x) / 2;
        int medianPtY = (gp1.getPinWithLayers().front().m_y + gp2.getPinWithLayers().front().m_y) / 2;

        for (int layerId = startLayerId; layerId <= endLayerId; ++layerId) {
            gridPin.addPinWithLayer(Location(medianPtX, medianPtY, layerId));
        }

        // Debugging
        if (GlobalParam::gVerboseLevel <= VerboseLevel::DEBUG) {
            std::cout << "GP1: " << gp1.getPinWithLayers().front() << ", GP2: " << gp2.getPinWithLayers().front() << std::endl;
            std::cout << "Median Point: (" << medianPtX << ", " << medianPtY << ")" << std::endl;
        }
    }
}