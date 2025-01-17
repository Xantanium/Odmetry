#include "Odometry"

namespace rudra {

template <typename Strategy>

Pose Odometry<Strategy>::update(long countX, long countYR, long countYL)
{
    return strategy.update(countX, countYR, countYL);
}

} // namespace rudra
