#pragma once
#include <atomic>
#include <mutex>
#include <memory>

struct SharedTwistData
{
    double linear_x = 0.0;
    double angular_z = 0.0;
    std::atomic_bool updated{false};
    std::mutex mtx;
};

typedef std::shared_ptr<SharedTwistData> SharedTwistDataPtr;
