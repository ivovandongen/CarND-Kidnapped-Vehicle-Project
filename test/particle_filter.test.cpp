#include <test.hpp>

#include <particle_filter.h>

#include <cmath>
#include <vector>

TEST(ParticleFilter, Init) {
    ParticleFilter filter{10};

    double std_dev[] = {.3, .3, .01};
    filter.init(10, 10, M_PI_2 / 2, std_dev);

    ASSERT_EQ(10, filter.particles.size());
}

TEST(ParticleFilter, PredictionStraight) {
    ParticleFilter filter{10};

    double std_dev[] = {.3, .3, .01};
    filter.init(10, 10, M_PI_2 / 2, std_dev);

    filter.prediction(0.1, std_dev, 10, 0);
    ASSERT_EQ(10, filter.particles.size());
}

TEST(ParticleFilter, PredictionCurve) {
    ParticleFilter filter{10};

    double std_dev[] = {.3, .3, .01};
    filter.init(10, 10, M_PI_2 / 2, std_dev);

    filter.prediction(0.1, std_dev, 10, 0.1);
    ASSERT_EQ(10, filter.particles.size());
}