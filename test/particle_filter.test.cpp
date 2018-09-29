#include <test.hpp>


#include <cmath>
#include <vector>

#define protected public

#include <particle_filter.h>

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

TEST(ParticleFilter, DataAssociation) {
    ParticleFilter filter{10};

    std::vector<LandmarkObs> predicted{
            {0, 0, 0},
            {1, 1, 1},
            {2, 2, 2},
            {3, 3, 3}
    };

    std::vector<LandmarkObs> observations{
            {0, 3, 3},
            {1, 2, 2},
            {2, 1, 1},
            {3, 0, 0}
    };

    filter.dataAssociation(predicted, observations);

    ASSERT_EQ(observations[0].id, 3);
    ASSERT_EQ(observations[1].id, 2);
    ASSERT_EQ(observations[2].id, 1);
    ASSERT_EQ(observations[3].id, 0);
}

TEST(ParticleFilter, Resamle) {
    ParticleFilter filter{5};

    double std_dev[] = {0, 0, 0};
    filter.init(0, 0, 0, std_dev);

    filter.weights = {10, 0, 0, 0, 0};

    filter.resample();

    for (Particle &particle: filter.particles) {
        ASSERT_EQ(particle.id, 0);
    }
}

TEST(ParticleFilter, FindLandmarksInRange) {
    ParticleFilter filter;

    std::vector<Map::single_landmark_s> landmark_list = {
            {0, 3,   4}, // range == 25
            {1, 3.1, 4} // range == 25.61
    };

    auto result = filter.findLandmarksInRange(0, 0, 5, landmark_list);

    ASSERT_EQ(result.size(), 1);
    ASSERT_EQ(result[0].id, 0);
}