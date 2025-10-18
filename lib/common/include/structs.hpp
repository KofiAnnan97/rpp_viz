#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "pp_constants.hpp"

struct SampleCountByAlgo{
    int rrt_star_count = AlgoConstants::DEFAULT_SAMPLE_COUNT;
    int prm_count = AlgoConstants::DEFAULT_SAMPLE_COUNT;
};

#endif // STRUCTS_HPP
