#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "map_data.hpp"

class AStar{
    public:
        AStar();
        void solve();
        pair<vector<pair<int,int>>, float> reconstruct_path();

    private:
        int get_f_score();
};

#endif // A_STAR_HPP