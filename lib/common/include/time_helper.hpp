#ifndef TIME_HELPER_HPP
#define TIME_HELPER_HPP

#include <chrono>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace std::chrono;

typedef std::chrono::_V2::system_clock::time_point c_time_point;
typedef pair<float,string> time_pair;

class TimeHelper {
    public:
        static c_time_point get_time(string time_title, bool display_time);
        static time_pair convert_from_ms(int millis);
        static int convert_to_ms(time_pair);

        static const int MINS_IN_HOUR = 60;
        static const int SECS_IN_MIN = 60;
        static const int MILLISECS_IN_SEC = 1000;
};

#endif // TIME_HELPER_HPP
