#include "time_helper.hpp"

c_time_point TimeHelper::get_time(string time_title, bool display_time){
    auto now = high_resolution_clock::now();
    auto n = std::chrono::system_clock::to_time_t(now);
    if(display_time) cout << time_title << ": " << std::put_time(std::localtime(&n), "%F %T\n") << std::flush;
    return now;
}

time_pair TimeHelper::convert_from_ms(int millis){
    float duration = (float)millis;
    time_pair temp = {duration, "ms"};
    if(duration > MILLISECS_IN_SEC){
        duration /= MILLISECS_IN_SEC;
        if(duration > SECS_IN_MIN){
            duration /= SECS_IN_MIN;
            temp.second = "mins";
        }
        else temp.second = "s";
    }
    temp.first = duration;
    return temp;
}

int TimeHelper::convert_to_ms(time_pair given_time){
    int multipler = 1;
    if(given_time.second == "ms") return given_time.first;
    else if(given_time.second == "s") multipler = MILLISECS_IN_SEC;
    else if(given_time.second == "mins") multipler = MILLISECS_IN_SEC*SECS_IN_MIN;
    else if(given_time.second == "hrs") multipler = MILLISECS_IN_SEC*SECS_IN_MIN*MINS_IN_HOUR;
    return given_time.first*multipler;
}
