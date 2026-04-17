#include "types.hpp"

auto_buff::RunePoints auto_buff::RunePoints::operator+ (const RunePoints &other) const{
    RunePoints res;
    res.center = center + other.center;
    res.bottom_right = bottom_right + other.bottom_right;
    res.top_right = top_right + other.top_right;
    res.top_left = top_left + other.top_left;
    res.bottom_left = bottom_left + other.bottom_left;
    return res;
}

auto_buff::RunePoints auto_buff::RunePoints::operator/(const float &other) const {
    RunePoints res;
    res.center = center / other;
    res.bottom_right = bottom_right / other;
    res.top_right = top_right / other;
    res.top_left = top_left / other;
    res.bottom_left = bottom_left / other;
    return res;
}

auto_buff::RunePoints auto_buff::RunePoints::operator*(const float &other) const {
    RunePoints res;
    res.center = center * other;
    res.bottom_right = bottom_right * other;
    res.top_right = top_right * other;
    res.top_left = top_left * other;
    res.bottom_left = bottom_left * other;
    return res;
}