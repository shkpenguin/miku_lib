#pragma once

#include <cmath>
#include <vector>
#include <string>
#define FMT_HEADER_ONLY
#include "fmt/core.h"

enum AngleType {
    COMPASS,
    STANDARD
};

enum AngleUnit {
    DEGREES,
    RADIANS
};

template<AngleUnit U = RADIANS, AngleType T = STANDARD>
struct AngleTemplate {
    float value = 0.0;

    AngleTemplate(float value = 0.0) : value(value) {}

    template<AngleUnit OtherU, AngleType OtherT>
    AngleTemplate(const AngleTemplate<OtherU, OtherT>& other) {
        float tmp = other.value;

        if constexpr (OtherU == DEGREES) tmp = tmp * M_PI / 180.0;
        if constexpr (OtherT == COMPASS) tmp = M_PI / 2.0 - tmp;  // compass → standard

        if constexpr (T == COMPASS) tmp = M_PI / 2.0 - tmp;        // standard → compass

        if constexpr (U == DEGREES) tmp = tmp * 180.0 / M_PI;

        value = tmp;
    }
    operator float() const { return value; }
    AngleTemplate operator+(const AngleTemplate& other) const {
        return AngleTemplate(value + other.value);
    }
    AngleTemplate operator+(const float& other) const {
        return AngleTemplate(value + other);
    }
    AngleTemplate& operator+=(const AngleTemplate& other) {
        value += other.value;
        return *this;
    }
    AngleTemplate& operator+=(const float& other) {
        value += other;
        return *this;
    }
    AngleTemplate operator-(const AngleTemplate& other) const {
        return AngleTemplate(value - other.value);
    }
    AngleTemplate operator-(const float& other) const {
        return AngleTemplate(value - other);
    }
    AngleTemplate& operator-=(const AngleTemplate& other) {
        value -= other.value;
        return *this;
    }
    AngleTemplate& operator-=(const float& other) {
        value -= other;
        return *this;
    }
    AngleTemplate operator*(float scalar) const {
        return AngleTemplate(value * scalar);
    }
    AngleTemplate& operator*=(float scalar) {
        value *= scalar;
        return *this;
    }
    AngleTemplate operator/(float scalar) const {
        return AngleTemplate(value / scalar);
    }
    AngleTemplate& operator/=(float scalar) {
        value /= scalar;
        return *this;
    }
    ~AngleTemplate() = default;
    template<AngleType K = T>
    inline constexpr std::enable_if_t<K == COMPASS, AngleTemplate<U, STANDARD>> standard() const {
        if constexpr (U == DEGREES) return AngleTemplate<DEGREES, STANDARD>(90.0 - value);
        return AngleTemplate<RADIANS, STANDARD>(M_PI / 2.0 - value);
    };
    template<AngleType K = T>
    inline constexpr std::enable_if_t<K == STANDARD, AngleTemplate<U, COMPASS>> compass() const {
        if constexpr (U == DEGREES) return AngleTemplate<DEGREES, COMPASS>(90.0 - value);
        return AngleTemplate<RADIANS, COMPASS>(M_PI / 2.0 - value);
    };
    template<AngleUnit V = U>
    inline constexpr std::enable_if_t<V == DEGREES, AngleTemplate<RADIANS, T>> radians() const {
        return AngleTemplate<RADIANS, T>(value * M_PI / 180.0);
    };
    template<AngleUnit V = U>
    inline constexpr std::enable_if_t<V == RADIANS, AngleTemplate<DEGREES, T>> degrees() const {
        return AngleTemplate<DEGREES, T>(value * 180.0 / M_PI);
    };
    // wrap to [-180, 180) for degrees or [-pi, pi) for radians
    inline float wrap() const {
        if constexpr (U == DEGREES) {
            float mod = std::fmod(value + 180.0, 360.0);
            if(mod < 0) mod += 360.0;
            return mod - 180.0;
        } else {
            float mod = std::fmod(value + M_PI, 2.0 * M_PI);
            if(mod < 0) mod += 2.0 * M_PI;
            return mod - M_PI;
        }
    };
    // wrap to [0, 360) for degrees or [0, 2pi) for radians
    inline float norm() const {
        if constexpr (U == DEGREES) {
            float mod = std::fmod(value, 360.0);
            if(mod < 0) mod += 360.0;
            return mod;
        } else {
            float mod = std::fmod(value, 2.0 * M_PI);
            if(mod < 0) mod += 2.0 * M_PI;
            return mod;
        }
    };
};

using standard_radians = AngleTemplate<RADIANS, STANDARD>;
using standard_degrees = AngleTemplate<DEGREES, STANDARD>;
using compass_radians = AngleTemplate<RADIANS, COMPASS>;
using compass_degrees = AngleTemplate<DEGREES, COMPASS>;

struct Point {
    float x;
    float y;

    Point(float x = 0.0, float y = 0.0) 
        : x(x), y(y) {}

    float magnitude() const {
        return std::hypot(x, y);
    }
    standard_radians angle_to(const Point& other) const {
        return standard_radians(atan2(other.y - y, other.x - x));
    }
    float distance_to(const Point& other) const {
        return std::hypot(other.x - x, other.y - y);
    }
};

struct Pose {
    float x; 
    float y;
    standard_radians theta;

    Pose(float x = 0.0, float y = 0.0, float theta = 0.0)
        : x(x), y(y), theta(theta) {}

    float magnitude() const {
        return std::hypot(x, y);
    }
    standard_radians angle_to(const Point& other) const {
        return standard_radians(atan2(other.y - y, other.x - x));
    }
    float distance_to(const Point& other) const {
        return std::hypot(other.x - x, other.y - y);
    }
    std::string to_string() const {
        return fmt::format("{:.1f} {:.1f} @{:.1f}", x, y, float(compass_degrees(theta).wrap()));
    }
};

struct Polygon {
    std::vector<Point> ccw_vertices;
};