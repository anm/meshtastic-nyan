#pragma once

constexpr double pi = 3.141592653589793;
constexpr double tau = pi * 2.0;
constexpr double deg_to_rad = tau / 360.0;
constexpr double rad_to_deg = 360.0 / tau;

constexpr uint16_t METERS_PER_NM = 1852;
constexpr double MPS_TO_KNOTS = (60.0 * 60.0) / METERS_PER_NM;
constexpr double KPH_TO_KNOTS = 1000.0 / METERS_PER_NM;
