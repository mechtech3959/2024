#pragma once


#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>

#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/dimensionless.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
//idk
constexpr units::inch_t RedTapeX = 420.2_in;
constexpr units::inch_t BlueTapeX =231.2_in;
constexpr units::inch_t TapeOffset = 0_in;
constexpr units::inch_t Grid3Y = 174.19_in; // april tag Ycordlocations 
constexpr units::inch_t Grid2Y = 108.19_in;
constexpr units::inch_t Grid1Y = 42.19_in;
constexpr units::inch_t BlueYOffSetLeft = 4_in;// prob same from last year
constexpr units::inch_t RedYOffSetLeft = 0_in;// prob same from last year

constexpr units::degree_t RedHeading = 0_deg;// prob same from last year
constexpr units::degree_t BlueHeading = 179.9_deg;// prob same from last year
