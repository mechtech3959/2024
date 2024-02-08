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
constexpr units::inch_t RedTapeX = 420.02_in;
constexpr units::inch_t BlueTapeX =231.2_in;
constexpr units::inch_t TapeOffset = 0_in;
// https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf pg 4 for april ID
constexpr units::inch_t sourceC =9.68_in;  // april tag #1 & #10 far from wall 
constexpr units::inch_t sourceF =34.79_in;  // april tag #2 & #9 close to the wall
constexpr units::inch_t speakerS =196.17_in; // april tag #3 source side of the red station
constexpr units::inch_t speakerA =218.42_in; // april tag #4 amp side
constexpr units::inch_t RAmp =323.00_in; // april tag #5 red amp on the right side of the red side
constexpr units::inch_t BAmp =323.00_in; // april tag #6 blue amp on the left side of the blue side

constexpr units::inch_t Grid3Y = 174.19_in; // last year 
constexpr units::inch_t Grid2Y = 108.19_in; // last year
constexpr units::inch_t Grid1Y = 42.19_in; // last year
constexpr units::inch_t BlueYOffSetLeft = 4_in;// prob same from last year
constexpr units::inch_t RedYOffSetLeft = 0_in;// prob same from last year

constexpr units::degree_t RedHeading = 0_deg;// prob same from last year
constexpr units::degree_t BlueHeading = 179.9_deg;// prob same from last year
