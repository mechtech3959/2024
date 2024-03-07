// JUST HERE FOR LATER
#pragma once
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
 
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

// #include "frc/geometry/Rotation2d.h"
// #include "frc/geometry/Translation2d.h"
// #include "frc/trajectory/constraint/TrajectoryConstraint.h"
// idk
constexpr units::inch_t RedTapeX = 420.02_in;
// BLUE IS CONSIDERED 0,0 SIDE
constexpr units::inch_t BlueTapeX = 231.2_in;
constexpr units::inch_t TapeOffset = 0_in;
// https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf
// pg 4 for april ID
constexpr units::inch_t sourceB_farY = 9.68_in; // april tag #1   
constexpr units::inch_t sourceB_CloseY = 34.79_in; // aprile tag #2
constexpr units::inch_t sourceB_CloseY =34.79_in; // april tag #9 & #9 close to the wall
constexpr units::inch_t speakerSY = 196.17_in; // april tag #3 source side of the red station
constexpr units::inch_t speakerAY = 218.42_in; // april tag #4 amp side
constexpr units::inch_t RAmpY = 323.00_in; // april tag #5 red amp on the right side of the red side
constexpr units::inch_t BAmpY =323.00_in; // april tag #6 blue amp on the left side of the blue side

constexpr units::inch_t Grid3Y = 174.19_in;     // last year
constexpr units::inch_t Grid2Y = 108.19_in;     // last year
constexpr units::inch_t Grid1Y = 42.19_in;      // last year
constexpr units::inch_t BlueYOffSetLeft = 0_in; // This was set to 4 
constexpr units::inch_t RedYOffSetLeft = 0_in;  // prob same from last year

constexpr units::degree_t RedHeading = 0_deg;      
constexpr units::degree_t BlueHeading = 179.9_deg; 


constexpr frc::Translation2d BlueTop{ 250.50_in, 260.64_in}; // right of the field avoiding stage and goes to midfield
constexpr frc::Translation2d BlueBottom{ 250.50_in, 29.64_in}; // left of the field avoiding stage and goes to midfield
 constexpr frc::Translation2d MidfieldNote5{ 250.50_in, 29.64_in};// bottom
constexpr frc::Translation2d MidfieldNote4{ 250.50_in, 95.64_in};
constexpr frc::Translation2d MidfieldNote3{ 250.50_in,  161.64_in};
constexpr frc::Translation2d MidfieldNote2{ 250.50_in,  227.64_in};
constexpr frc::Translation2d MidfieldNote1{ 250.50_in,  293.64_in};//top