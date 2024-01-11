#pragma once

#include <map>
#include "WaypointPoses.h"

namespace autoSelector{
    enum ScorePosition{
        Grid1Pos1,
        Grid2Pos1,
        Grid3Pos1,
        Grid1Pos2,
        Grid2Pos2,
        Grid3Pos2,
        Grid1Pos3,
        Grid2Pos3,
        Grid3Pos3,
        Switch
    };
    
    class ScorePositionPose{
        public:
        const std::string name;
        const frc::Pose2d bluePose;
        const frc::Pose2d redPose;

        ScorePositionPose(std::string id, frc::Pose2d blue, frc::Pose2d red): name(id),bluePose(blue), redPose(red)
        {}
    };
    /*
    constexpr std::map<ScorePosition, ScorePositionPose> ScorePositionMap{
        ScorePosition::Grid1Pos1, ScorePositionPose("Grid 1 Left Cone", waypoints::Blue8Left, waypoints::Red1Left)
    };
*/
    enum ScorePositionHeight{
        Low,
        Med,
        High
    };



}