#pragma once

#include "LimelightHelpers.h"
#include "LoggingLevel.h"
#include "networktables/NetworkTable.h"
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <units/length.h>

class LimeLight {
public:
  LimeLight(std::string name);

  bool IsTargetVisible();

  frc::Pose2d GetRobotPose();

  void SendData(std::string name, LoggingLevel verbose);

  units::inch_t GetReflectiveTargetRange(double targetHeight);

  void updateTracking();
  void updateBackTracking();

  double aprilTagID;
  double Note;

  bool m_LimelightHasTarget;
  double m_LimelightTurnCmd;
  double m_LimelightDriveCmd;

  units::inch_t Distance();

private:
  std::shared_ptr<nt::NetworkTable> m_limelight;

  double clamp(double in, double minval, double maxval) {
    if (in > maxval)
      return maxval;
    if (in < minval)
      return minval;
    return in;
  };
};

/*
class LL3DPose{

public:
 LL3DPose(std::vector<double> p3d)
 {
  pose3d = p3d;
 }

  frc::Pose2d GetPose2d(){
    if(isVisable()){
      return frc::Pose2d(  units::meter_t{pose3d.at(0)},
                            units::meter_t{pose3d.at(1)},
                            frc::Rotation2d{units::degree_t{pose3d.at(5)}}
                          );
    }else{
      return frc::Pose2d();
    }
  };

  bool isVisable(){
    if(pose3d.size()>=6){
      return true;
    }else{
      return false;
    }
  };

  std::vector<double> pose3d;

};
*/
