// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include "LoggingLevel.h"

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

class LimeLight {
 
 
  std::shared_ptr<nt::NetworkTable> m_limelight;
  

public:
  LimeLight(std::string name);



  bool IsTargetVisible();

  frc::Pose2d GetRobotPose();

  void SendData(std::string name, LoggingLevel verbose);

  units::inch_t GetReflectiveTargetRange(double targetHight);


};
