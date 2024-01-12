// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LimeLight.h"

LimeLight::LimeLight() {


}

bool LimeLight::IsTargetVisable(){
    
    double tv = m_limelight->GetNumber("tv", 0.0);
    if(tv== 0.0){
      return false;
    }else{
      return true;
    }
    
   return false;
  }

  frc::Pose2d LimeLight::GetRobotPose(){

    //create storage vector for bot pose from limelight
    LL3DPose pose3d{m_limelight->GetNumberArray("botpose",std::array<double, 6>{} )};
    //get 3d pose from limelight
    return pose3d.GetPose2d();
    
    //translate 3d pose to 2d pose
    units::meter_t x{pose3d.at(0)};
    units::meter_t y{pose3d.at(1)};
    units::degree_t heading{pose3d.at(5)};

    return frc::Pose2d(x,y,frc::Rotation2d(heading));
  }

  units::inch_t LimeLight::GetReflectiveTargetRange(double targetHight){
    
    double targetOffsetAngle_Horizontal = m_limelight->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = m_limelight->GetNumber("ty",0.0);
    double targetArea = m_limelight->GetNumber("ta",0.0);
    double targetSkew = m_limelight->GetNumber("ts",0.0);

    if(IsTargetVisable()){
      double targetOffsetAngle_Vertical = m_limelight->GetNumber("ty", 0.0);
      double targetOffsetAngle_VerticalLen = m_limelight->GetNumber("tvert", 0.0)/2.0;
      targetOffsetAngle_Vertical = targetOffsetAngle_Vertical+targetOffsetAngle_VerticalLen;
      // how many degrees back is your limelight rotated from perfectly vertical?
      double limelightMountAngleDegrees = 0.0;

      // distance from the center of the Limelight lens to the floor
      double limelightHeightInches = 20.0;

      // distance from the target to the floor
      double goalHeightInches = targetHight;

      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      //calculate distance
      double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/tan(angleToGoalRadians);


      return units::inch_t(distanceFromLimelightToGoalInches);
    }/*else{
       return 0_in; GetEstimatedRangeToTarget();
       }*/
    return 0_in;
  }

  void LimeLight::SendData(LoggingLevel verbose){
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
                                    //continue
        case LoggingLevel::PID: //send PID (closed loop control) data
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
              {
                frc::Pose2d p = GetRobotPose();
                frc::SmartDashboard::PutBoolean("Limelight visable x", IsTargetVisable());
                frc::SmartDashboard::PutNumber("Limelight pose x", p.X().value());
                frc::SmartDashboard::PutNumber("Limelight pose y", p.Y().value());
                frc::SmartDashboard::PutNumber("Limelight pose heading", p.Rotation().Degrees().value());
                
              }
                                    //continue
        default: break; //make sure nothing else prints
        

    }
  }

