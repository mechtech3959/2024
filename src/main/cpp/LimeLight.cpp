#include "LimeLight.h"

LimeLight::LimeLight(std::string name) {
  m_limelight = nt::NetworkTableInstance::GetDefault().GetTable(name);
}

bool LimeLight::IsTargetVisible() {

  double tv = m_limelight->GetNumber("tv", 0.0);
  if (tv == 0.0) {
    return false;
  } else {
    return true;
  }

  return false;
}

frc::Pose2d LimeLight::GetRobotPose() {

  // create storage vector for bot pose from limelight
  std::vector<double> data =
      m_limelight->GetNumberArray("botpose_wpiblue", std::array<double, 7>{});
  // get 3d pose from limelight

  if (IsTargetVisible()) {
    // translate 3d pose to 2d pose
    units::meter_t x{data.at(0)};
    units::meter_t y{data.at(1)};
    units::degree_t heading{data.at(5)};

    return frc::Pose2d(x, y, frc::Rotation2d(heading));
  } else {
    return frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
  }
}

units::inch_t LimeLight::GetReflectiveTargetRange(double targetHeight) {
  /*
  double targetOffsetAngle_Horizontal = m_limelight->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = m_limelight->GetNumber("ty",0.0);
  double targetArea = m_limelight->GetNumber("ta",0.0);
  double targetSkew = m_limelight->GetNumber("ts",0.0);

  if(IsTargetVisable()){
    double targetOffsetAngle_Vertical = m_limelight->GetNumber("ty", 0.0);
    double targetOffsetAngle_VerticalLen = m_limelight->GetNumber("tvert",
  0.0)/2.0; targetOffsetAngle_Vertical =
  targetOffsetAngle_Vertical+targetOffsetAngle_VerticalLen;
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightHeightInches = 20.0;

    // distance from the target to the floor
    double goalHeightInches = targetHeight;

    double angleToGoalDegrees = limelightMountAngleDegrees +
  targetOffsetAngle_Vertical; double angleToGoalRadians = angleToGoalDegrees *
  (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches -
  limelightHeightInches)/tan(angleToGoalRadians);


    return units::inch_t(distanceFromLimelightToGoalInches);
  }else{
    return 0_in; //GetEstimatedRangeToTarget();
  }
  */
  return 0_in;
}

void LimeLight::SendData(std::string name, LoggingLevel verbose) {
  switch (verbose) {
  case LoggingLevel::Everything: // everything that is not in the cases below it
                                 // continue
  case LoggingLevel::PID:        // send PID (closed loop control) data
                                 // continue
  case LoggingLevel::Basic:      // minimal useful data to driver
  {
    frc::Pose2d p = GetRobotPose();
    frc::SmartDashboard::PutBoolean(name + " visible", IsTargetVisible());
    frc::SmartDashboard::PutNumber(name + " pose x",
                                   units::inch_t(p.X()).value());
    frc::SmartDashboard::PutNumber(name + " pose y",
                                   units::inch_t(p.Y()).value());
    frc::SmartDashboard::PutNumber(name + " pose heading",
                                   p.Rotation().Degrees().value());
  }
    // continue
  default:
    break; // make sure nothing else prints
  }
}
