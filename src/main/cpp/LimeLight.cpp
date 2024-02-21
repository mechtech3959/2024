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
void LimeLight::Update_Limelight_Tracking() {
  // Proportional Steering Constant:
  // If your robot doesn't turn fast enough toward the target, make
  // this number bigger If your robot oscillates (swings back and
  // forth past the target) make this smaller
  const double STEER_K = 0.04;

  // Proportional Drive constant: bigger = faster drive
  const double DRIVE_K = 0.26;

  // Area of the target when your robot has reached the goal
  const double DESIRED_TARGET_AREA = 4;
  const double MAX_DRIVE = 1.0;
  const double MAX_STEER = 0.5f;

  double tx = LimelightHelpers::getTX("limelight-greenie");
  double ty = LimelightHelpers::getTY("limelight-greenie");
  double ta = LimelightHelpers::getTA("limelight-greenie");
  double tv = LimelightHelpers::getTV("limelight-greenie");

  if (tv < 1.0) {
    m_LimelightHasTarget = false;
    m_LimelightDriveCmd = 0.0;
    m_LimelightTurnCmd = 0.0;
  } else {
    m_LimelightHasTarget = true;

    // Proportional steering
    m_LimelightTurnCmd = tx * STEER_K;
    m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER, MAX_STEER);

    // drive forward until the target area reaches our desired area
    m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    m_LimelightDriveCmd = clamp(m_LimelightDriveCmd, -MAX_DRIVE, MAX_DRIVE);
  }
}

void LimeLight::AmpAuto() {
  autotimer.Start();
  Update_Limelight_Tracking();
  double ID = LimelightHelpers::getFiducialID("limelight-greenie");
  if (autotimer.Get() < 5_s) {
    if (ID == 6 || ID == 5)
      drive.diffDrive.ArcadeDrive(m_LimelightDriveCmd, m_LimelightTurnCmd);
  } else {
    if (autotimer.Get() < 9_s && autotimer.Get() > 5_s) {
      drive.diffDrive.ArcadeDrive(0.0, 0.0);
      // shoot l8r
    }
    if (autotimer.Get() > 10_s) {
      drive.diffDrive.ArcadeDrive(-0.5, 0.6);
    }
  }
  autotimer.Stop();
  /*while (autotimer.Get() < 1.0_s) {
  drive.diffDrive.ArcadeDrive(-0.6, 0.0);
}
while (autotimer.Get() < 1.7_s) {
  drive.diffDrive.ArcadeDrive(0.0, 0.6);
}
while (autotimer.Get() < 15.0_s) {
  Update_Limelight_Tracking();
  drive.diffDrive.ArcadeDrive(0.4, m_LimelightTurnCmd);
}*/
}
