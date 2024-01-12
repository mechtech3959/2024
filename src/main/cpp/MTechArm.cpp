#include "MTechArm.h" //header file

Arm::Arm() {
  // constructor- declaring constants for arm
  configDevices();
  Init();
}

void Arm::Init() {
  // initialization for the code
  m_motor2.Follow(m_motor1); // slave follow

  // inverting motor
  m_motor1.SetInverted(TalonFXInvertType::Clockwise);
  m_motor2.SetInverted(!m_motor1.GetInverted());

  // zeroing the sensors
  m_motor1.SetSelectedSensorPosition(m_encoder.GetAbsolutePosition() *
                                     constants::armConstants::TicksPerDegree);
}

void Arm::configDevices() {
  // config for motors and encoders
  // resets
  m_motor1.ConfigFactoryDefault();
  m_motor2.ConfigFactoryDefault();
  m_encoder.ConfigFactoryDefault();

  TalonFXConfiguration config;

  config.forwardSoftLimitThreshold =
      140.0 * constants::armConstants::TicksPerDegree;
  config.reverseSoftLimitThreshold =
      -135.0 * constants::armConstants::TicksPerDegree;
  config.forwardSoftLimitEnable = false;
  config.reverseSoftLimitEnable = false;

  config.slot0.kP = .01;
  config.slot0.kF = .05;

  double degreesPerSec = 150.0;

  config.motionCruiseVelocity =
      (degreesPerSec / 10.0) * constants::armConstants::TicksPerDegree;
  config.motionAcceleration =
      4.0 *
      config.motionCruiseVelocity; // 1 sec for arm to achieve cruising velocity
  config.motionCurveStrength = 2;

  // config.Motor1Config.initializationStrategy =
  // phoenix::sensors::SensorInitializationStrategy::BootToZero;
  m_motor1.ConfigAllSettings(config);
  // m_motor2.ConfigAllSettings(config);

  CANCoderConfiguration eConfig;

  eConfig.magnetOffsetDegrees = -57.0;
  eConfig.absoluteSensorRange = AbsoluteSensorRange::Signed_PlusMinus180;
  m_encoder.ConfigAllSettings(eConfig);
}

void Arm::SetAngle(units::degree_t goal) {

  if (goal > 130_deg) {
    goal = 130_deg;
  }
  if (goal < -135_deg) {
    goal = -135_deg;
  }
  double turnTarget =
      goal.value() *
      constants::armConstants::TicksPerDegree; // how many ticks away the goal
                                               // is

  double rads = units::radian_t(goal).value();
  double arbFF = 0.09 * std::cos(rads);
  m_motor1.Set(
      motorcontrol::ControlMode::MotionMagic, turnTarget,
      DemandType::DemandType_ArbitraryFeedForward,
      arbFF); // tells motor controller to go to turnTarget with MotionMagic
  // closed loop set point function
}

void Arm::SetSpeed(double rawMotorSpeed) {
  m_motor1.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed);
}

units::degree_t Arm::GetOffsetAngle() {
  double encoderRaw =
      m_encoder.ConfigGetParameter(ParamEnum::eMagnetOffset, 0); // offset angle
  units::degree_t angle(encoderRaw); // convert to degrees
  return angle;
}

units::degree_t Arm::GetAngle() {
  double encoderRaw =
      m_encoder.GetAbsolutePosition(); // gets the angle the arm is currently at
  units::degree_t angle(encoderRaw);   // convert to degrees
  return angle;
}

double Arm::GetRawAngle() {
  double encoderRaw =
      m_encoder.GetAbsolutePosition(); // gets the angle without being zeroed
                                       // out - encoder ticks
  return encoderRaw;
}

void Arm::SendData(LoggingLevel verbose) {
  // sends data to dashboard with the enum LoggingLevel
  switch (verbose) {
  case LoggingLevel::Everything: // everything that is not in the cases below it
    // continue
    {
      frc::SmartDashboard::PutNumber("Arm raw speed",
                                     m_motor1.GetSelectedSensorVelocity());
      frc::SmartDashboard::PutNumber("Arm raw position",
                                     m_motor1.GetSelectedSensorPosition());
      frc::SmartDashboard::PutNumber(
          "Arm Estimated Raw position",
          m_encoder.GetAbsolutePosition() *
              constants::armConstants::TicksPerDegree);
      frc::SmartDashboard::PutNumber(
          "Arm Soft Lim Rev", -135.0 * constants::armConstants::TicksPerDegree);
      frc::SmartDashboard::PutNumber(
          "Arm Soft Lim For", 225.0 * constants::armConstants::TicksPerDegree);
    }
  case LoggingLevel::PID: // send PID (closed loop control) data
    // continue
    {
      frc::SmartDashboard::PutNumber(
          "Arm raw turn pid target",
          ctreHelpers::CTRE_Get_PID_Target(m_motor1)); // target of motor
      frc::SmartDashboard::PutNumber(
          "Arm raw turn pid error",
          ctreHelpers::CTRE_Get_PID_Error(m_motor1)); // error
      frc::SmartDashboard::PutNumber("Arm Output %",
                                     m_motor1.GetMotorOutputPercent()); // error
    }
    // continue
  case LoggingLevel::Basic: // minimal useful data to driver
    // continue
    { frc::SmartDashboard::PutNumber("Arm Angle", GetAngle().value()); }
  default:
    break; // make sure nothing else prints
  }
}
