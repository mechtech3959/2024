// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/Encoder.h>
#include <frc/RobotController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class Drive : public frc2::SubsystemBase {
public:
  Drive();

  frc2::CommandPtr ArcadeDriveCommand(std::function<double()> fwd,
                                      std::function<double()> rot);
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

private:
  rev::CANSparkMax m_leftMotor{constants::drive::kLeftMotor1Port,
                               rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightMotor{constants::drive::kRightMotor1Port,
                                rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftMotorFollower{constants::drive::kLeftMotor2Port,
                                       rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightMotorFollower{constants::drive::kRightMotor2Port,
                                        rev::CANSparkMax::MotorType::kBrushed};

  frc::DifferentialDrive m_drive{[this](auto val) { m_leftMotor.Set(val); },
                                 [this](auto val) { m_rightMotor.Set(val); }};

  ctre::phoenix6::hardware::CANcoder m_leftEncoder{
      constants::drive::kLeftEncoderPorts};

  ctre::phoenix6::hardware::CANcoder m_rightEncoder{
      constants::drive::kRightEncoderPorts};

  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{3_V / 1_s, std::nullopt, 4_s, std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            m_leftMotor.SetVoltage(driveVoltage);
            m_rightMotor.SetVoltage(-driveVoltage);
          },
          [this](frc::sysid::SysIdRoutineLog *log) {
            log->Motor("drive-left")
                .voltage((m_leftMotor.Get() + m_leftMotorFollower.Get()) / 2 *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{
                    m_leftEncoder.GetPosition().GetValueAsDouble() /
                    constants::drive::kRotPerM})
                .velocity(units::meters_per_second_t{
                    m_leftEncoder.GetVelocity().GetValueAsDouble() /
                    constants::drive::kRotPerM});
            log->Motor("drive-right")
                .voltage((m_rightMotor.Get() + m_rightMotorFollower.Get()) / 2 *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{
                    -m_rightEncoder.GetPosition().GetValueAsDouble() /
                    constants::drive::kRotPerM})
                .velocity(units::meters_per_second_t{
                    -m_rightEncoder.GetVelocity().GetValueAsDouble() /
                    constants::drive::kRotPerM});
          },
          this}};
};
