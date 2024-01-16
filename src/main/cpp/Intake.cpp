// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake(){
    Init();
}

void Intake::Init(){
    //sets motors and encoders
    m_motorIntake.RestoreFactoryDefaults();

    /*(int constants::IntakeConstants::MotorID,
                       constants::IntakeConstants::CANBus)*/

    /*(int constants::IntakeConstants::MotorID,
                          constants::IntakeConstants::CANBus)*/

    // initialize motors here or something

    bool dir = true;
    m_motorIntake.SetInverted(dir);
}

void Intake::SetIntakeSpeed(double Speed){
  // sets intake motor speed
    m_motorIntake.Set(Speed);
}


void Intake::IntakeDown(){
    m_clawPistion.Set(true);
}

void Intake::IntakeUp(){
    m_clawPistion.Set(false);
}
void Claw::ClawToggle(){
    m_clawPistion.Set(!m_clawPistion.Get());
}


void Intake::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
                                    //continue
                                    {
                                        frc::SmartDashboard::PutNumber("Intake position", m_clawPistion.Get());
                                        frc::SmartDashboard::PutNumber("Intake speed", m_motorIntake.Get());
                                    }
        case LoggingLevel::PID: //send PID (closed loop control) data
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
                                    //continue
        default: break; //make sure nothing else prints
        

    }
}
