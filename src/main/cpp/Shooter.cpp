// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 #include "Shooter.h"

Shooter::Shooter(){
    Init();
}

void Shooter::Init(){
    //sets motors and encoders
    m_motorShooter.RestoreFactoryDefaults();
    
    m_motorShooter.SetInverted(true);
}


void Shooter::SetShooterSpeed(double Speed){
    //sets Shooter motor speed
    m_motorShooter.Set(Speed);
}


void Shooter::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
                                    //continue
                                    {
                                        frc::SmartDashboard::PutNumber("Shooter speed", m_motorShooter.Get());
                                    }
        case LoggingLevel::PID: //send PID (closed loop control) data
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
                                    //continue
        default: break; //make sure nothing else prints
        

    }
}
