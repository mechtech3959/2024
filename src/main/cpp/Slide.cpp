#include "Slide.h"



Slide::Slide(){

    configDevices();
    Init();

}

void Slide::Init(){
    //set motors and encoders
    
    m_motor.SetSelectedSensorPosition(0);
    


}

void Slide::configDevices(){
    m_motor.ConfigFactoryDefault();
    
    TalonFXConfiguration config;

    config.clearPositionOnLimitR = true;
    config.forwardLimitSwitchSource = LimitSwitchSource_FeedbackConnector;
    config.forwardLimitSwitchNormal = LimitSwitchNormal_NormallyOpen;
    config.reverseLimitSwitchSource = LimitSwitchSource_FeedbackConnector;
    config.reverseLimitSwitchNormal = LimitSwitchNormal_NormallyOpen;

    
    

    config.slot0.kP = .04;
    config.slot0.kF = .05;
    config.slot0.allowableClosedloopError = 50.0;

    double inchesPerSec = 20.0; 


    config.motionCruiseVelocity = (constants::slideConstants::EncoderTicksPerInch*inchesPerSec)/10.0;
    config.motionAcceleration = 5.0*config.motionCruiseVelocity; // 1/6 sec for arm to achieve cruising velocity
    config.motionCurveStrength = 7;

    m_motor.ConfigAllSettings(config);
    m_motor.SetInverted(TalonFXInvertType::Clockwise);

/*
    m_encoder.ConfigFactoryDefault();

    CANCoderConfiguration eConfig;

    eConfig.magnetOffsetDegrees = 0.0;
    m_encoder.ConfigAllSettings(eConfig);
*/
}

void Slide::SetPosition(units::inch_t position){
    //sets position
    if(position<0_in){
        position = 0_in;
    }
    if(position>18_in){
        position = 18_in;
    }
    m_target = position;
    m_motor.Set(ControlMode::MotionMagic, constants::slideConstants::EncoderTicksPerInch*position.value());
}

void Slide::SetSpeed(double speed){
    //sets the speed of the motor
    m_motor.Set(ControlMode::PercentOutput, speed);
}

units::inch_t Slide::GetPosition(){
    return units::inch_t(m_motor.GetSelectedSensorPosition()/constants::slideConstants::EncoderTicksPerInch);
    //returns position
}


void Slide::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
            {
                frc::SmartDashboard::PutNumber("raw Slide position", m_motor.GetSelectedSensorPosition());
            }
                                    //continue
        case LoggingLevel::PID: //send PID (closed loop control) data
            {
                frc::SmartDashboard::PutNumber("Slide Error", ctreHelpers::CTRE_Get_PID_Error(m_motor));
                frc::SmartDashboard::PutNumber("Slide Target", ctreHelpers::CTRE_Get_PID_Target(m_motor));
                frc::SmartDashboard::PutNumber("Slide PID Output%", m_motor.GetMotorOutputPercent());
            }
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
            {
                frc::SmartDashboard::PutNumber("Slide position (in)", GetPosition().value());
            }
                                    //continue
        default: break; //make sure nothing else prints
        

    }
}