#include "Elevator.h" //header file


Elevator::Elevator()
{
    //constructor- declaring constants for elevator
    configDevices();
    Init();

}


void Elevator::Init(){
//initialization for the code

   
    //zeroing the sensors
    m_motor1.SetSelectedSensorPosition(0);
   
}
void Elevator::configDevices(){
    m_motor1.ConfigFactoryDefault();
    m_motor2.ConfigFactoryDefault();
    
    TalonFXConfiguration config;

    config.clearPositionOnLimitR = true;
    config.forwardLimitSwitchSource = LimitSwitchSource_FeedbackConnector;
    config.forwardLimitSwitchNormal = LimitSwitchNormal_NormallyOpen;
    config.reverseLimitSwitchSource = LimitSwitchSource_FeedbackConnector;
    config.reverseLimitSwitchNormal = LimitSwitchNormal_NormallyOpen;
    

    config.slot0.kP = .06;
    config.slot0.kF = .05;

    double inchesPerSec = 20.0; 

    config.motionCruiseVelocity = (inchesPerSec / 10.0) * (2048.0 * constants::elevatorConstants::MotorGearRatio);// inchs/100ms*ticks per inch
    config.motionAcceleration = 5.0*config.motionCruiseVelocity; // 1 sec for arm to achieve cruising velocity



    m_motor1.ConfigAllSettings(config);

    //inverting motor
    m_motor1.SetInverted(TalonFXInvertType::Clockwise);    
    m_motor2.SetInverted(m_motor1.GetInverted());

    m_motor2.Follow(m_motor1);

/*
    m_encoder.ConfigFactoryDefault();

    CANCoderConfiguration eConfig;

    eConfig.magnetOffsetDegrees = 0.0;
    m_encoder.ConfigAllSettings(eConfig);
*/
}
      

void Elevator::SetSpeed(double rawMotorSpeed){
    m_motor1.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed);
    
} 
void Elevator::SetHeight(units::inch_t pos){
    

    m_motor1.Set(ControlMode::MotionMagic, pos.value()*constants::elevatorConstants::EncoderTicksPerInch);

}

units::inch_t Elevator::GetHeight(){
    return units::inch_t(m_motor1.GetSelectedSensorPosition()/constants::elevatorConstants::EncoderTicksPerInch);;
}

void Elevator::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
            {
                frc::SmartDashboard::PutNumber("Elevator Raw position", m_motor1.GetSelectedSensorPosition());
            }
                                    //continue
        case LoggingLevel::PID: //send PID (closed loop control) data
            {
                frc::SmartDashboard::PutNumber("Elevator Error", ctreHelpers::CTRE_Get_PID_Error(m_motor1));
                frc::SmartDashboard::PutNumber("Elevator Target", ctreHelpers::CTRE_Get_PID_Target(m_motor1));
                frc::SmartDashboard::PutNumber("Elevator PID Output%", m_motor1.GetMotorOutputPercent());
            }
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
            {
                frc::SmartDashboard::PutNumber("Elevator Height (in)", GetHeight().value());
            }
                                    //continue
        default: break; //make sure nothing else prints
        

    }
}
