#include <ctre/Phoenix.h>
#include <ctre/phoenix6/controls/Follower.hpp>

TalonFX ShooterM{5};
TalonFX ShooterS{6};

void SFollow() { ShooterS.Follow(ShooterM); };
void SpeakerS() {
  ShooterM.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
               85); // I SET THIS TO A RANDOM NUMBER
};
void AmpS() {
  ShooterM.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
               15); // I SET THIS TO A RANDOM NUMBER
};
