#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <memory>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

using namespace pathplanner;

RobotContainer::RobotContainer() {}
    /// Returns a CommandPtr to the selected autonomous routine
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
          std::unique_ptr<frc2::Command>{chooser.GetSelected()};

        // Load the path you want to follow using its name in the GUI
    }

    void RobotContainer::GetAutonomousPos() {
      frc::Pose2d pathstartpose = PathPlannerAuto::getStartingPoseFromAutoFile(
          chooser.GetSelected()->GetName());

      m_swerve.SetPose(pathstartpose);
}
