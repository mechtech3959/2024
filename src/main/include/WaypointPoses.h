
#pragma once


#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>

#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/dimensionless.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

#include "Constants.h"

namespace waypoints{

constexpr units::inch_t RedTapeX = 610.77_in - 14.25_in;
constexpr units::inch_t BlueTapeX = 40.45_in + 14.25_in+4_in;
constexpr units::inch_t TapeOffset = 0_in;
constexpr units::inch_t Grid3Y = 174.19_in;
constexpr units::inch_t Grid2Y = 108.19_in;
constexpr units::inch_t Grid1Y = 42.19_in;

constexpr units::inch_t BlueYOffSetLeft = 4_in;
constexpr units::inch_t RedYOffSetLeft = 0_in;

constexpr units::degree_t RedHeading = 0_deg;
constexpr units::degree_t BlueHeading = 179.9_deg;


constexpr frc::Pose2d Red1Right{  RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid1Y - 22_in, 
                            frc::Rotation2d(RedHeading)
                          };
constexpr frc::Pose2d Red1Center{ RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid1Y, 
                            frc::Rotation2d(RedHeading)
                          };
constexpr frc::Pose2d Red1Left{ RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                          Grid1Y + 22_in, 
                          frc::Rotation2d(RedHeading)
                        };

constexpr frc::Pose2d Red2Right{  RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid2Y - 22_in, 
                            frc::Rotation2d(RedHeading)
                          };
constexpr frc::Pose2d Red2Center{ RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid2Y, 
                            frc::Rotation2d(RedHeading)
                          };
constexpr frc::Pose2d Red2Left{ RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                          Grid2Y + 22_in, 
                          frc::Rotation2d(RedHeading)
                        };
    
constexpr frc::Pose2d Red3Right{  RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid3Y - 22_in, 
                            frc::Rotation2d(RedHeading)
                          };
constexpr frc::Pose2d Red3Center{ RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid3Y, 
                            frc::Rotation2d(RedHeading)
                          };
constexpr frc::Pose2d Red3Left{ RedTapeX - TapeOffset - constants::swerveConstants::WheelBaseLength/2.0, 
                          Grid3Y + 22_in, 
                          frc::Rotation2d(RedHeading)
                        };

constexpr frc::Pose2d Blue8Right{ BlueTapeX + TapeOffset + constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid1Y + 22_in + 4_in, 
                            frc::Rotation2d(BlueHeading)
                          };
constexpr frc::Pose2d Blue8Center{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid1Y, 
                            frc::Rotation2d(BlueHeading)
                          };
constexpr frc::Pose2d Blue8Left{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                          Grid1Y - 22_in, 
                          frc::Rotation2d(BlueHeading)
                        };

constexpr frc::Pose2d Blue7Right{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid2Y + 22_in + 4_in, 
                            frc::Rotation2d(BlueHeading)
                          };
constexpr frc::Pose2d Blue7Center{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid2Y, 
                            frc::Rotation2d(BlueHeading)
                          };
constexpr frc::Pose2d Blue7Left{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                          Grid2Y - 22_in, 
                          frc::Rotation2d(BlueHeading)
                        };
    
constexpr frc::Pose2d Blue6Right{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid3Y + 22_in, 
                            frc::Rotation2d(BlueHeading)
                          };
constexpr frc::Pose2d Blue6Center{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                            Grid3Y, 
                            frc::Rotation2d(BlueHeading)
                          };
constexpr frc::Pose2d Blue6Left{ BlueTapeX + TapeOffset  + constants::swerveConstants::WheelBaseLength/2.0, 
                          Grid3Y - 22_in, 
                          frc::Rotation2d(BlueHeading)
                        };
    

constexpr frc::Translation2d BlueCorridorNear{ BlueTapeX + 50_in, 195_in}; //waypoing near scoring zone for the flat corridor to game pieces
constexpr frc::Translation2d BlueCorridorFar{  220_in, 195_in}; //waypoing near game pieces for the flat corridor to game pieces
constexpr frc::Translation2d BlueBumpNear{ BlueTapeX + 40_in, 30_in};     //waypoing near scoring zone for the Speed Bump to game pieces
constexpr frc::Translation2d BlueBumpFar{  220_in, 30_in};     //waypoing near game pieces for the Speed Bump to game pieces

constexpr frc::Translation2d BlueSwitchNear{ BlueTapeX + 30_in, 107.39_in};     //waypoing near alliance wall for switch
constexpr frc::Translation2d BlueSwitch{     BlueTapeX + 96.75_in - 2_in, 107.39_in}; //Center of switch
constexpr frc::Translation2d BlueSwitchFar{  BlueTapeX + 162_in, 107.39_in};     //waypoing Far from alliance wall for switch

constexpr frc::Translation2d BluePiece1{ BlueTapeX + 224_in, 180.19_in-15_in};  //farthest from wall
constexpr frc::Translation2d BluePiece2{ BlueTapeX + 224_in, 132.19_in};
constexpr frc::Translation2d BluePiece3{ BlueTapeX + 224_in,  84.19_in};
constexpr frc::Translation2d BluePiece4{ BlueTapeX + 224_in,  36.19_in};  //nearest to wall

constexpr frc::Translation2d RedCorridorNear{ RedTapeX - 40_in, 185_in}; //waypoing near scoring zone for the flat corridor to game pieces
constexpr frc::Translation2d RedCorridorFar{  433_in, 185_in};   //waypoing near game pieces for the flat corridor to game pieces
constexpr frc::Translation2d RedBumpNear{ RedTapeX - 40_in, 30_in};    //waypoing near scoring zone for the Speed Bump to game pieces
constexpr frc::Translation2d RedBumpFar{  433_in, 30_in};      //waypoing near game pieces for the Speed Bump to game pieces
    
constexpr frc::Translation2d RedSwitchNear{ RedTapeX - 30_in, 107.39_in};     //waypoing near alliance wall for switch
constexpr frc::Translation2d RedSwitch{     RedTapeX - 96.75_in, 107.39_in}; //Center of switch
constexpr frc::Translation2d RedSwitchFar{  RedTapeX - 162_in, 107.39_in};     //waypoing Far from alliance wall for switch

constexpr frc::Translation2d RedPiece1{ RedTapeX - 224_in, 180.19_in};
constexpr frc::Translation2d RedPiece2{ RedTapeX - 224_in, 132.19_in};
constexpr frc::Translation2d RedPiece3{ RedTapeX - 224_in,  84.19_in};
constexpr frc::Translation2d RedPiece4{ RedTapeX - 224_in,  36.19_in};


class WaypointPoses{
  public:
    WaypointPoses(){
      //Bottom Left, Top Right, speed limit
      frc::RectangularRegionConstraint slowRegionRedSwitch{frc::Translation2d{472_in, 59_in},
                                                  frc::Translation2d{537_in, 155_in},
                                                  frc::MaxVelocityConstraint{.55_mps}};

      frc::RectangularRegionConstraint slowRegionRedBump{frc::Translation2d{491_in, 0_in},
                                                  frc::Translation2d{505_in, 59_in},
                                                  frc::MaxVelocityConstraint{.75_mps}};

      frc::RectangularRegionConstraint slowRegionBlueSwitch{frc::Translation2d{113_in, 59_in},
                                                  frc::Translation2d{220_in, 155_in},
                                                  frc::MaxVelocityConstraint{.55_mps}};

      frc::RectangularRegionConstraint slowRegionBlueBump{frc::Translation2d{145_in, 0_in},
                                                  frc::Translation2d{155_in, 59_in},
                                                  frc::MaxVelocityConstraint{.75_mps}};
      
      config.AddConstraint(slowRegionRedSwitch);
      config.AddConstraint(slowRegionRedBump);
      config.AddConstraint(slowRegionBlueSwitch);
      config.AddConstraint(slowRegionBlueBump);
    }
    //configure traj with speed and acceleration 
    frc::TrajectoryConfig config{ constants::swerveConstants::MaxSpeed*.5, 
                                  constants::swerveConstants::MaxAcceleration*.35};
    
    

            
};

}
