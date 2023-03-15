#include "commands/auto/Auto_DriveDistance.h"

#include "commands/DriveRotateCommand.h"
#include "commands/DriveTrajectoryCommand.h"

Auto_DriveDistance::Auto_DriveDistance(DriveSubsystem* drive) {
  AddCommands(DriveTrajectoryCommand(
    {
      frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Pose2d { 1_m, 0_m, 0_deg },
      //  frc::Pose2d { 2_m, 0_m, 0_deg },
      // frc::Pose2d { 3_m, 0_m, 0_deg },
    },
    frc::Rotation2d { 0_deg },
    false,
    true,
    drive));
  // DriveRotateCommand(frc::Rotation2d { 180_deg }, drive),
  // DriveTrajectoryCommand(
  //   {
  //     frc::Pose2d { 4_m, 0_m, 180_deg },
  //     frc::Pose2d { 3_m, 0_m, 180_deg },
  //     frc::Pose2d { 2_m, 0_m, 180_deg },
  //     frc::Pose2d { 1_m, 0_m, 180_deg },
  //     frc::Pose2d { 0_m, 0_m, 180_deg },
  //   },
  //   frc::Rotation2d { 180_deg },
  //   false,
  //   false,
  //   drive));
}
