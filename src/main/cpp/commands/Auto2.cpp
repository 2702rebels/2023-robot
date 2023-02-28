#include "commands/Auto2.h"

#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto2::Auto2(DriveSubsystem* drive) {
  AddCommands(
    DriveTrajectoryCommand(
      {
        frc::Pose2d { 0_m, 0_m, 0_deg },
        frc::Pose2d { 1_m, 0_m, 0_deg },
        frc::Pose2d { 2_m, 0_m, 0_deg },
        frc::Pose2d { 3_m, 0_m, 0_deg },
        frc::Pose2d { 4_m, 0_m, 0_deg },
      },
      frc::Rotation2d { 0_deg },
      false,
      true,
      drive),
    DriveTrajectoryCommand(
      {
        frc::Pose2d { 4_m, 0_m, 0_deg },
        frc::Pose2d { 3_m, 0_m, 0_deg },
        frc::Pose2d { 2_m, 0_m, 0_deg },
        frc::Pose2d { 1_m, 0_m, 0_deg },
        frc::Pose2d { 0_m, 0_m, 0_deg },
      },
      frc::Rotation2d { 180_deg },
      true,  // robot is driving backwards, but will rotate
      false,
      drive));
}
