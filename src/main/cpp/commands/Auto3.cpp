#include "commands/Auto3.h"

#include "commands/DriveRotateCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto3::Auto3(DriveSubsystem* drive, ExtendingArmSubsystem* arm) {
  AddCommands(
    frc2::InstantCommand([arm] { arm->MoveArm(ExtendingArmPositions::SCORE_MID_CONE); }, { arm }),
    DriveTrajectoryCommand(
      {
        frc::Pose2d { 0_m, 0_m, 180_deg },
        frc::Pose2d { 0.2_m, 0_m, 180_deg },
        frc::Pose2d { 0.4_m, 0_m, 180_deg },
      },
      frc::Rotation2d { 180_deg },
      false,
      true,
      drive),
    frc2::InstantCommand([arm] { arm->ScoreCone(); }, { arm }),
    DriveRotateCommand(frc::Rotation2d { 0_deg }, drive),
    DriveTrajectoryCommand(
      {
        frc::Pose2d { 0_m, 0_m, 0_deg },
        frc::Pose2d { 1_m, 0_m, 0_deg },
        frc::Pose2d { 2_m, 0_m, 0_deg },
      },
      frc::Rotation2d { 0_deg },
      false,
      false,
      drive));
}
