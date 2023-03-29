#include "commands/auto/Auto2.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/DriveToPoseCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto initialPose = frc::Pose2d { 0_m, 0_m, 0_deg };
constexpr auto targetPose = frc::Pose2d { 2_m, 1_m, 90_deg };

Auto2::Auto2(DriveSubsystem* drive) {
  AddCommands(
    frc2::InstantCommand([drive] { drive->ResetOdometry(initialPose); }, {}),
    DriveTrajectoryCommand(
      {
        Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
        Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }, 2.5_mps),
        Waypoint(frc::Pose2d { 3.5_m, 0_m, 0_deg }, 1_mps),
        Waypoint(frc::Pose2d { 4.8_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
      },
      initialPose.Translation(),
      true,   // reset odometry
      false,  // timer-based stop
      true,   // print trajectory
      drive));
  // DriveToPoseCommand(targetPose, drive));
}
