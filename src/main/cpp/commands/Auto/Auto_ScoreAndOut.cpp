#include "commands/auto/Auto_ScoreAndOut.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmMoveCommand.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "commands/IntakeCubeCommand.h"
#include "commands/ScoreConeCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto initialPose = frc::Pose2d { 0_m, 0_m, 180_deg };

Auto_ScoreAndOut::Auto_ScoreAndOut(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([intake] { intake->CloseCone(); }, {}),
    ArmMoveCommand(arm, 68_deg, 0_in),
    ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CONE),
    ScoreConeCommand(arm, intake),
    DriveTrajectoryCommand(
      {
        Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
        Waypoint(frc::Pose2d { 2.7_m, 0_m, 0_deg }, frc::Rotation2d(180_deg), 0.7_mps),
      },
      initialPose.Translation(),
      true,  // reset odometry
      true,  // stop based on timer
      drive),
    frc2::ParallelCommandGroup(
      ArmMoveCommand(arm, ArmPosition::INTAKE_FLOOR_CUBE),
      frc2::ParallelRaceGroup(IntakeCubeCommand(intake), frc2::WaitCommand(4_s)),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 2.7_m, 0_m, 0_deg }, frc::Rotation2d(180_deg), 0.7_mps),
          Waypoint(frc::Pose2d { 4.9_m, -0.3_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        initialPose.Translation(),
        false,
        true,  // stop based on timer
        drive)),
    frc2::InstantCommand([intake] { intake->CloseCube(); }, {}),
    ArmMoveCommand(arm, ArmPosition::STOWED));
}
