#include "commands/auto/Auto_ScoreAndBalance.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmMoveCommand.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "commands/IntakeCubeCommand.h"
#include "commands/ScoreConeCommand.h"
#include "commands/ScoreCubeCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto origin = frc::Translation2d { 0_m, 0_m };

// class Auto_ScoreAndBalance_IntakeTimout
//   : public frc2::CommandHelper<frc2::ParallelRaceGroup, Auto_ScoreAndBalance_IntakeTimout> {
//  public:
//   Auto_ScoreAndBalance_IntakeTimout(IntakeSubsystem* intake) {
//     AddCommands(IntakeCubeCommand(intake), frc2::WaitCommand(2_s));
//   }
// };
// Auto_ScoreAndBalance_IntakeTimout(intake),

Auto_ScoreAndBalance::Auto_ScoreAndBalance(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([intake] { intake->CloseCone(); }, {}),
    ArmMoveCommand(arm, 78_deg, 0_in),
    ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CONE),
    frc2::ParallelCommandGroup(
      frc2::SequentialCommandGroup(
        ScoreConeCommand(arm, intake),
        ArmMoveCommand(arm, ArmPosition::INTAKE_FLOOR_CUBE),
        frc2::ParallelRaceGroup(IntakeCubeCommand(intake), frc2::WaitCommand(2_s))),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        origin,
        true,
        false,
        false,
        drive)),
    frc2::ParallelCommandGroup(
      ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CUBE),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 4.6_m, 0_m, 180_deg }, frc::Rotation2d(0_deg)),
          Waypoint(frc::Pose2d { 3.6_m, 0_m, 180_deg }),
          Waypoint(frc::Pose2d { 2.6_m, 0_m, 180_deg }),
          Waypoint(frc::Pose2d { 1.6_m, 0_m, 180_deg }, 1_mps),
          Waypoint(frc::Pose2d { 0_m, 0_m, 180_deg }, frc::Rotation2d(180_deg)),
        },
        origin,
        false,
        false,
        false,
        drive)),
    frc2::ParallelCommandGroup(
      ScoreCubeCommand(arm, intake),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 90_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 0_m, 1_m, 90_deg }),
          Waypoint(frc::Pose2d { 0.5_m, 2_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 2_m, 2_m, 0_deg }, frc::Rotation2d(90_deg)),
        },
        origin,
        false,
        true,
        drive)),
    AutoBalanceCommand(drive));
}