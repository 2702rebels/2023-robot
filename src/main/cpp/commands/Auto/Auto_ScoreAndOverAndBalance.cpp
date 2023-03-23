#include "commands/auto/Auto_ScoreAndOverAndBalance.h"

#include "commands/ArmMoveCommand.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveOverBridgeCommand.h"
#include "commands/ScoreCubeCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto initialPose = frc::Pose2d { 0_m, 0_m, 180_deg };

Auto_ScoreAndOverAndBalance::Auto_ScoreAndOverAndBalance(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([drive] { drive->ResetOdometry(initialPose); }, {}),
    frc2::InstantCommand([intake] { intake->CloseCube(); }, {}),
    ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CUBE),
    ScoreCubeCommand(arm, intake),
    DriveOverBridgeCommand(drive),
    AutoBalanceCommand(drive));
}
