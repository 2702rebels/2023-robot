#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"

class Auto_ScoreAndBalance
  : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto_ScoreAndBalance> {
 public:
  Auto_ScoreAndBalance(DriveSubsystem* drive, ArmSubsystem* arm);
};
