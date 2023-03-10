#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

class Auto1 : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto1> {
 public:
  Auto1(DriveSubsystem* drive);
};
