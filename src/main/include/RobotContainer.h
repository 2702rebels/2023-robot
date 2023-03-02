#pragma once

#include <frc/AddressableLED.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "commands/Auto1.h"
#include "commands/Auto2.h"
#include "commands/Auto3.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ExtendingArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.
 * Since command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

enum RobotMode { CONE, CUBE };

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void OnDisable();
  void OnTeleEnable();

  void ToggleRobotMode(RobotMode mode);
  void ToggleRobotMode();

  std::string GetRobotModeString();

 private:
  // The driver's controller
  frc::XboxController m_driverController { OIConstants::kDriverControllerPort };
  frc::XboxController m_operatorController { OIConstants::kDriverControllerPort };

  // The robot's subsystems
#ifdef DRIVETRAIN_ENABLED
  DriveSubsystem m_drive;
#endif

#ifdef EXTENDING_ARM_ENABLED
  ExtendingArmSubsystem m_exArm;
#endif

#ifdef EXTENDING_ARM_ENABLED
#ifdef INTAKE_ENABLED
  IntakeSubsystem m_intake;
#endif
#endif

#ifdef LED_ENABLED
  frc::AddressableLED m_led { LEDConstants::kpwmPort };
  std::array<frc::AddressableLED::LEDData, LEDConstants::kLength> m_ledBuffer;
#endif

  // The autonomous routines
  Auto1 m_auto1 { &m_drive };
  Auto2 m_auto2 { &m_drive };
  Auto3 m_auto3 { &m_drive };

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  RobotMode m_mode = RobotMode::CONE;

  void ConfigureButtonBindings();
};
