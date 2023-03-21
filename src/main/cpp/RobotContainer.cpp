#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include <utility>

#include "Constants.h"
#include "commands/ArmMoveCommand.h"
#include "commands/IntakeConeCommand.h"
#include "commands/IntakeCubeCommand.h"
#include "commands/IntakeHomeGripperCommand.h"
#include "commands/ScoreConeCommand.h"
#include "commands/ScoreCubeCommand.h"
#include "commands/SetLedCommand.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // initialize all of your commands and subsystems here

#ifdef LED_ENABLED
  m_leds.SetRGB(255, 0, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 1_s);
#endif

// register subsystems on the shuffleboard
#ifdef DRIVETRAIN_ENABLED
  frc::Shuffleboard::GetTab("Drivetrain").Add(m_drive);
#endif
#ifdef ARM_ENABLED
  frc::Shuffleboard::GetTab("Arm").Add(m_arm);
#endif

#ifdef TRACE_LOG
  // Log Shuffleboard events for command initialize, execute, finish, interrupt
  frc2::CommandScheduler::GetInstance().OnCommandInitialize([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command initialized", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    frc::DataLogManager::Log(fmt::format("Scheduler|{}: Initialize", command.GetName()));
  });
  frc2::CommandScheduler::GetInstance().OnCommandExecute([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command executed", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    // frc::DataLogManager::Log(fmt::format("Scheduler|{}: Execute", command.GetName()));
  });
  frc2::CommandScheduler::GetInstance().OnCommandFinish([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command finished", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    frc::DataLogManager::Log(fmt::format("Scheduler|{}: Finish", command.GetName()));
  });
  frc2::CommandScheduler::GetInstance().OnCommandInterrupt([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command interrupted", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    frc::DataLogManager::Log(fmt::format("Scheduler|{}: Interupt", command.GetName()));
  });
#endif

  // configure the button bindings
  ConfigureButtonBindings();

  // set up default drive command:
  // - the left stick controls translation of the robot
  // - turning is controlled by the X axis of the right stick
#ifdef DRIVETRAIN_ENABLED
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drive.Drive(
        -frc::ApplyDeadband(m_driverController.GetLeftY(), m_drive.GetDeadband()),
        -frc::ApplyDeadband(m_driverController.GetLeftX(), m_drive.GetDeadband()),
        -frc::ApplyDeadband(m_driverController.GetRightX(), m_drive.GetDeadband()),
        true);
    },
    { &m_drive }));
#endif

  // autonomous command chooser
#ifdef AUTO_CHOOSER
  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Auto");
  m_chooser.SetDefaultOption("DoNothing", &m_auto_doNothing);
  m_chooser.AddOption("DriveForwards", &m_auto_driveForwards);
  m_chooser.AddOption("Auto2", &m_auto2);
#ifdef ARM_ENABLED
  m_chooser.AddOption("Auto3", &m_auto3);
#endif
  tab.Add(m_chooser);
  tab.AddPersistent("DriveAutoDistance", 1.0);
#endif
}

void RobotContainer::OnAutoEnable() {
  m_leds.SetRGB(0, 255, 0);
  m_leds.SetPattern(LEDPattern::FLASH, 500_ms);
}

void RobotContainer::OnTeleEnable() {
  SetRobotMode(RobotMode::CONE);
}

void RobotContainer::OnDisable() {
#ifdef ARM_ENABLED
  m_arm.OnDisable();
#endif

#ifdef INTAKE_ENABLED
  m_intake.OnDisable();
#endif

#ifdef LED_ENABLED
  m_leds.SetRGB(255, 0, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 3_s);
#endif
}

void RobotContainer::SetRobotMode(RobotMode mode) {
  m_mode = mode;
  if (m_mode == RobotMode::CUBE) {
#ifdef LED_ENABLED
    m_leds.SetRGB(255, 0, 255);
    m_leds.SetPattern(LEDPattern::SOLID, 0_s);
#endif
  } else if (m_mode == RobotMode::CONE) {
#ifdef LED_ENABLED
    m_leds.SetRGB(255, 255, 0);
    m_leds.SetPattern(LEDPattern::SOLID, 0_s);
#endif
  }
}

void RobotContainer::ToggleRobotMode() {
  if (m_mode == RobotMode::CONE) {
    SetRobotMode(RobotMode::CUBE);
  } else {
    SetRobotMode(RobotMode::CONE);
  }
}

std::string RobotContainer::GetRobotModeString() {
  if (m_mode == RobotMode::CONE) {
    return "CONE";
  } else if (m_mode == RobotMode::CUBE) {
    return "CUBE";
  }
  return "Undefined";
}

void RobotContainer::ConfigureButtonBindings() {
#ifdef ARM_ENABLED
#ifdef INTAKE_ENABLED
  // Reload parameters.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
    .OnTrue(new frc2::InstantCommand(
      [this] {
        m_arm.ConfigureTuningParams();
        m_intake.ConfigureTuningParams();
      },
      {}));
#endif
#endif

  // Toggle robot mode.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(frc2::InstantCommand([this] { ToggleRobotMode(); }, {}).ToPtr());

#ifdef DRIVETRAIN_ENABLED
  // Reset swerve heading by zeroing the gyro
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick)
    .OnTrue(frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, { &m_drive }).ToPtr());

  // Lock the wheels
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick)
    .WhileTrue(frc2::RunCommand([this] { m_drive.StopX(); }, { &m_drive }).ToPtr());

  frc2::Trigger(
    m_driverController.POVLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(
      frc2::InstantCommand([this] { m_drive.UpdateTuningParameters(); }, { &m_drive }).ToPtr());

  // frc2::Trigger(
  //   m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::InstantCommand([this] { m_drive.ResetOdometry(frc::Pose2d {}); }, { &m_drive })
  //             .ToPtr());
#endif

#ifdef ARM_ENABLED
  // Zero's the arm axis. TODO: Change this to run the homing command.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
    .OnTrue(IntakeHomeGripperCommand(&m_intake).ToPtr());

  frc2::Trigger(
    m_driverController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(m_arm.JogUp());

  frc2::Trigger(
    m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(m_arm.JogDown());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    .OnTrue(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
    .OnTrue(frc2::cmd::Either(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CONE).ToPtr(),
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CUBE).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
    .OnTrue(frc2::cmd::Either(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CONE).ToPtr(),
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CUBE).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    .OnTrue(frc2::cmd::Sequence(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_LOW_CUBE),
      frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(-0.5); }, { &m_intake }).ToPtr()))
    .OnFalse(frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }).ToPtr());

#ifdef INTAKE_ENABLED
  // Intake automatically.
  frc2::Trigger(m_driverController.LeftTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_STATION_CONE), IntakeConeCommand(&m_intake)),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_STATION_CUBE), IntakeCubeCommand(&m_intake)),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(frc2::cmd::Sequence(ArmMoveCommand(&m_arm, ArmPosition::STOWED)));

  // Score automatically.
  // frc2::Trigger(m_driverController.LeftTrigger(
  //                 0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::cmd::Sequence(
  //     m_exArm.MoveArm(ExtendingArmPositions::INTAKE_FLOOR_CONE),
  //     frc2::InstantCommand(
  //       [this] { m_intake.SetIntakeSpeed(IntakeConstants::kIntakingSpeed); }, { &m_intake })))
  //   .OnFalse(frc2::cmd::Sequence(
  //     frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }),
  //     m_exArm.MoveArm(ExtendingArmPositions::STOWED)));

  // Intake manually.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CONE), IntakeConeCommand(&m_intake)),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CUBE), IntakeCubeCommand(&m_intake)),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  // Score Manually.
  frc2::Trigger(m_driverController.RightTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Either(
      ScoreConeCommand(&m_arm, &m_intake).ToPtr(),
      ScoreCubeCommand(&m_arm, &m_intake).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));
  // .OnFalse(frc2::cmd::Parallel(
  //   IntakeHomeGripperCommand(&m_intake), ArmMoveCommand(&m_arm, ArmPosition::STOWED)));

  // Score manually.
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
  //   .OnTrue(frc2::cmd::Either(
  //     m_exArm.ScoreCone(), m_exArm.ScoreCube(), [this] { return m_mode == RobotMode::CONE; }));
#endif
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifdef AUTO_CHOOSER
  return m_chooser.GetSelected();
#else
  return new frc2::InstantCommand([this] { frc::DataLogManager::Log("No autonomous code"); });
#endif
}
