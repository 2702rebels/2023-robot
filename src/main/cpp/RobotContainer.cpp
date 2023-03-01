#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "commands/HomeAxis1Command.h"
#include "commands/HomeAxis2Command.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // initialize all of your commands and subsystems here
#ifdef EXTENDING_ARM_ENABLED
#ifdef INTAKE_ENABLED
  m_exArm.SetIntakeReference(&m_intake);
#endif
#endif

#ifdef LED_ENABLED
  m_led.SetLength(LEDConstants::kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
  // for (int i = 0; i < LEDConstants::kLength; i++) {
  //   m_ledBuffer[i].SetRGB(255, 0, 0);
  // }

  m_ledBuffer[0].SetRGB(255, 0, 0);
  m_ledBuffer[1].SetRGB(0, 0, 255);
  m_ledBuffer[2].SetRGB(0, 0, 0);
  m_ledBuffer[3].SetRGB(0, 255, 0);
  m_ledBuffer[4].SetRGB(0, 0, 0);
  m_ledBuffer[5].SetRGB(255, 0, 0);
  m_ledBuffer[6].SetRGB(0, 0, 255);
  m_ledBuffer[7].SetRGB(0, 0, 0);
  m_ledBuffer[8].SetRGB(0, 255, 0);
  m_ledBuffer[9].SetRGB(0, 0, 0);

  m_led.SetData(m_ledBuffer);
#endif

// register subsystems on the shuffleboard
#ifdef DRIVETRAIN_ENABLED
  frc::Shuffleboard::GetTab("Drivetrain").Add(m_drive);
#endif
#ifdef EXTENDING_ARM_ENABLED
  frc::Shuffleboard::GetTab("Arm").Add(m_exArm);
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
        -units::meters_per_second_t {
          frc::ApplyDeadband(m_driverController.GetLeftY(), m_drive.GetDeadband()) },
        -units::meters_per_second_t {
          frc::ApplyDeadband(m_driverController.GetLeftX(), m_drive.GetDeadband()) },
        -units::radians_per_second_t {
          frc::ApplyDeadband(m_driverController.GetRightX(), m_drive.GetDeadband()) },
        true,
        true);
    },
    { &m_drive }));
#endif

  // autonomous command chooser
#ifdef AUTO_CHOOSER
  m_chooser.SetDefaultOption("Auto1", &m_auto1);
  m_chooser.AddOption("Auto2", &m_auto2);
  m_chooser.AddOption("Auto3", &m_auto3);
  frc::Shuffleboard::GetTab("Auto").Add(m_chooser);
#endif
}

void RobotContainer::OnDisable() {
#ifdef EXTENDING_ARM_ENABLED
  m_exArm.OnDisable();
#endif
}

void RobotContainer::ToggleRobotMode() {
  if (m_mode == RobotMode::CONE) {
    m_mode = RobotMode::CUBE;
#ifdef LED_ENABLED
    for (int i = 0; i < LEDConstants::kLength; i++) {
      m_ledBuffer[i].SetRGB(255, 0, 255);
    }
    m_led.SetData(m_ledBuffer);
#endif
  } else {
    m_mode = RobotMode::CONE;
#ifdef LED_ENABLED
    for (int i = 0; i < LEDConstants::kLength; i++) {
      // m_ledBuffer[i].SetRGB(255, 255, 0);
      m_ledBuffer[i].SetRGB(0, 0, 0);
    }
    m_led.SetData(m_ledBuffer);
#endif
  }
}

std::string RobotContainer::GetRobotModeState() {
  if (m_mode == RobotMode::CONE) {
    return "CONE";
  } else if (m_mode == RobotMode::CUBE) {
    return "CUBE";
  }
  return "Undefined";
}

void RobotContainer::ConfigureButtonBindings() {
  // Toggle robot mode.
  frc2::Trigger(
    m_driverController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::InstantCommand([this] { ToggleRobotMode(); }, {}).ToPtr());

#ifdef DRIVETRAIN_ENABLED
  frc2::Trigger(
    m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, { &m_drive }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick)
    .OnTrue(frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, { &m_drive }).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    .OnTrue(frc2::InstantCommand([this] { m_drive.ResetOdometry(frc::Pose2d {}); }, { &m_drive })
              .ToPtr());
#endif

#ifdef EXTENDING_ARM_ENABLED
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
    .OnTrue(new frc2::InstantCommand([this] { m_exArm.ConfigureTuningParams(); }, { &m_exArm }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
    .OnTrue(new frc2::InstantCommand(
      [this] {
        m_exArm.ZeroAxis1();
        m_exArm.ZeroAxis2();
        m_exArm.ZeroAxis3();
      },
      { &m_exArm }));

  // frc2::Trigger(
  //   m_driverController.POVLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::InstantCommand([this] { m_exArm.SetAxis3NextSetpoint(0_deg); }, { &m_exArm
  //   }).ToPtr());

  // frc2::Trigger(
  //   m_driverController.POVRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::InstantCommand([this] { m_exArm.SetAxis3NextSetpoint(90_deg); },
  //   {&m_exArm}).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    .OnTrue(m_exArm.MoveArm(ExtendingArmPositions::STOWED));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
    .OnTrue(frc2::cmd::Either(
      m_exArm.MoveArm(ExtendingArmPositions::SCORE_MID_CONE),
      m_exArm.MoveArm(ExtendingArmPositions::SCORE_MID_CUBE),
      [this] { return m_mode == RobotMode::CONE; }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
    .OnTrue(frc2::cmd::Either(
      m_exArm.MoveArm(ExtendingArmPositions::SCORE_HIGH_CONE),
      m_exArm.MoveArm(ExtendingArmPositions::SCORE_HIGH_CUBE),
      [this] { return m_mode == RobotMode::CONE; }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    .OnTrue(frc2::InstantCommand(
              [this] { m_intake.SetIntakeSpeed(IntakeConstants::kIntakingSpeed); }, { &m_intake })
              .ToPtr())
    .OnFalse(frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }).ToPtr());

#ifdef INTAKE_ENABLED
  // Intake automatically.
  frc2::Trigger(m_driverController.RightTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Sequence(
      m_exArm.MoveArm(ExtendingArmPositions::INTAKE_STATION_CONE),
      frc2::InstantCommand([this] { m_intake.StartIntaking(); }, { &m_intake })))
    .OnFalse(frc2::cmd::Sequence(
      frc2::InstantCommand([this] { m_intake.StopIntaking(); }, { &m_intake }),
      m_exArm.MoveArm(ExtendingArmPositions::STOWED)));

  // Score automatically.
  frc2::Trigger(m_driverController.LeftTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Sequence(
      m_exArm.MoveArm(ExtendingArmPositions::INTAKE_FLOOR_CONE),
      frc2::InstantCommand(
        [this] { m_intake.SetIntakeSpeed(IntakeConstants::kIntakingSpeed); }, { &m_intake })))
    .OnFalse(frc2::cmd::Sequence(
      frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }),
      m_exArm.MoveArm(ExtendingArmPositions::STOWED)));

  // Intake manually.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        m_exArm.MoveArm(ExtendingArmPositions::INTAKE_FLOOR_CONE),
        frc2::InstantCommand([this] { m_intake.StartIntaking(); }, { &m_intake })),
      frc2::cmd::Sequence(
        m_exArm.MoveArm(ExtendingArmPositions::INTAKE_FLOOR_CUBE),
        frc2::InstantCommand(
          [this] { m_intake.SetIntakeSpeed(IntakeConstants::kIntakingSpeed); }, { &m_intake })),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(frc2::cmd::Sequence(
      frc2::InstantCommand([this] { m_intake.StopIntaking(); }, { &m_intake }),
      m_exArm.MoveArm(ExtendingArmPositions::STOWED)));

  // Score manually.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(frc2::cmd::Either(
      m_exArm.ScoreCone(), m_exArm.ScoreCube(), [this] { return m_mode == RobotMode::CONE; }));
#endif
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifdef AUTO_CHOOSER
  return m_chooser.GetSelected();
#else
#ifdef DRIVETRAIN_ENABLED
  // set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

  // add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // an example trajectory to follow (all units in meters)
  const auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    {
      frc::Pose2d { 0_m, 0_m, 0_deg },
      frc::Pose2d { 1_m, 0_m, 0_deg },
      frc::Pose2d { 2_m, 0_m, 0_deg },
      frc::Pose2d { 3_m, 0_m, 0_deg },
      frc::Pose2d { 4_m, 0_m, 0_deg },
    },
    // pass the config
    config);

  frc::ProfiledPIDController<units::radians> thetaController {
    AutoConstants::kPThetaController,
    AutoConstants::kIThetaController,
    AutoConstants::kDThetaController,
    AutoConstants::kThetaControllerConstraints
  };

  thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });

  const auto rotation = frc::Rotation2d { 180_deg };
  frc2::SwerveControllerCommand<NUMBER_OF_MODULES> swerveControllerCommand(
    exampleTrajectory,
    [this]() { return m_drive.GetPose(); },
    m_drive.kDriveKinematics,
    frc2::PIDController {
      AutoConstants::kPXController, AutoConstants::kIXController, AutoConstants::kDXController },
    frc2::PIDController {
      AutoConstants::kPYController, AutoConstants::kIYController, AutoConstants::kDYController },
    thetaController,
    [rotation]() { return rotation; },
    [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
    { &m_drive });

  // reset odometry to the starting pose of the trajectory
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
    std::move(swerveControllerCommand),
    frc2::InstantCommand([this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
#else
  return new frc2::InstantCommand(
    [this] { frc::DataLogManager::Log("Drivetrain disabled. Auto not run."); });
#endif
#endif
}
