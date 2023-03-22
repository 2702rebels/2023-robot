#include "commands/DriveOverBridgeCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(DriveOverBridgeCommandSpeedIn, "Auto/DriveOverBridge/Speed In", 1.2);
REGISTER_DOUBLE(DriveOverBridgeCommandSpeedOut, "Auto/DriveOverBridge/Speed Out", 0.7);
REGISTER_DOUBLE(DriveOverBridgeCommandAngleThreshold, "Auto/DriveOverBridge/Angle Threshold", 8.0);
REGISTER_DOUBLE(DriveOverBridgeCommandVelocityThreshold, "Auto/DriveOverBridge/Velocity Threshold", 40.0);
REGISTER_DOUBLE(DriveOverBridgeCommandTimeOut, "Auto/DriveOverBridge/Time Out", 0.5);
REGISTER_DOUBLE(DriveOverBridgeCommandTimeIn, "Auto/DriveOverBridge/Time In", 1.2);

// clang-format on

DriveOverBridgeCommand::DriveOverBridgeCommand(DriveSubsystem* drive) : m_drive { drive } {
  AddRequirements(drive);
}

void DriveOverBridgeCommand::Initialize() {
  m_speedIn = units::meters_per_second_t { GetDriveOverBridgeCommandSpeedIn() };
  m_speedOut = units::meters_per_second_t { GetDriveOverBridgeCommandSpeedOut() };
  m_angleThreshold = units::degree_t { GetDriveOverBridgeCommandAngleThreshold() };
  m_velocityThreshold =
    units::degrees_per_second_t { GetDriveOverBridgeCommandVelocityThreshold() };
  m_timeOut = units::second_t { GetDriveOverBridgeCommandTimeOut() };
  m_timeIn = units::second_t { GetDriveOverBridgeCommandTimeIn() };

  m_speed = m_speedIn;
  m_stage = 0;
  m_angle = 10_rad;  // some big number so that command does not finish right away
  m_timer.Reset();
}

void DriveOverBridgeCommand::Execute() {
  const auto currentPose = m_drive->GetPose();

  m_angle = m_drive->GetInclination();
  const auto angleVelocity = m_drive->GetInclinationVelocity();

  switch (m_stage) {
    case 0:
      // start driving up the ramp
      if (m_angle > m_angleThreshold) {
        m_stage = 1;
      }
      break;

    case 1:
      // reduce speed as we start going downhill
      if (angleVelocity < -m_velocityThreshold) {
        m_speed = m_speedOut;
      }

      // start driving down the ramp
      if (m_angle < -m_angleThreshold) {
        m_stage = 2;
      }
      break;

    case 2:
      // we are on the ground, start timer to reverse
      if (units::math::fabs(m_angle) < 2_deg) {
        m_stage = 3;
        m_timer.Restart();
      }
      break;

    case 3:
      if (m_timer.Get() > m_timeOut) {
        m_stage = 4;
        m_timer.Restart();
        m_speed = -m_speedIn;
      }
      break;
  }

  const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    m_speed, 0_mps, 0_rad_per_s, currentPose.Rotation().Radians());

  m_drive->SetChassisSpeeds(speeds);

#ifdef AUTO_COMMAND_LOG
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/DriveOverBridge/Angle", m_angle.convert<units::degree>().value());
  frc::SmartDashboard::PutString("Telemetry/DriveOverBridge/Stage", m_stage == 1 ? "Entry" : m_stage == 2 ? "Exit" : m_stage == 3 ? "Stop" : m_stage == 4 ? "Reverse" : "None");
  // clang-format on
#endif
}

void DriveOverBridgeCommand::End(bool interrupted) {
  // do not stop since we want AutoBalance to take over
  // m_drive->Stop();
}

bool DriveOverBridgeCommand::IsFinished() {
  return m_stage == 4 && m_timer.Get() > m_timeIn;
}
