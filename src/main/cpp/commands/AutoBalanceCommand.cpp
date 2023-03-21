#include "commands/AutoBalanceCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(AutoBalanceSpeed, "Auto/Balance/Speed", 1.0);
REGISTER_DOUBLE(AutoBalanceAngleThreshold, "Auto/Balance/Angle Threshold", 5.0);
REGISTER_DOUBLE(AutoBalanceVelocityThreshold, "Auto/Balance/Velocity Threshold", 8.0);

// clang-format on

AutoBalanceCommand::AutoBalanceCommand(DriveSubsystem* drive) : m_drive { drive } {
  AddRequirements(drive);
}

void AutoBalanceCommand::Initialize() {
  m_speed = units::meters_per_second_t { GetAutoBalanceSpeed() };
  m_angleThreshold = units::degree_t { GetAutoBalanceAngleThreshold() };
  m_velocityThreshold = units::degrees_per_second_t { GetAutoBalanceVelocityThreshold() };

  m_angle = 10_rad;  // some big number so that command does not finish right away
}

void AutoBalanceCommand::Execute() {
  const auto currentPose = m_drive->GetPose();

  m_angle = m_drive->GetInclination();
  const auto angleVelocity = m_drive->GetInclinationVelocity();

  const auto counterBalanced = (m_angle < 0_rad && angleVelocity > m_velocityThreshold) ||
                               (m_angle > 0_rad && angleVelocity < -m_velocityThreshold);

  if (counterBalanced) {
    m_drive->Stop();
  } else {
    const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      m_speed * (m_angle > 0_rad ? 1.0 : -1.0),
      0_mps,
      0_rad_per_s,
      currentPose.Rotation().Radians());

    m_drive->SetChassisSpeeds(speeds);
  }

#ifdef AUTO_TELEMETRY
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Angle", m_angle.convert<units::degree>().value());
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Angle Velocity", angleVelocity.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutBoolean("Telemetry/AutoBalance/Balanced", counterBalanced);
  // clang-format on
#endif
}

void AutoBalanceCommand::End(bool interrupted) {
  m_drive->StopX();
}

bool AutoBalanceCommand::IsFinished() {
  return units::math::fabs(m_angle) < m_angleThreshold;
}
