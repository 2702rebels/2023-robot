#include "commands/AutoBalanceCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(AutoBalanceSpeed, "Auto/Balance/Speed", 1.1);
REGISTER_DOUBLE(AutoBalanceStopAngle, "Auto/Balance/Stop Angle", 5.0);
REGISTER_DOUBLE(AutoBalanceStopV, "Auto/Balance/Stop Velocity", 4.0);
REGISTER_DOUBLE(AutoBalanceFactor, "Auto/Balance/Speed Factor", 16.0);
REGISTER_DOUBLE(AutoBalanceThresholdV, "Auto/Balance/Threshold Velocity", 8.0);

// clang-format on

AutoBalanceCommand::AutoBalanceCommand(DriveSubsystem* drive) : m_drive { drive } {
  AddRequirements(drive);
}

void AutoBalanceCommand::Initialize() {
  m_speed = units::meters_per_second_t { GetAutoBalanceSpeed() };
  m_factor = GetAutoBalanceFactor();
  m_stopAngle = units::degree_t { GetAutoBalanceStopAngle() };
  m_stopV = units::degrees_per_second_t { GetAutoBalanceStopV() };
  m_thresholdV = units::degrees_per_second_t { GetAutoBalanceThresholdV() };

  m_angle = 10_rad;
}

void AutoBalanceCommand::Execute() {
  const auto currentPose = m_drive->GetPose();

  m_angle = m_drive->GetInclination();
  m_velocity = m_drive->GetInclinationVelocity();

  const auto counterBalanced =
    units::math::fabs(m_angle) < 15.0_deg && ((m_angle < 0_rad && m_velocity > m_thresholdV) ||
                                              (m_angle > 0_rad && m_velocity < -m_thresholdV));

  if (counterBalanced) {
    m_drive->Stop();
  } else {
    const auto f =
      std::min(1.0, pow(units::math::fabs(m_angle).convert<units::degree>().value() / m_factor, 2));

    const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      m_speed * (m_angle > 0_rad ? 1.0 : -1.0) * f,
      0_mps,
      0_rad_per_s,
      currentPose.Rotation().Radians());

    m_drive->SetChassisSpeeds(speeds);
  }

#ifdef AUTO_COMMAND_LOG
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Angle", m_angle.convert<units::degree>().value());
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Velocity", m_velocity.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutBoolean("Telemetry/AutoBalance/Balanced", counterBalanced);
  // clang-format on
#endif
}

void AutoBalanceCommand::End(bool interrupted) {
  m_drive->StopX();
}

bool AutoBalanceCommand::IsFinished() {
  return units::math::fabs(m_angle) < m_stopAngle && units::math::fabs(m_velocity) < m_stopV;
}
