#include "Field.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

Field::Field() {}

void Field::Periodic() {
  frc::SmartDashboard::PutBoolean("Cone 1", m_stationScore == Stations::CONE_1);
  frc::SmartDashboard::PutBoolean("Cone 2", m_stationScore == Stations::CONE_2);
  frc::SmartDashboard::PutBoolean("Cone 3", m_stationScore == Stations::CONE_3);
  frc::SmartDashboard::PutBoolean("Cone 4", m_stationScore == Stations::CONE_4);
  frc::SmartDashboard::PutBoolean("Cone 5", m_stationScore == Stations::CONE_5);
  frc::SmartDashboard::PutBoolean("Cone 6", m_stationScore == Stations::CONE_6);
  frc::SmartDashboard::PutBoolean("Cube 1", m_stationScore == Stations::CUBE_1);
  frc::SmartDashboard::PutBoolean("Cube 2", m_stationScore == Stations::CUBE_2);
  frc::SmartDashboard::PutBoolean("Cube 3", m_stationScore == Stations::CUBE_3);
  frc::SmartDashboard::PutBoolean("Load L", m_stationLoad == Stations::LOAD_L);
  frc::SmartDashboard::PutBoolean("Load R", m_stationLoad == Stations::LOAD_R);
}

frc::Pose2d Field::GetScorePosition() const {
  const auto alliance = frc::DriverStation::GetAlliance();
  const auto adjustment = alliance == frc::DriverStation::Alliance::kRed
                          ? RobotConstants::kBumperSize / 2
                          : -RobotConstants::kBumperSize / 2;

  const auto position = Positions[alliance * (Stations::LOAD_R + 1) + m_stationScore];
  return frc::Pose2d(
    frc::Translation2d(position.X() + adjustment, position.Y()),
    alliance == frc::DriverStation::Alliance::kRed ? 0_deg : 180_deg);
}

void Field::ShiftSelection(double inputX, double inputY) {
  if (std::fabs(inputX) >= 0.5) {
    const auto timestamp = frc::Timer::GetFPGATimestamp();
    if (timestamp - m_lastShiftedAt < 0.2_s) {
      return;
    }

    m_lastShiftedAt = timestamp;

    if (inputX < 0) {
      if (m_stationScore > 0) {
        m_stationScore = (Stations) (m_stationScore - 1);
      }
    } else {
      if (m_stationScore < Stations::CUBE_3) {
        m_stationScore = (Stations) (m_stationScore + 1);
      }
    }
  }

  if (std::fabs(inputY) >= 0.5) {
    if (inputY > 0) {
      m_stationLoad = Stations::LOAD_L;
    } else {
      m_stationLoad = Stations::LOAD_R;
    }
  }
}
