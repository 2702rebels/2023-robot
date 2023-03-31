#pragma once

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <vector>

#include "Constants.h"

class Field : public frc2::SubsystemBase {
 public:
  enum Stations {
    CONE_1,
    CONE_2,
    CONE_3,
    CONE_4,
    CONE_5,
    CONE_6,
    CUBE_1,
    CUBE_2,
    CUBE_3,
    LOAD_L,
    LOAD_R
  };

  Field();
  void Periodic() override;

  /**
   * Shifts selection of the target station.
   */
  void ShiftSelection(double inputX, double inputY);

  /**
   * Selects the target score station.
   */
  void SelectScoreStation(Stations station) {
    m_stationScore = station;
  }

  /**
   * Selects the target load station.
   */
  void SelectLoadStation(Stations station) {
    m_stationLoad = station;
  }

  /**
   * Returns the position based on the currently selected station and alliance from DS.
   */
  frc::Pose2d GetScorePosition() const;

 private:
  inline constexpr static frc::Translation2d Positions[] = {
    { 138.271_cm, 51.905_cm },             // RED  - CONE_1
    { 138.271_cm, 163.665_cm },            // RED  - CONE_2
    { 138.271_cm, 219.545_cm },            // RED  - CONE_3
    { 138.271_cm, 331.305_cm },            // RED  - CONE_4
    { 138.271_cm, 387.185_cm },            // RED  - CONE_5
    { 138.271_cm, 498.945_cm },            // RED  - CONE_6
    { 138.271_cm, 107.785_cm },            // RED  - CUBE_1
    { 138.271_cm, 275.423_cm },            // RED  - CUBE_2
    { 138.271_cm, 442.818_cm },            // RED  - CUBE_3
    { 0_cm, 0_cm },                        // RED  - LOAD_L
    { 0_cm, 0_cm },                        // RED  - LOAD_R
    { 54.3_ft - 138.271_cm, 51.905_cm },   // BLUE - CONE_1
    { 54.3_ft - 138.271_cm, 163.665_cm },  // BLUE - CONE_2
    { 54.3_ft - 138.271_cm, 219.545_cm },  // BLUE - CONE_3
    { 54.3_ft - 138.271_cm, 331.305_cm },  // BLUE - CONE_4
    { 54.3_ft - 138.271_cm, 387.185_cm },  // BLUE - CONE_5
    { 54.3_ft - 138.271_cm, 498.945_cm },  // BLUE - CONE_6
    { 54.3_ft - 138.271_cm, 107.785_cm },  // BLUE - CUBE_1
    { 54.3_ft - 138.271_cm, 275.423_cm },  // BLUE - CUBE_2
    { 54.3_ft - 138.271_cm, 442.818_cm },  // BLUE - CUBE_3
    { 0_cm, 0_cm },                        // BLUE - LOAD_L
    { 0_cm, 0_cm },                        // BLUE - LOAD_R
  };

  Stations m_stationScore;
  Stations m_stationLoad;
  units::second_t m_lastShiftedAt;
};
