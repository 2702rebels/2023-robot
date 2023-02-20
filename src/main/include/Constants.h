#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#include "Directives.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants. This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters
// Note that these are not the maximum capable speeds of the robot, rather the
// allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 2_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed { 2 * std::numbers::pi };

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
// Distances between centers of wheels on robot
constexpr units::meter_t kTrackWidth = 0.5461_m;  // right and left wheels
constexpr units::meter_t kWheelBase = 0.5461_m;   // front and back wheels

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 7;
constexpr int kRearLeftDrivingCanId = 8;
constexpr int kFrontRightDrivingCanId = 2;
constexpr int kRearRightDrivingCanId = 1;

constexpr int kFrontLeftTurningCanId = 5;
constexpr int kRearLeftTurningCanId = 6;
constexpr int kFrontRightTurningCanId = 4;
constexpr int kRearRightTurningCanId = 3;
}  // namespace DriveConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference = kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
  (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) / kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
  (kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
  ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
  60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor = (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
  (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
  units::radian_t { kTurningEncoderPositionFactor };

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 0.5;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.1;
}  // namespace OIConstants

namespace IntakeConstants {
constexpr int kIntakeLeaderControllerCanId = 9;
constexpr int kIntakeFollowerControllerCanId = 10;

// TODO: We might need separate speeds for cube and cone.
constexpr double kIntakingSpeed = 0.6;
constexpr double kOuttakingSpeed = -0.7;
constexpr double kintakeIdleSpeed = 0.2;  // 775's can stall at 20% duty cycle for an entire match.
}  // namespace IntakeConstants

namespace ExtendingArmConstants {
// Motor controller CAN ID's
constexpr int kExtendingArmAxis1MotorControllerCanId = 11;
constexpr int kExtendingArmAxis2MotorControllerCanId = 12;
constexpr int kExtendingArmAxis3MotorControllerCanId = 13;

// PID constants for the arm axis's.
// Axis 1 (Shoulder)
constexpr double kAxis1P = 0.05;
constexpr double kAxis1I = 0;
constexpr double kAxis1D = 0.00;
constexpr double kAxis1FF = 0.000156;
constexpr double kAxis1MinOutput = -1.0;
constexpr double kAxis1MaxOutput = 1;

// double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

// Neo into a versa 45:1 -> sprocket reduction of 26:14 -> sprocket reduction of 60:14.
constexpr double kAxis1OverallReduction = (9.0 / 1.0) * (5.0 / 1.0) * (26.0 / 14.0) * (60.0 / 14.0);
constexpr double kAxis1MotorRotationsPerDegree = 360 / kAxis1OverallReduction;
constexpr double kAxis1ZeroAngle = 0;

// Axis 2 (Extension)
constexpr double kAxis2P = 0.03;
constexpr double kAxis2I = 0;
constexpr double kAxis2D = 0;
constexpr double kAxis2FF = 0;
constexpr double kAxis2MinOutput = -1;
constexpr double kAxis2MaxOutput = 1;

// Neo 550 into versa 9:1.
constexpr double kAxis2OverallReduction = (4.0 / 1.0) * (3.0 / 1.0);
// Belt is running on an 18t 5mm HTD pulley.
constexpr double kAxis2MotorRotationsPerInchOfTravel =
  kAxis2OverallReduction / ((18.0 * 5.0) / 25.4);

// Axis 3 (Wrist)
constexpr double kAxis3P = 0.05;
constexpr double kAxis3I = 0;
constexpr double kAxis3D = 0;
constexpr double kAxis3FF = 0;
constexpr double kAxis3MinOutput = -1;
constexpr double kAxis3MaxOutput = 1;

}  // namespace ExtendingArmConstants

namespace DoubleJointedArmConstants {
// Motor controller CAN ID's.
constexpr int kDoubleJointedArmAxis1MotorControllerCanId = 11;
constexpr int kDoubleJointedArmAxis2MotorControllerCanId = 12;
constexpr int kDoubleJointedArmAxis3MotorControllerCanId = 13;

// PID constants for the arm axis's.
// Axis 1 (Shoulder)
constexpr double kAxis1P = 0.1;
constexpr double kAxis1I = 0;
constexpr double kAxis1D = 0;
constexpr double kAxis1FF = 0;
constexpr double kAxis1MinOutput = -1;
constexpr double kAxis1MaxOutput = 1;

// Neo into a versa 45:1 > 18t pulley on an arc that would encompase 191.5 teeth over 360deg.
constexpr double kAxis1OverallReduction = (5.0 / 1.0) * (4.0 / 1.0) * (191.5 / 18.0);
constexpr double kAxis1MotorRotationsPerDegree = kAxis1OverallReduction / 360;
constexpr double kAxis1ZeroAngle = 0;

// Axis 2 (Extension)
constexpr double kAxis2P = 0.1;
constexpr double kAxis2I = 0;
constexpr double kAxis2D = 0;
constexpr double kAxis2FF = 0;
constexpr double kAxis2MinOutput = -1;
constexpr double kAxis2MaxOutput = 1;

// Neo into versa 45:1 running a 60:14 chain reduction on the shoulder joint.
constexpr double kAxis2OverallReduction = (9.0 / 1.0) * (5.0 / 1.0) * (60.0 / 14.0);
constexpr double kAxis2MotorRotationsPerDegree = kAxis2OverallReduction / 360;
constexpr double kAxis2ZeroAngle = 0;

// Axis 3 (Wrist)
constexpr double kAxis3P = 0.01;
constexpr double kAxis3I = 0.000005;
constexpr double kAxis3D = 0;
constexpr double kAxis3FF = 0;
constexpr double kAxis3MinOutput = -1;
constexpr double kAxis3MaxOutput = 1;

// Neo 550 into a 9:1 versa running a 48:18 belt reduction.
constexpr double kAxis3OverallReduction = (9.0 / 1.0) * (4.1 / 1.0) * (48.0 / 18.0);
constexpr double kAxis3MotorRotationsPerDegree = kAxis3OverallReduction / 360;
}  // namespace DoubleJointedArmConstants