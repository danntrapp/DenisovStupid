// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/simulation/LinearSystemSim.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <numbers>



namespace DriveConstants {
inline constexpr int kLeftMotor1Port = 0;
inline constexpr int kLeftMotor2Port = 1;
inline constexpr int kRightMotor1Port = 2;
inline constexpr int kRightMotor2Port = 3;

inline constexpr int kLeftEncoderPorts[]{0, 1};
inline constexpr int kRightEncoderPorts[]{2, 3};
inline constexpr bool kLeftEncoderReversed = false;
inline constexpr bool kRightEncoderReversed = true;

inline constexpr auto kTrackwidth = 0.69_m;
const auto kMoment = 7_kg_sq_m;
constexpr auto kWheelDiameter = 6_in;
constexpr auto kWeight = 70_lb;

inline constexpr int kEncoderCPR = 1024;
inline constexpr auto kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameter * std::numbers::pi) / static_cast<double>(kEncoderCPR);

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
inline constexpr auto ks = 0.22_V;
inline constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
inline constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

inline constexpr double kVelError = 0.1; // 0.1 m/s
inline constexpr double kRotError = 0.001; //0.001 rad
inline constexpr double kPosError = 0.001; //0.001 m
inline constexpr double kSidePosError = 0.005; //0.005 m

inline constexpr std::array<double, 7> kStdDev = {
  kPosError, kPosError, kRotError, kVelError, 
  kVelError, kSidePosError, kSidePosError};
// Example value only - as above, this must be tuned for your drive!
inline constexpr double kPDriveVel = 8.5;
}  // namespace DriveConstants


class DrivetrainSimulation {
public:
  DrivetrainSimulation(Drivetrain &drivetrain)
      : m_driveSim{
          frc::DCMotor::NEO(2), 7.29,
          DriveConstants::kTrackwidth, DriveConstants::kMoment,
          DriveConstants::kWeight, DriveConstants::kWheelDiameter, 
          DriveConstants::kStdDev}, 
        m_gyroYaw(HALSIM_GetSimValueHandle(
            HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw")),
        m_poseSim(
          drivetrain.GetHeading(), drivetrain.GetEncoderDistance(drivetrain.m_leftEncoder),
          drivetrain.GetEncoderDistance(drivetrain.m_rightEncoder)) {}

public:
  hal::SimDouble m_gyroYaw;
  frc::sim::DifferentialDrivetrainSim m_driveSim;
  frc::DifferentialDriveOdometry m_poseSim;
};

Drivetrain::Drivetrain()
    : m_left1{DriveConstants::kLeftMotor1Port, rev::CANSparkMax::MotorType::kBrushless},
      m_left2{DriveConstants::kLeftMotor2Port, rev::CANSparkMax::MotorType::kBrushless},
      m_right1{DriveConstants::kRightMotor1Port, rev::CANSparkMax::MotorType::kBrushless},
      m_right2{DriveConstants::kRightMotor2Port, rev::CANSparkMax::MotorType::kBrushless},
      m_leftEncoder{DriveConstants::kLeftEncoderPorts[0], DriveConstants::kLeftEncoderPorts[1]},
      m_rightEncoder{DriveConstants::kRightEncoderPorts[0], DriveConstants::kRightEncoderPorts[1]},
      m_odometry{m_gyro.GetRotation2d(), units::meter_t{0}, units::meter_t{0}},
      m_driveKinematics{DriveConstants::kTrackwidth},
      m_sim_state(new DrivetrainSimulation (*this)) {
  wpi::SendableRegistry::AddChild(&m_drive, &m_left1);
  wpi::SendableRegistry::AddChild(&m_drive, &m_right1);
  

  m_left1.SetInverted(true);

  m_left2.Follow(m_left1);
  m_right2.Follow(m_right2);



  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.

  // Set the distance per pulse for the encoders
  m_leftEncoder.SetDistancePerPulse(DriveConstants::kEncoderDistancePerPulse.value());
  m_rightEncoder.SetDistancePerPulse(DriveConstants::kEncoderDistancePerPulse.value());

  ResetEncoders();
}

void Drivetrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
}

void Drivetrain::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_left1.SetVoltage(left);
  m_right1.SetVoltage(right);
  m_drive.Feed();
}

void Drivetrain::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

double Drivetrain::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
}

void Drivetrain::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t Drivetrain::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

double Drivetrain::GetTurnRate() {
  return -m_gyro.GetRate();
}

frc::Pose2d Drivetrain::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  return {units::meters_per_second_t{m_leftEncoder.GetRate()},
          units::meters_per_second_t{m_rightEncoder.GetRate()}};
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           units::meter_t{m_leftEncoder.GetDistance()},
                           units::meter_t{m_rightEncoder.GetDistance()}, pose);
}

void Drivetrain::SimulationPeriodic() {
  if (!m_sim_state)
    return;

  // Assume perfect kinematics and get the new gyro angle
  const auto chassis_speed = m_driveKinematics.ToChassisSpeeds(GetWheelSpeeds());
  const auto theta = m_sim_state->m_poseSim.GetPose().Rotation();
  const auto new_theta =
      theta.RotateBy(units::radian_t{chassis_speed.omega * 20_ms});
  // robot nav x defines clockwise as positive instead of counterclockwise
  m_sim_state->m_gyroYaw.Set(-new_theta.Degrees().value());

  // Feed this simulated gyro angle into the odometry to get simulated position
  m_sim_state->m_poseSim.Update(new_theta, GetEncoderDistance(m_leftEncoder), GetEncoderDistance(m_rightEncoder));

  m_field.GetObject("simulation")->SetPose(m_sim_state->m_poseSim.GetPose());
}
