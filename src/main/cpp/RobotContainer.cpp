// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();

    // Set up default drive command
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_drive.ArcadeDrive(-m_driverController.GetY(lHand),
                                m_driverController.GetX(rHand));
        },
        {&m_drive}));
    
}

void RobotContainer::ConfigureButtonBindings()
{
    // Configure your button bindings here
frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kX)
                .WhenPressed(&m_shooterToggle);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperRight)
        .WhenPressed(&m_driveFullSpeed)
        .WhenReleased(&m_driveHalfSpeed);
     frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kA)
        .WhenPressed(&m_ShooterOn)
        .WhenReleased(&m_ShooterOff);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kBumperLeft)
        .WhenPressed(&m_TargetToggle);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kB)
        .WhenPressed(&m_IntakeToggle);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kY)
        .WhenPressed(&m_Home);
    frc2::JoystickButton(&m_driverController, (int)frc::XboxController::Button::kStart)
        .WhenPressed(&m_ResetOdometry);        
}

frc2::Command *RobotContainer::GetCaminhoA()
{
    // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics, 10_V);

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                 AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(DriveConstants::kDriveKinematics);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
         frc::Translation2d(7_m, 0_m), 
         frc::Translation2d(7_m, 3_m)
         },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(1_m, 3_m, frc::Rotation2d(180_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel,DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel,DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    m_shooter.SetAutonomous(true);
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
    
}
frc2::Command *RobotContainer::GetCaminhoB()
{
    // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics, 10_V);

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                 AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(DriveConstants::kDriveKinematics);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
         frc::Translation2d(5.7_m, 0_m), 
         frc::Translation2d(5.7_m, 3_m),
          frc::Translation2d(1_m, 3_m), 
         frc::Translation2d(1_m, 1.5_m)
         },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(8_m, 1.5_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel,DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel,DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    m_shooter.SetAutonomous(true);
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
    
}
frc2::Command *RobotContainer::GetCaminhoC()
{
    // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics, 10_V);

    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                 AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(DriveConstants::kDriveKinematics);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(90_deg)),
        // Pass through these two interior waypoints, making an 's' curve path
        {
         frc::Translation2d(0.5_m, 1.5_m), 
         frc::Translation2d(2_m, 3.5_m),
         frc::Translation2d(3.5_m, 3.5_m),
          frc::Translation2d(3.5_m, 1.5_m), 
         frc::Translation2d(6_m, 0_m),
         frc::Translation2d(7.5_m, 0_m)
         },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(8_m, 3_m, frc::Rotation2d(90_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel,DriveConstants::kDDriveVel),
        frc2::PIDController(DriveConstants::kPDriveVel, DriveConstants::kIDriveVel,DriveConstants::kDDriveVel),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_drive.ResetOdometry(exampleTrajectory.InitialPose());
    m_shooter.SetAutonomous(true);
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
    
}
frc2::Command *RobotContainer::Home()
{
    // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics, 10_V);

    // Set up config for trajectory
    frc::TrajectoryConfig config(2_mps,1_mps_sq);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(DriveConstants::kDriveKinematics);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        m_drive.GetPose(),
        // Pass through these two interior waypoints, making an 's' curve path
        {     
            frc::Translation2d(1_m, 1_m)
         },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d(0.5_m, 0_m, frc::Rotation2d(0_deg)),
        // Pass the config
        config);

    frc2::RamseteCommand ramseteCommand(
        exampleTrajectory, [this]() { return m_drive.GetPose(); },
        frc::RamseteController(AutoConstants::kRamseteB,
                               AutoConstants::kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics,
        [this] { return m_drive.GetWheelSpeeds(); },
        frc2::PIDController(DriveConstants::kPDriveVel, 0,0),
        frc2::PIDController(DriveConstants::kPDriveVel, 0,0),
        [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
        {&m_drive});

    // Reset odometry to the starting pose of the trajectory.
    m_shooter.SetAutonomous(true);
    // no auto
    return new frc2::SequentialCommandGroup(
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
    
}
