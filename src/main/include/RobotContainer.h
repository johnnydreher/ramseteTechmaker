// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;

private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...
  frc2::Command *m_HomeCommand = nullptr;
  frc2::Command *Home();

  // The robot's subsystems

  frc2::InstantCommand m_driveHalfSpeed{[this] { m_drive.SetMaxOutput(0.7); },
                                        {}};
  frc2::InstantCommand m_driveFullSpeed{[this] { m_drive.SetMaxOutput(1); },
                                        {}};
  frc2::InstantCommand m_shooterToggle{[this] { m_shooter.Set(m_shooter.Get() > 0 ? 0 : ShooterConstants::kMaxSpeed); },
                                       {}};
  frc2::InstantCommand m_TargetToggle{[this] { m_shooter.ToggleTarget(); },
                                      {}};
  frc2::InstantCommand m_IntakeToggle{[this] { m_shooter.ToggleIntake(); },
                                      {}};
  frc2::InstantCommand m_ShooterOn{[this] { m_shooter.Shoot(true); },
                                   {}};
  frc2::InstantCommand m_ShooterOff{[this] { m_shooter.Shoot(false); },
                                    {}};
  frc2::InstantCommand m_ResetOdometry{[this] { m_drive.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg))); },
                                       {}};
  frc2::InstantCommand m_Home{[this] {  m_HomeCommand = Home(); if (m_HomeCommand != nullptr) m_HomeCommand->Schedule();},
                                       {}};
  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command *> m_chooser;

  void ConfigureButtonBindings();
};
