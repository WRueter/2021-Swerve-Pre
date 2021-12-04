// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveAtPercent extends CommandBase {
  /** Creates a new DriveAtPercent. */
  private Translation2d fwd;
  public DriveAtPercent(double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivetrainSubsystem.getInstance());
    fwd = new Translation2d(percent, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DrivetrainSubsystem.getInstance().drive(fwd, 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DrivetrainSubsystem.getInstance().drive(fwd, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
