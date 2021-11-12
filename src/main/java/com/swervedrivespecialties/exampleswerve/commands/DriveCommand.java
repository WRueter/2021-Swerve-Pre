// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervedrivespecialties.exampleswerve.commands;

import static com.swervedrivespecialties.exampleswerve.Constants.CHASSIS.*;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotContainer;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {

  private static RobotContainer rc = RobotContainer.getInstance();

  /** Creates a new DriveCommandNew. */
  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivetrainSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -rc.getLeftY(); //-Robot.getOi().getPrimaryJoystick().getRawAxis(1);
    forward = Utilities.deadband(forward);
    // Square the forward stick
    forward = SPEED_SCALE * Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = -rc.getLeftX();
    strafe = Utilities.deadband(strafe);
    // Square the strafe stick
    strafe = SPEED_SCALE * Math.copySign(Math.pow(strafe, 2.0), strafe);

    double rotation = -rc.getRightX();
    rotation = Utilities.deadband(rotation);
    // Square the rotation stick
    rotation = SPEED_SCALE * Math.copySign(Math.pow(rotation, 2.0), rotation);

    DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      System.out.println("Drive Command Interrupted");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
