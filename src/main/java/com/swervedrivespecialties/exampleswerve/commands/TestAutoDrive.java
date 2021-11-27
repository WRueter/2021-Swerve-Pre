// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervedrivespecialties.exampleswerve.commands;

import java.util.Optional;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAutoDrive extends CommandBase {
  /** Creates a new TestAutoDrive. */

  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();

  double lastTimestamp;
  double timestamp;
  double dt;

  private static final PidConstants FOLLOWER_TRANSLATION_PID = new PidConstants(0.0, 0.0, 0.0);
  private static final PidConstants FOLLOWER_ROTATION_PID = new PidConstants(0.0, 0.0, 0.0);
  private static final HolonomicFeedforward FOLLOWER_HOLONOMIC_FEEDFORWARD = new HolonomicFeedforward(
          new DrivetrainFeedforwardConstants(1/(14.0 * 12.0), 0.0, 0.0)
  );

  private Trajectory testTrajectory;
  public TestAutoDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drive);
    Path sixFtLine = new Path(Rotation2.ZERO);
    sixFtLine.addSegment(new PathLineSegment(
    new Vector2(0.0, 0.0),
    new Vector2(120, 0.0)));
    testTrajectory = new Trajectory(0.0, 0.0, sixFtLine, CONSTRAINTS);
  }

  public static final ITrajectoryConstraint[] CONSTRAINTS = {
    new MaxVelocityConstraint(12.0 * 12.0),
    new MaxAccelerationConstraint(13.0 * 12.0),
    new CentripetalAccelerationConstraint(25.0 * 12.0)
  };

  private HolonomicMotionProfiledTrajectoryFollower follower;




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    follower = new HolonomicMotionProfiledTrajectoryFollower(FOLLOWER_TRANSLATION_PID, FOLLOWER_ROTATION_PID, FOLLOWER_HOLONOMIC_FEEDFORWARD);

    follower.follow(testTrajectory);
    System.out.println("TEST AUTO DRIVE");
  }

  @Override
  public void execute(){
    updateTime();
    Optional<HolonomicDriveSignal> driveSignal = calculate();
    if (driveSignal.isPresent()){
      holonomicDrive(driveSignal.get());
    } else{
      holonomicDrive(new HolonomicDriveSignal(Vector2.ZERO, 0.0, true));
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return follower.getCurrentTrajectory().isEmpty();
  }

  private RigidTransform2 getPose(){
    return new RigidTransform2(_drive.getKinematicPosition(), _drive.getGyroAngle());
  }

  private void updateTime(){
    lastTimestamp = timestamp;
    timestamp = Timer.getFPGATimestamp();
    dt = timestamp - lastTimestamp;
  }

  private Optional<HolonomicDriveSignal> calculate(){
    return follower.update(getPose(), _drive.getKinematicVelocity(), _drive.getGyroRate(), timestamp, dt);
  }

  private void holonomicDrive(HolonomicDriveSignal sig){
    _drive.drive(new Translation2d(sig.getTranslation().x, sig.getTranslation().y), sig.getRotation(), sig.isFieldOriented());
  }

}
