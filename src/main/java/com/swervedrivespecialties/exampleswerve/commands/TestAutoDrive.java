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
import org.frcteam2910.common.control.PathArcSegment;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAutoDrive extends CommandBase {
  /** Creates a new TestAutoDrive. */

  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();

  double lastTimestamp;
  double timestamp;
  double dt;

  private Trajectory _traj;

  //--PID CONSTANTS--\\
  private static double kInterceptVoltage = 0.036368916875838035;//0.04422471929595917;//0.21757089186024456;//0.014857233341018905 * 5;
  private static double kPathFollowingVelocityFeedForward = 0.006653587782016155;//0.006626004633618817;//0.006096860014174834;
  private static double kPathFollowingAccelFeedForward = 0;//0.0007216306379892838;//0;
  // T = Path Following Translation PID Constant
  private static double kT_P = .00125 * 5.75;
  private static double kT_I = 0.0;
  private static double kT_D = kT_P*1.5;
  // R = Path Following Rotation PID Constant
  private static double kR_P = 0.0;
  private static double kR_I = 0.0;
  private static double kR_D = 0.0;
  //-----------------\\

  /*
  OLD (AND WRONG)--- 
  Velocity Feed Forward: 0.006096860014174834
  Acceleration Feed Forward: 0
  Static Feed Forward: 0.21757089186024456
  R-Squared: 0.9582200000000001
  Computation Time: 1407.1136412
  ------
  NEW (WITH ACCEL)--
  Velocity Feed Forward: 0.006626004633618817
  Acceleration Feed Forward: 0.0007216306379892838
  Static Feed Forward: 0.04422471929595917
  R-Squared: 0.9325800000000001
  Computation Time: 0.04967330000000003
  ------------------
  NEW (WITHOUT ACCEL)--
  Velocity Feed Forward: 0.006653587782016155
  Acceleration Feed Forward: 0
  Static Feed Forward: 0.036368916875838035
  R-Squared: 0.9915100000000001
  Computation Time: 1716.5102539
  ---------------------
  */

  private static final PidConstants FOLLOWER_TRANSLATION_PID = new PidConstants(kT_P, kT_I, kT_D);
  private static final PidConstants FOLLOWER_ROTATION_PID = new PidConstants(kR_P, kR_I, kR_D);
  private static final HolonomicFeedforward FOLLOWER_HOLONOMIC_FEEDFORWARD = new HolonomicFeedforward(
          new DrivetrainFeedforwardConstants(
            kPathFollowingVelocityFeedForward, 
            kPathFollowingAccelFeedForward, 
            kInterceptVoltage)
        );

  private Trajectory testTrajectory;
  public TestAutoDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drive);
    Path arc = new Path(Rotation2.ZERO);
    arc.addSegment(new PathArcSegment(new Vector2(0, 0), new Vector2(50,-50), new Vector2(35, -15)));
    Path sixFtLine = new Path(Rotation2.ZERO);
    sixFtLine.addSegment(new PathLineSegment(
    new Vector2(0.0, 0.0),
    new Vector2(120, 0.0)));
    testTrajectory = new Trajectory(0.0, 0.0, arc, CONSTRAINTS);
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
    if(follower.getCurrentTrajectory().isPresent()){
      _traj = follower.getCurrentTrajectory().get();
      SmartDashboard.putNumber("Pos Error", _traj.calculateSegment(_traj.getDuration()).translation.subtract(_drive.getKinematicPosition()).length);
      SmartDashboard.putNumber("Real Pos", _drive.getKinematicPosition().length);
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
