package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.Util.LogData;
import com.swervedrivespecialties.exampleswerve.commands.DriveAtPercent;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import com.swervedrivespecialties.exampleswerve.commands.TestAutoDrive;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static RobotContainer robotContainer;
    private static LogData _logData;
    private static DrivetrainSubsystem drivetrain;

    @Override
    public void robotInit() {
        robotContainer = RobotContainer.getInstance();
        drivetrain = DrivetrainSubsystem.getInstance();
        _logData = new LogData();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit(){
        _logData.log(true, false, "Velocity", drivetrain.getKinematicVelocity().length);
    }

    @Override
    public void autonomousPeriodic(){
        _logData.log(false, false, "Velocity", drivetrain.getKinematicVelocity().length);
    }

    @Override
    public void disabledInit(){
        _logData.log(false, true, "Velocity", drivetrain.getKinematicVelocity().length);
    }
}
