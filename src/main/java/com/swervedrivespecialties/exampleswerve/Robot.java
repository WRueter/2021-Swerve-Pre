package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static RobotContainer robotContainer;

    private static DrivetrainSubsystem drivetrain;

    @Override
    public void robotInit() {
        robotContainer = RobotContainer.getInstance();
        drivetrain = DrivetrainSubsystem.getInstance();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
