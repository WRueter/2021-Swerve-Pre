// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import com.swervedrivespecialties.exampleswerve.commands.TestAutoDrive;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer instance;

    private XboxController primaryController = new XboxController(0);

    private JoystickButton _a = new JoystickButton(primaryController, XboxController.Button.kA.value);
    private JoystickButton _y = new JoystickButton(primaryController, XboxController.Button.kY.value);

    private RobotContainer() {

        configureButtonBindings();
        configureSubsystemDefaultCommands();

    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }

        return instance;
    }

    private void configureButtonBindings() {
        _a.whenPressed(() -> DrivetrainSubsystem.getInstance().resetGyroscope());
        _y.whenPressed(new TestAutoDrive());
    }

    private void configureSubsystemDefaultCommands(){
        DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand());
    }

    public double getLeftX(){
        return primaryController.getRawAxis(XboxController.Axis.kLeftX.value);
    }
    public double getLeftY(){
        return primaryController.getRawAxis(XboxController.Axis.kLeftY.value);
    }
    public double getRightX(){
        return primaryController.getRawAxis(XboxController.Axis.kRightX.value);
    }
    public double getRightY(){
        return primaryController.getRawAxis(XboxController.Axis.kRightY.value);
    }


}
