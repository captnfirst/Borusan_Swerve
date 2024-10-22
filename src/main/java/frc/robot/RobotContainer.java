// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Swerve swerve = new Swerve();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final Joystick driver = new Joystick(Constants.kControls.DRIVE_JOYSTICK_ID);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.drive(
      () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(driver.getRawAxis(1)),
      () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(driver.getRawAxis(0)), 
      () -> -Constants.kControls.THETA_DRIVE_LIMITER.calculate(driver.getRawAxis(2)),
      true,
      true
    ));

    new JoystickButton(driver, 10).onTrue(swerve.zeroGyroCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}