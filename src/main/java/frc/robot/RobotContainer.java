// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  public static Swerve swerve = new Swerve();

  public final Joystick driver = new Joystick(Constants.kControls.DRIVE_JOYSTICK_ID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.drive(
        () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(driver.getRawAxis(1)),
        () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(driver.getRawAxis(0)),
        () -> -Constants.kControls.THETA_DRIVE_LIMITER.calculate(driver.getRawAxis(2)),
        true,
        true));

    new JoystickButton(driver, 10).onTrue(swerve.zeroGyroCommand());
  }

  PathPlannerPath kTuning = PathPlannerPath.fromPathFile("Tuning");
  PathPlannerPath kTuningiki = PathPlannerPath.fromPathFile("Tuning_Back");

  public Command getAutonomousCommand() {
    if (Robot.m_autoSelected == Constants.AutoNames.ktuning) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
        swerve.resetOdometry(kTuning.getPreviewStartingHolonomicPose());
      } else {
        swerve.resetOdometry(kTuning.flipPath().getPreviewStartingHolonomicPose());
      }
      return AutoBuilder.followPath(kTuning);
    } else if (Robot.m_autoSelected == Constants.AutoNames.ktuningiki) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
        swerve.resetOdometry(kTuningiki.getPreviewStartingHolonomicPose());
      } else {
        swerve.resetOdometry(kTuningiki.flipPath().getPreviewStartingHolonomicPose());
      }
      return AutoBuilder.followPath(kTuningiki);
    } else {
      return null;
    }
  }
}