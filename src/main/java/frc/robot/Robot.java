// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  static String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public void AlliancePiplinesetter() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.setPipelineIndex(Constants.Limelight.kName, 0);
    } else {
      LimelightHelpers.setPipelineIndex(Constants.Limelight.kName, 1);
    }
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    RobotContainer.swerve.forceZeroGyro();

    m_chooser.setDefaultOption(Constants.AutoNames.kDefaultAuto, Constants.AutoNames.kDefaultAuto);
    m_chooser.addOption(Constants.AutoNames.ktuning, Constants.AutoNames.ktuning);
    m_chooser.addOption(Constants.AutoNames.ktuningiki, Constants.AutoNames.ktuningiki);
    SmartDashboard.putData("Otonom Secici", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_autoSelected = m_chooser.getSelected();
    AlliancePiplinesetter();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}