/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021;

import com.team1323.io.Xbox;
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.robot.UpdateManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private RobotContainer robotContainer;
  private UpdateManager updateManager;
  private Swerve swerve;
  private Xbox operator;
  private DriverStation ds;

  @Override
  public void robotInit() {
      // TODO: Initialise hood and do hood setpoint login in Superstructure.
      swerve = Swerve.getInstance();
      robotContainer = new RobotContainer();
      updateManager = new UpdateManager(
              robotContainer.getDrivetrainSubsystem()
      );
      updateManager.startLoop(5.0e-3);
  }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

}
