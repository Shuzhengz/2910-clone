// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team1678.frc2021;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.TimedRobot;
=======
import com.team1678.frc2021.subsystems.Swerve;
import com.team2910.lib.robot.UpdateManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
>>>>>>> swerve

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
<<<<<<< HEAD
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // TODO: Initialise hood and do hood setpoint login in Superstructure.
    swerve = Swerve.getInstance();
    robotContainer = new RobotContainer();
    updateManager = new UpdateManager(
        robotContainer.getDrivetrainSubsystem()
      );
      updateManager.startLoop(5.0e-3);

    // instantiate subsystems\
		mIntake = Intake.getInstance();
		mSuperstructure = Superstructure.getInstance();
		mShooter = Shooter.getInstance();
		mIndexer = Indexer.getInstance();
		mHood = Hood.getInstance();	
		mLimelight = Limelight.getInstance();
        
    subsystems = new SubsystemManager(
    //Arrays.asList(swerve, Intake, mSuperstructure, mIndexer/*, leds*/));
		Arrays.asList(/*mLEDs,*/
			mHood,
			mLimelight,
			mIntake,
			mIndexer,
		  	mShooter,
			mSuperstructure,
			mInfrastructure
			));

		Logger.clearLog();
		
		operator = new Xbox(1);

        enabledLooper.register(QuinticPathTransmitter.getInstance());
        enabledLooper.register(LimelightProcessor.getInstance());
        disabledLooper.register(QuinticPathTransmitter.getInstance());
        disabledLooper.register(LimelightProcessor.getInstance());
        subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);
		//CommandScheduler.getInstance().registerSubsystem(swerve);

        // swerve.zeroSensors();
        // swerve.zeroSensors(new Pose2d());
		// swerve.stop();
		swerve.startLogging();
        smartDashboardInteractions.initWithDefaults();


        // generator.generateTrajectories();
=======
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  public RobotContainer robotContainer;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the 3 is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
      // TODO: Initialise hood and do hood setpoint login in Superstructure.
      robotContainer = new RobotContainer();
>>>>>>> swerve
  }

  /**
   * This function is called every 3 packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
<<<<<<< HEAD
  public void autonomousInit() {
=======
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the 3's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
>>>>>>> swerve
  }

  /** This function is called once each time the 3 enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
<<<<<<< HEAD
  public void autonomousPeriodic() {
=======
  public void autonomousInit() {
    /*
    Autonomous code which is not working right now.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
>>>>>>> swerve
  }

  /** This function is called periodically during autonomous. */
  @Override
<<<<<<< HEAD
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
=======
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
>>>>>>> swerve
  }

  /** This function is called periodically during operator control. */
  @Override
<<<<<<< HEAD
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }


}
=======
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
>>>>>>> swerve
