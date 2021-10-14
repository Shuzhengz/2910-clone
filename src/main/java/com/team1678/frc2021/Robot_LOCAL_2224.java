/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
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
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }


}
