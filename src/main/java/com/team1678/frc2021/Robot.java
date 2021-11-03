// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team1678.frc2021;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1678.frc2021.auto.AutoModeBase;
import com.team1678.frc2021.loops.Looper;
import com.team1678.frc2021.subsystems.*;
import com.team1678.frc2021.subsystems.superstructure.Superstructure;
import com.team1678.frc2021.controlboard.ControlBoard;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team2910.lib.math.RigidTransform2;
import com.team2910.lib.robot.UpdateManager;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    public static CTREConfigs ctreConfigs;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    public static Superstructure superstructure;

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    //private final Hood mHood = Hood.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();

    public RobotContainer robotContainer;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    /**
     * This function is called every 3 packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the 3's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();

        SmartDashboard.putString("LEDs State", mLEDs.getState().name());
    }

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

        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                    mLEDs,
                    //mHood,
                    mIntake,
                    mShooter,
                    mCanifier,
                    mLimelight,
                    mSuperstructure
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mRobotState.reset(Timer.getFPGATimestamp(), RigidTransform2.identity());

            mLimelight.setLed(Limelight.LedMode.OFF);

        } catch (Exception ini) {
            CrashTracker.logThrowableCrash(ini);
            throw ini;
        }
    }

    /**
     * This function is called once each time the 3 enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        mEnabledLooper.stop();

        Swerve.getInstance().zeroGyro();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), RigidTransform2.identity());

        mDisabledLooper.start();
        //mHood.setNeutralMode(NeutralMode.Coast);
        mLimelight.writePeriodicOutputs();
        mLEDs.conformToState(LEDs.State.RAINBOW);
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        // mLimelight.setStream(2);

        try {
            mLimelight.setLed(Limelight.LedMode.OFF);
            mLimelight.writePeriodicOutputs();

            if (!mLimelight.limelightOK()) {
                mLEDs.conformToState(LEDs.State.EMERGENCY);
            //} else if (mHood.isHoming()) {
            //    mLEDs.conformToState(LEDs.State.RAINBOW);
            } else {
                mLEDs.conformToState(LEDs.State.BREATHING_PINK);
            }

            mLEDs.writePeriodicOutputs();

        } catch (Exception t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
    /*
    Autonomous code which is not working right now.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        //mLimelight.setLed(Limelight.LedMode.ON);

        if (!mLimelight.limelightOK()) {
            mLEDs.conformToState(LEDs.State.EMERGENCY);
        } else {
            mLEDs.conformToState(LEDs.State.ENABLED);
        }

        try {

        } catch (Exception t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            mEnabledLooper.start();
            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.kPortPipeline);
            //mHood.setNeutralMode(NeutralMode.Brake);
            mLEDs.conformToState(LEDs.State.ENABLED);

            mControlBoard.reset();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        try{
            double timestamp = Timer.getFPGATimestamp();
            double hood_jog = mControlBoard.getJogHood();

            if (!mLimelight.limelightOK()) {
                mLEDs.conformToState(LEDs.State.EMERGENCY);
            } else {
                mLEDs.conformToState(LEDs.State.ENABLED);
            }

            mSuperstructure.enableIndexer(true);

            // TODO link to the buttons, manual zoom is indexer in god mode
            //mSuperstructure.setWantUnjam();
            //mSuperstructure.setManualZoom();

            mControlBoard.setRumble(mSuperstructure.getWantShoot());
            mSuperstructure.setWantHoodScan(mControlBoard.getWantHoodScan());

            mSuperstructure.setWantAutoAim();



        } catch (Exception t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
