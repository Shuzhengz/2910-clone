package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.channels.ReadPendingException;

public class Indexer extends Subsystem {

    // Variable Declarations
    private static Indexer mInstance = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Slot Proxies
    private final DigitalInput mLowerBeamBreak = new DigitalInput(Constants.kLowerBeamBreak);
    private final DigitalInput mUpperBeamBreak = new DigitalInput(Constants.kUpperBeamBreak);

    // Slot state arrays
    private boolean mSlotsClean;

    private int mBallCount = 0;

    private int mCurrentSlot;
    private final TalonFX mMaster;

    private boolean mIntakeHasBall;
    private boolean mIntakeCanPass = false;
    private boolean mIntakeReverse = false;
    private boolean mShooterNeedShoot = false;

    private State mState = State.IDLE;

    public enum WantedAction {
        NONE, PREP, ZOOM, BARF, FEED, PASS_THROUGH,
    }

    public enum State {
        IDLE, PREPPING, ZOOMING, BARFING, FEEDING, PASSING_THROUGH,
    }

    /**
     * The Indexer Utility Class
     */
    private Indexer() {

        // Set up TalonFX for the Falcon500
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mMaster.config_kP(0, Constants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kIndexerKf, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(1, Constants.kIndexerVelocityKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(1, Constants.kIndexerVelocityKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(1, Constants.kIndexerVelocityKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(1, Constants.kIndexerVelocityKf, Constants.kLongCANTimeoutMs);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kIndexerMaxVelocity);
        mMaster.configMotionAcceleration(Constants.kIndexerMaxAcceleration);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.0);
    }

    /**
     * Returns the Indexer velocity
     *
     * @return Indexer velocity
     */
    public double getIndexerDemand() {
        double rValue = 0;
        try {
            if (mPeriodicIO.indexer_control_mode == ControlMode.PercentOutput) {
                rValue = mPeriodicIO.indexer_demand;
            }
        } catch (ReadPendingException readError) {
            System.out.println("Unable to read Indexer Velocity");
        }
        return rValue;
    }

    /**
     * Spins the motor, in the percentage output control mode
     *
     * @param voltage the voltage you want to spin it at, in percent output mode
     */
    private void spinMotor(double voltage) {
        mPeriodicIO.indexer_demand = voltage;
    }

    /**
     * Checks if the slots are filled
     *
     * @return Are the slots filled
     */
    public synchronized boolean slotsFilled() {
        return mSlotsClean;
    }

    /**
     * Checks if the indexer should Index
     *
     * @return if it should index
     */
    public boolean shouldIndex() {
        if (slotsFilled()) {
            return false;
        }
        return isBallAtIntake();
    }

    /**
     * Checks if the slots are empty
     *
     * @return Are the slots empty
     */
    public synchronized boolean slotsEmpty() {
        return !mSlotsClean;
    }

    /**
     * Gets the instant of the Indexer
     *
     * @return Indexer Instance
     */
    public static synchronized Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    /**
     * Updates the slot status
     */
    private void updateSlots() {
        mIntakeHasBall = mPeriodicIO.beamBreakSensor[0];
        mSlotsClean = mPeriodicIO.beamBreakSensor[1];
        if (mBallCount != 0) {
            mSlotsClean = false;
        }
    }

    /**
     * Checks if the lower beambreak sensor has a ball
     *
     * @return ic the lower beambreak has a ball
     */
    private boolean isBallAtIntake() {
        return mIntakeHasBall;
    }

    /**
     * Sets the percentage for the open loop, percent output mode for the indexer
     *
     * @param percentage The Percentage to set the open loop at
     */
    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.indexer_control_mode = ControlMode.PercentOutput;
        mPeriodicIO.indexer_demand = percentage;
    }

    public synchronized boolean getIntakeCanPass() {
        return mIntakeCanPass;
    }

    public synchronized boolean getIntakeReverse() {
        return mIntakeReverse;
    }

    public synchronized boolean getShooterNeedsShoot() {
        return mShooterNeedShoot;
    }

    /**
     * Stops the open loop
     */
    @Override
    public void stop() {
        setOpenLoop(0);
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Checks the system
     *
     * @return true
     */
    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * Zeros the encoder on the Falcon500 for the Indexer
     */
    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
    }

    /**
     * Registers the enabled loops
     *
     * @param enabledLooper the enabled ILooper
     */
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                // Things to do on stop
                mState = State.IDLE;
                stop();
            }
        });
    }

    public void runStateMachine() {
        updateSlots();
        switch (mState) {
            case IDLE:
                mIntakeCanPass = false;
                mIntakeReverse = false;
                mShooterNeedShoot = false;
                spinMotor(0);
                break;
            case PREPPING:
                mShooterNeedShoot = false;
                mIntakeReverse = false;
                mIntakeCanPass = false;
                if (slotsEmpty()) {
                    spinMotor(Constants.kIndexingVoltage);
                } else if (!slotsFilled() && mIntakeHasBall) {
                    spinMotor(Constants.kZoomingVoltage);
                } else {
                    boolean atIntake = false;
                    spinMotor(-Constants.kIndexingVoltage);
                    while (!atIntake) {
                        atIntake = mLowerBeamBreak.get();
                    }
                    spinMotor(0);
                    mState = State.FEEDING;
                }
                break;
            case ZOOMING:
                mShooterNeedShoot = false;
                mIntakeReverse = false;
                if (slotsEmpty()) {
                    spinMotor(Constants.kZoomingVoltage);
                    mShooterNeedShoot = true;
                    mIntakeCanPass = true;
                } else if (slotsEmpty() && mIntakeHasBall) {
                    spinMotor(Constants.kZoomingVoltage);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ex) {
                        Thread.currentThread().interrupt();
                    }
                    mShooterNeedShoot = true;
                    mIntakeCanPass = true;
                } else {
                    mShooterNeedShoot = true;
                    mIntakeCanPass = true;
                    spinMotor(Constants.kZoomingVoltage);
                }
                break;
            case BARFING:
                spinMotor(Constants.kBarfingVoltage);
                mIntakeReverse = true;
                mShooterNeedShoot = false;
                break;
            case FEEDING:
                //TODO complete
                mIntakeReverse = false;
                mIntakeCanPass = true;
                mShooterNeedShoot = false;
                if (mBallCount == 3) {
                    System.out.println("Indexer full");
                } else if (mBallCount < 3 && mBallCount > 0) {
                    spinMotor(Constants.kIndexingVoltage);
                }
                break;
            case PASSING_THROUGH:
                mIntakeReverse = false;
                mIntakeCanPass = true;
                mShooterNeedShoot = true;
                spinMotor(Constants.kZoomingVoltage);
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case PREP:
                mState = State.PREPPING;
                break;
            case ZOOM:
                mState = State.ZOOMING;
                break;
            case BARF:
                mState = State.BARFING;
                break;
            case FEED:
                mState = State.FEEDING;
                break;
            case PASS_THROUGH:
                mState = State.PASSING_THROUGH;
                break;
        }
    }

    /**
     * Sets the periodic inputs for the Indexer
     */
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.beamBreakSensor[0] = mLowerBeamBreak.get();  // Slot closest to the shooter
        mPeriodicIO.beamBreakSensor[1] = mUpperBeamBreak.get();
        mPeriodicIO.indexer_current = mMaster.getStatorCurrent();
    }

    /**
     * Sets the Indexer output to the Smart Dashboard on the Driverstation
     */
    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("IndexerControlMode", mPeriodicIO.indexer_control_mode.name());
        SmartDashboard.putNumber("IndexerDemand", mPeriodicIO.indexer_demand);
        SmartDashboard.putNumber("IndexerVelocity", mPeriodicIO.indexer_velocity);

        SmartDashboard.putNumber("BallsInIndexer", mBallCount);
        SmartDashboard.putBoolean("LowerBeamBreak", mPeriodicIO.beamBreakSensor[0]);
        SmartDashboard.putBoolean("UpperBeamBreak", mPeriodicIO.beamBreakSensor[1]);
        SmartDashboard.putBoolean("Snapped", mPeriodicIO.snapped);
    }

    /**
     * Sets the PeriodicIO needed for the Indexer
     */
    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        private boolean[] beamBreakSensor = {false, false};

        public double indexer_velocity;
        public double indexer_current;
        public boolean snapped;

        // OUTPUTS
        public boolean clearForIntake;
        public ControlMode indexer_control_mode = ControlMode.PercentOutput;
        public double indexer_demand;
    }
}
