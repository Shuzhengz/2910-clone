package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    private static double kIntakingVoltage = 3.5;
    private static double kIdleVoltage = 0;
    private static double mCurrent;

    private static Intake mInstance;
    private TimeDelayedBoolean mIntakeSolenoidTimer = new TimeDelayedBoolean();

    private Solenoid mDeploySolenoid;

    public enum WantedAction {
        NONE, INTAKE, RETRACT, STAY_OUT, BARF,
    }

    public enum State {
        IDLE, INTAKING, RETRACTING, STAYING_OUT, BARFING
    }

    private State mState = State.IDLE;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private final TalonFX mMaster;
    private final TalonFX mSlave;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterIntakeRollerId);
        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kDeploySolenoidId);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kSlaverIntakeRollerId, Constants.kMasterIntakeRollerId);
        mSlave.setInverted(true);
    }

    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
        // Zero sensors?
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    runStateMachine();

                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    public void runStateMachine() {
        switch (mState) {
            case INTAKING:
                if (mPeriodicIO.intake_out) {
                    mPeriodicIO.demand = kIntakingVoltage;
                } else {
                    mPeriodicIO.demand = 0.0;
                }
                mPeriodicIO.deploy = true;
                break;
            case RETRACTING:
                if (mPeriodicIO.intake_out) {
                    mPeriodicIO.demand = -kIntakingVoltage;
                } else {
                    mPeriodicIO.demand = 0.0;
                }
                mPeriodicIO.deploy = true;
                break;
            case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                mPeriodicIO.deploy = false;
                break;
            case STAYING_OUT:
                mPeriodicIO.demand = 0;
                mPeriodicIO.deploy = true;
                break;
            case BARFING:
                mPeriodicIO.demand = -kIntakingVoltage;
                mPeriodicIO.deploy = true;
                break;
        }
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public double getCurrent() {
        return mCurrent;
    }

    public synchronized State getState() {
        return mState;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
            case NONE:
                mState = State.IDLE;
                break;
            case INTAKE:
                mState = State.INTAKING;
                break;
            case RETRACT:
                mState = State.RETRACTING;
                break;
            case STAY_OUT:
                mState = State.STAYING_OUT;
                break;
            case BARF:
                mState = State.BARFING;
                break;
        }

    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.intake_out = mIntakeSolenoidTimer.update(mPeriodicIO.deploy, 0.2);
        mCurrent = mPeriodicIO.current;
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mDeploySolenoid.set(mPeriodicIO.deploy);
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putString("Intake State", mState.toString());
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;
        public boolean intake_out;

        // OUTPUTS
        public double demand;
        public boolean deploy;
    }
}

