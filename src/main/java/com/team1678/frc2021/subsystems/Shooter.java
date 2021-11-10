package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2021.Constants;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final TalonFX mMasterOverhead;

    private boolean mRunningManual = false;
    private static double kUpperVelocityConversion = 75.0 / 512.0;
    private static double kMainVelocityConversion = 600.0 / 2048.0;
    private static double kShooterTolerance = 200.0;

    private Shooter() {
        //for flywheel motor
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterFlywheelID);
        mMasterOverhead = TalonFXFactory.createDefaultTalon(Constants.kOverheadFlywheelID);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kSlaveFlywheelID, Constants.kMasterFlywheelID);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.config_kP(0, Constants.kShooterP, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kShooterI, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kShooterD, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kShooterF, Constants.kLongCANTimeoutMs);
        mMaster.config_IntegralZone(0, (int) (200.0 / kUpperVelocityConversion));
        mMaster.selectProfileSlot(0, 0);

        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.02);
        mMaster.configSupplyCurrentLimit(curr_lim);

        mSlave.setInverted(false);

        //for overhead motor
        mMasterOverhead.set(ControlMode.PercentOutput, 1);
        mMasterOverhead.setInverted(false);
        mMasterOverhead.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMasterOverhead.enableVoltageCompensation(true);

        mMasterOverhead.config_kP(1, Constants.kShooterP, Constants.kLongCANTimeoutMs);
        mMasterOverhead.config_kI(1, Constants.kShooterI, Constants.kLongCANTimeoutMs);
        mMasterOverhead.config_kD(1, Constants.kShooterD, Constants.kLongCANTimeoutMs);
        mMasterOverhead.config_kF(1, Constants.kShooterF, Constants.kLongCANTimeoutMs);
        mMasterOverhead.config_IntegralZone(1, (int) (200.0 / kMainVelocityConversion));
        mMasterOverhead.selectProfileSlot(1, 0);

        mMasterOverhead.set(ControlMode.PercentOutput, 0);
        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configClosedloopRamp(0.2);
        mMasterOverhead.configClosedloopRamp(0.2);
    }

    public synchronized boolean spunUp() {
        if (mPeriodicIO.upper_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.upper_demand, mPeriodicIO.upper_velocity, kShooterTolerance);
        }
        return false;
    }

    public static synchronized Shooter mInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                //startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                //stopLogging();
            }
        });
    }

    public synchronized void setOpenLoop(double upper) {
        mPeriodicIO.upper_demand = upper;
        mRunningManual = true;
    }

    public synchronized double getDemand() {
        return mPeriodicIO.upper_demand;
    }

    public synchronized double getShooterRPM() {
        return mMaster.getSelectedSensorVelocity();
    }

    public synchronized double getVelocity() {
        return mPeriodicIO.upper_velocity;
    }

    //for main wheel
    public synchronized void setVelocityOverhead(double velocity) {
        mPeriodicIO.main_demand = velocity;
        mRunningManual = false;
    }

    public synchronized void setOpenLoopOverhead(double main) {
        mPeriodicIO.main_demand = main;
        mRunningManual = true;
    }

    public synchronized double getDemandOverhead() {
        return mPeriodicIO.main_demand;
    }

    public synchronized double getShooterRPMOverhead() {
        return mMasterOverhead.getSelectedSensorVelocity();
    }

    public synchronized boolean spunUpOverhead() {
        if (mPeriodicIO.main_demand > 0) {
            return Util.epsilonEquals(mPeriodicIO.main_demand, mPeriodicIO.main_velocity, kShooterTolerance);
        }
        return false;
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.upper_demand = velocity;
        mRunningManual = false;

    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        //for upper wheel
        mPeriodicIO.upper_velocity = mMaster.getSelectedSensorVelocity() * kUpperVelocityConversion;
        mPeriodicIO.upper_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.upper_current = mMaster.getStatorCurrent();
        mPeriodicIO.upper_temperature = mMaster.getTemperature();

        //for main wheel
        mPeriodicIO.main_velocity = mMasterOverhead.getSelectedSensorVelocity() * kMainVelocityConversion;
        mPeriodicIO.main_voltage = mMasterOverhead.getMotorOutputVoltage();
        mPeriodicIO.main_current = mMasterOverhead.getStatorCurrent();
        mPeriodicIO.main_temperature = mMasterOverhead.getTemperature();
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {

        //for upper wheel
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.upper_demand / kUpperVelocityConversion);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0);
        }

        //for main wheel
        if (!mRunningManual) {
            mMasterOverhead.set(ControlMode.Velocity, mPeriodicIO.main_demand / kMainVelocityConversion);
        } else {
            mMasterOverhead.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public synchronized void outputTelemetry() {

        //for upper wheel
        SmartDashboard.putNumber("Upper Wheel Velocity", mPeriodicIO.upper_velocity);
        SmartDashboard.putNumber("Upper Wheel Current", mPeriodicIO.upper_current);
        SmartDashboard.putNumber("Upper Wheel Goal", mPeriodicIO.upper_demand);
        SmartDashboard.putNumber("Upper Wheel Temperature", mPeriodicIO.upper_temperature);

        //for main wheel
        SmartDashboard.putNumber("Main Wheel Velocity", mPeriodicIO.main_velocity);
        SmartDashboard.putNumber("Main Wheel Current", mPeriodicIO.main_current);
        SmartDashboard.putNumber("Main Wheel Goal", mPeriodicIO.main_demand);
        SmartDashboard.putNumber("Main Wheel Temperature", mPeriodicIO.main_temperature);
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public synchronized boolean checkSystem() {
        return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;

        public double upper_velocity;
        public double upper_voltage;
        public double upper_current;
        public double upper_temperature;
        public double main_velocity;
        public double main_voltage;
        public double main_current;
        public double main_temperature;

        //OUTPUTS
        public double upper_demand;
        public double main_demand;
    }
}
