package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team1678.frc2021.Constants;
import com.team254.lib.drivers.BaseTalonChecker;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Hood extends ServoMotorSubsystem {
    protected static AnalogEncoder mEncoder;
    private static Hood mInstance;
    private boolean mHoming = true;

    public static synchronized Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.kHoodConstants);
        }
        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);
        mEncoder = new AnalogEncoder(Constants.kHoodConstants.kAbsoluteEncoderID);
        //mMaster.setSelectedSensorPosition((int) unitsToTicks(17.66));
    }

    //@Override
    //public synchronized boolean atHomingLocation() {
    //   return Canifier.getHoodLimit(); TODO fix homing
    //}

    public synchronized boolean isHoming() {
        return mHoming;
    }

    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean getAtGoal() {
        return Util.epsilonEquals(getAngle(), getSetpoint(), 5.0);
    }

    public synchronized boolean getTucked() {
        return Util.epsilonEquals(getAngle(), Constants.kHoodConstants.kMinUnitsLimit, 5.0);
    }

    private void updateHoming() {
        mHoming = !new ServoMotorSubsystem(Constants.kHoodConstants) {
            @Override
            public boolean checkSystem() {
                return false;
            }
        }.atHomingLocation();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        updateHoming();
        if (mHoming) {
            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (mHoming) {
            System.out.println("is homing");
            mEncoder.reset();
            // Motor to encoder

            //mMaster.setSelectedSensorPosition(mEncoder.getDistance());
            //mMaster.setSelectedSensorPosition((int) unitsToTicks(17.66));

            //mMaster.overrideSoftLimitsEnable(true);
            System.out.println("Homed!!!");
            mHoming = false;
        }
        super.readPeriodicInputs();
    }

    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = -716113039054569446L;

            {
                add(new MotorChecker.MotorConfig<>("master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mRunOutputPercentage = 0.5;
                mRunTimeSec = 1.0;
                mCurrentFloor = 0.1;
                mRPMFloor = 90;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 200;
                mRPMSupplier = mMaster::getSelectedSensorVelocity;
            }
        });
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        SmartDashboard.putBoolean(mConstants.kName + " Calibrated", !mHoming);
        SmartDashboard.putBoolean("Hood at Homing Location", atHomingLocation());
        SmartDashboard.putNumber("Hood Encoder Readout", mEncoder.getDistance());
    }

    public void setCoastMode() {
        mMaster.setNeutralMode(NeutralMode.Coast);
    }
}
