package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team1678.frc2021.Constants;

public class Canifier extends Subsystem {
    private static Canifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;

    /**
     * The private utility class
     * An instance of the canifier
     */
    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicInputs = new PeriodicInputs();
    }

    /**
     * Gets and instance of the canifier if there are no existing instances
     * @return a new canifier instance
     */
    public static synchronized Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    /**
     * Reads the hood limit from the Periodic IO
     * @return the hood limit
     */
    public synchronized boolean getHoodLimit() {
        return mPeriodicInputs.hood_limit_;
    }

    /**
     * Gets the canifier device ID
     * @return the device ID as an integer
     */
    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    /**
     * Reads the periodic input
     */
    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);

        mPeriodicInputs.indexer_limit_ = !pins.SDA;
        mPeriodicInputs.hood_limit_ = !pins.SPI_MOSI_PWM1;

    }

    /**
     * Gets the canifier itself as a CANifier
     * The CANifier is the canifier object provided by CTRE
     * @return the canifier
     */
    public synchronized CANifier getCanifier() {
        return mCanifier;
    }

    /**
     * Check system
     * @return false
     */
    @Override
    public boolean checkSystem() {
        return false;
    }

    /**
     * The output for the canifier into the smartdashboard
     */
    @Override
    public void outputTelemetry() {
        // Add the things you want to put into the smartdashboard, if any
    }

    /**
     * Stops the loop or not
     */
    @Override
    public void stop() {
        // Put the set loop methods if any is wanted
    }

    /**
     * Zeros the sensors
     * Creates new periodic inputs
     */
    @Override
    public void zeroSensors() {
        mPeriodicInputs = new PeriodicInputs();
    }

    /**
     * The periodic inputs into the canifier
     */
    private static class PeriodicInputs {
        public boolean indexer_limit_;
        public boolean hood_limit_;
    }
}
