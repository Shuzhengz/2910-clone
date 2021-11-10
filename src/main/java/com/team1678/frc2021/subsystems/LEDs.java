/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2021.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.team1323.lib.util.HSVtoRGB;
import com.team1323.lib.util.MovingAverage;
import com.team1678.frc2021.loops.ILooper;
import com.team1678.frc2021.loops.Loop;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * Brings all da colors to da club
 */
public class LEDs extends Subsystem {
    private static LEDs instance = null;

    public static LEDs getInstance() {
        if (instance == null)
            instance = new LEDs();
        return instance;
    }

    CANifier canifier;

    /**
     * LED Utility class
     * creates new instances
     */
    public LEDs() {
        canifier = Canifier.getInstance().getCanifier();
    }

    boolean lit = false;
    double lastOnTime = 0.0;
    double lastOffTime = 0.0;
    double transTime = 0.0;

    /**
     * The states of the LED and the RGB values
     */
    public enum State {
        OFF(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        DISABLED(255.0, 20.0, 30.0, Double.POSITIVE_INFINITY, 0.0, false),
        ENABLED(0.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        EMERGENCY(255.0, 0.0, 0.0, 0.5, 0.5, false),
        LIMELIGHT_SEES_ONLY(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        INVISIBLE_TARGET_TRACKING(255.0, 255.0, 0.0, 0.0625, 0.0625, false),
        HOOD_TUCKED(255.0, 20.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        BALL_IN_INTAKE(255.0, 20.0, 0.0, 0.5, 0.5, false),
        BALL_IN_CARRIAGE(255.0, 20.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        DISK_IN_INTAKE(255.0, 60.0, 0.0, 0.5, 0.5, false),
        DISK_IN_PROBE(255.0, 60.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        TARGET_VISIBLE(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        TARGET_TRACKING(0.0, 255.0, 0.0, 0.0625, 0.0625, false),
        CLIMBING(255.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        RAINBOW(0, true),
        BREATHING_PINK(357, 10.0, true);

        // Declares doubles
        double red;
        double green;
        double blue;
        double onTime;
        double offTime;
        double cycleTime;
        double transitionTime;

        float startingHue;
        List<List<Double>> colors = new ArrayList<>();
        boolean isCycleColors;

        /**
         * The states
         *
         * @param r             red
         * @param g             green
         * @param b             blue
         * @param onTime        the time that it is on
         * @param offTime       the time that it is off
         * @param isCycleColors if it is currently cycling colors
         */
        private State(double r, double g, double b, double onTime, double offTime, boolean isCycleColors) {
            red = r / 255.0;
            green = g / 255.0;
            blue = b / 255.0;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        /**
         * The states
         *
         * @param hue   the hue of the light in float
         * @param cycle cycle or not
         */
        private State(float hue, boolean cycle) {
            this.startingHue = hue;
            this.isCycleColors = cycle;
        }

        /**
         * The states
         *
         * @param hue       the hue of the light in float
         * @param transTime the transition time for the colors
         * @param cycle     cycle or not
         */
        private State(float hue, double transTime, boolean cycle) {
            this.startingHue = hue;
            this.transitionTime = transTime;
            this.isCycleColors = cycle;
        }

        /**
         * The states
         *
         * @param colors         the colors in a list of lists
         * @param cycleTime      the cycle time of the lights
         * @param isCycleColors  is currently cycling color or not
         * @param transitionTime the transition time for the color
         */
        private State(List<List<Double>> colors, double cycleTime, boolean isCycleColors, double transitionTime) {
            this.colors = colors;
            this.cycleTime = cycleTime;
            this.isCycleColors = isCycleColors;
            this.transitionTime = transitionTime;
        }
    }

    // Initialize the current state as off
    private State currentState = State.OFF;

    /**
     * Gets the current state
     *
     * @return current state
     */
    public State getState() {
        return currentState;
    }

    /**
     * Set the state
     *
     * @param newState pass in a new state
     */
    private void setState(State newState) {
        if (newState != currentState) {
            currentState = newState;
            lastOffTime = 0.0;
            lastOnTime = 0.0;
            lit = false;
        }
    }

    // Creates new loop
    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {

        }

        @Override
        public void onStop(double timestamp) {

        }

    };

    /**
     * Sets the LED for the canifiers
     *
     * @param r red
     * @param g green
     * @param b blue
     */
    public void setLEDs(double r, double g, double b) {
        //A: Green
        //B: Red
        //C: Blue
        canifier.setLEDOutput(r, LEDChannel.LEDChannelB);
        canifier.setLEDOutput(g, LEDChannel.LEDChannelA);
        canifier.setLEDOutput(b, LEDChannel.LEDChannelC);
    }

    /**
     * Sets the state to new state
     *
     * @param state the state to set state to
     */
    public void conformToState(State state) {
        setState(state);
    }

    public double stateHue = State.RAINBOW.startingHue;
    public float saturation = 1.0f; // Ensures that the colors are on the outside of the color wheel
    public float value = 1.0f; // Hardcoded brightness
    public double startingTransTime = 0.0;
    public boolean resetBreath = false;

    /**
     * Writes to the periodic output
     */
    @Override
    public void writePeriodicOutputs() {
        double timestamp = Timer.getFPGATimestamp();
        if (currentState == State.RAINBOW && currentState.isCycleColors) {
            stateHue += 2;
            if (stateHue >= (360 - State.RAINBOW.startingHue)) {
                stateHue = State.RAINBOW.startingHue;
            }

            float[] rgb = new float[3];
            MovingAverage averageR = new MovingAverage(5);
            MovingAverage averageG = new MovingAverage(5);
            MovingAverage averageB = new MovingAverage(5);

            if (saturation > 1) {
                saturation = 1;
            }
            if (saturation < 0) {
                saturation = 0;
            }
            if (value > 1) {
                value = 1;
            }
            if (value < 0) {
                value = 0;
            }

            rgb = HSVtoRGB.convert(stateHue, saturation, value);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            setLEDs(rgb[0], rgb[1], rgb[2]);

        } else if (currentState == State.BREATHING_PINK && currentState.isCycleColors) {
            if (startingTransTime <= currentState.transitionTime && !resetBreath) {
                startingTransTime += currentState.transitionTime / 50.0;
            } else if (resetBreath) {
                startingTransTime -= currentState.transitionTime / 50.0;
            }
            if (resetBreath && startingTransTime <= 0.0) {
                resetBreath = false;
            } else if (!resetBreath && startingTransTime >= currentState.transitionTime) {
                resetBreath = true;
            }


            float[] rgb = new float[3];
            MovingAverage averageR = new MovingAverage(10);
            MovingAverage averageG = new MovingAverage(10);
            MovingAverage averageB = new MovingAverage(10);

            double valueBasedOnTime = currentState.transitionTime - startingTransTime;

            rgb = HSVtoRGB.convert(State.BREATHING_PINK.startingHue, 0.922f, valueBasedOnTime);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            setLEDs(rgb[0], rgb[1], rgb[2]);

        } else if (!lit && (timestamp - lastOffTime) >= currentState.offTime && !currentState.isCycleColors) {
            setLEDs(currentState.red, currentState.green, currentState.blue);
            lastOnTime = timestamp;
            lit = true;
        } else if (lit && !Double.isInfinite(currentState.onTime) && !currentState.isCycleColors) {
            if ((timestamp - lastOnTime) >= currentState.onTime) {
                setLEDs(0.0, 0.0, 0.0);
                lastOffTime = timestamp;
                lit = false;
            }
        }
    }

    /**
     * The output telemetry
     * For things to putout into the smartdashboard
     */
    @Override
    public void outputTelemetry() {
        // Things to output into the smartdashboard
    }

    /**
     * Registers the enabled ILoopers
     *
     * @param enabledLooper the looper that is enabled
     */
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    /**
     * Checks the system
     *
     * @return ture
     */
    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * Stop
     */
    @Override
    public void stop() {
        // What to do on stop
    }
}
