package com.team1678.frc2021.auto;

import com.team1678.frc2021.auto.modes.StandStillMode;
import com.team1678.frc2021.auto.modes.TestStraightPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.TEST_STRAIGHT_PATH;

    public static final Alliance STANDARD_CARPET_SIDE = Alliance.BLUE;
    public static final Alliance NONSTANDARD_CARPET_SIDE = Alliance.RED;
    
    private SendableChooser<AutoOption> modeChooser;
    private SendableChooser<Side> sideChooser;
    private SendableChooser<Alliance> allianceChooser;


    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);

        // 2021 autos
        modeChooser.addOption(AutoOption.TEST_STRAIGHT_PATH.name, AutoOption.TEST_STRAIGHT_PATH);

        sideChooser = new SendableChooser<Side>();
        sideChooser.setDefaultOption("Right", Side.RIGHT);
        sideChooser.addOption("Left", Side.LEFT);

        allianceChooser = new SendableChooser<Alliance>();
        allianceChooser.setDefaultOption("Blue", Alliance.BLUE);
        allianceChooser.addOption("Red", Alliance.RED);
    	
        SmartDashboard.putData("Mode Chooser", modeChooser);
        SmartDashboard.putData("Side Chooser", sideChooser);
        SmartDashboard.putData("Alliance Chooser", allianceChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();
        Side selectedSide = (Side) sideChooser.getSelected();
        boolean left = (selectedSide == Side.LEFT);
                
        return createAutoMode(selectedOption, left);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    public Alliance getSelectedAlliance(){
        Alliance alliance = (Alliance) allianceChooser.getSelected();
        return alliance;
    }
    
    enum AutoOption{
        TEST_STRAIGHT_PATH("Straight Path");

    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    enum Side{
        LEFT, RIGHT
    }

    public enum Alliance{
        RED, BLUE
    }
    
    private AutoModeBase createAutoMode(AutoOption option, boolean left){
    	switch(option){
            case TEST_STRAIGHT_PATH:
                return new TestStraightPath(left);
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
