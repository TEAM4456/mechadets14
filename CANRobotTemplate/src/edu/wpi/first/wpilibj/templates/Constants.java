package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Mech Cadets
 */
public class Constants
{
    public static final double WINCH_OVERSHOOT = 0.4;
    
    //These constants each refer to a different hook state
    public static final int HOOK_READY = 0;
    public static final int HOOK_WAIT_1 = 1;
    public static final int HOOK_WAIT_2 = 2;
    
    //These constants are for Vision Processing
    public static final double pixelsT = 640;
    public static final double pixelsV = 480;
    public static final double visionR = 47*Math.PI/180;
    public static final double lengthG = 36/2;
    public static final double vertAngle = 32.5;
    public static final double targetHeight = 32.0;//this height is from last year's reflective board. Must measure this year's goals!! Devin
    public static final double desiredDistance = 60;
    
    //Defines Xbox Buttons
    public static final int button_A = 1;
    public static final int button_B = 2;
    public static final int button_X = 3;
    public static final int button_Y = 4;
    public static final int button_leftBumper = 5;
    public static final int button_rightBumper = 6;
    public static final int button_Back = 7;
    public static final int button_Start = 8;
    public static final int button_leftStick = 9;
    public static final int button_rightStick = 10;
    public static final int axis_leftStick_X = 1;
    public static final int axis_leftStick_Y = 2;
    public static final int axis_triggers = 3;
    public static final int axis_rightStick_X = 4;
    public static final int axis_rightStick_Y = 5;
    public static final int axis_dPad_X = 6;   
}