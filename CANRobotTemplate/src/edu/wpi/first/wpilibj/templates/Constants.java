/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author samega15
 */
public class Constants
{
    
    //These constants each refer to a different hook state
    public final int HOOK_READY = 0;
    public final int HOOK_WAIT_1 = 1;
    public final int HOOK_WAIT_2 = 2;
    
    //Vision Processing
    public final double pixelsT = 640;
    public final double pixelsV = 480;
    public final double visionR = 47*Math.PI/180;
    public final double lengthG = 36/2;
    public final double vertAngle = 32.5;
    public final double targetHeight = 32.0;//this height is from last year's reflective board. Must measure this year's goals!! Devin
    public final double desiredDistance = 60;
    
    public final double WINCH_OVERSHOOT = 0.4;
    
    //Defines Xbox Buttons
    public final int button_A = 1;
    public final int button_B = 2;
    public final int button_X = 3;
    public final int button_Y = 4;
    public final int button_leftBumper = 5;
    public final int button_rightBumper = 6;
    public final int button_Back = 7;
    public final int button_Start = 8;
    public final int button_leftStick = 9;
    public final int button_rightStick = 10;
    public final int axis_leftStick_X = 1;
    public final int axis_leftStick_Y = 2;
    public final int axis_triggers = 3;
    public final int axis_rightStick_X = 4;
    public final int axis_rightStick_Y = 5;
    public final int axis_dPad_X = 6;
    
}
