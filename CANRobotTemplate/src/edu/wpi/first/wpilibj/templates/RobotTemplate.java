/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// GIT REPOSITORY SUCCESSFULLY MADE!

//CANRobotTemplate 2014

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.RobotDrive; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.templates.RobotTemplate;
import edu.wpi.first.wpilibj.command.Scheduler;

//test Mech Cadets

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    //The following two lines create the variables to hold the Autonomous command
    Command autonomousCommand; 
    SendableChooser autoChooser;
    
    Joystick controller;
    Gyro gyro;
    ADXL345_I2C accel;
    Compressor compressor;
    Servo servoVertical, servoHorizontal;

    //Relay ledLight;
    //left front 1
    //left back 2
    //right front 3
    //right back 4
   
    CriteriaCollection cc;
    
    //This allows the user of the driver netbook to manually adjust the distance he or she thinks they are from the target
    double distanceGuess;
    Timer timer = new Timer();
    
    public void robotInit()
    {
        //uses the init methods from the other classes
        DriveTrain.init();
        Loader.init();
        Shooter.init();
        UI.init();
        
        controller = new Joystick(1);
        compressor = new Compressor(4,1); //Assigns Compressor
        timer.start(); //Initialize timer
    }
    
    /**
     * This autonomous function is called once each time the robot enters autonomous mode.
     */
    public void autonomous()
    {
        DriveTrain.setChassisSafteyEnabled(false);
        
        //The robot will drive forward for 1.2 seconds
        DriveTrain.drive(1.0, 0.0, 0.0); //Mecanum Drive forward full speed
        timer.delay(1.2);
        DriveTrain.drive(0.0,0.0,0.0); //Turn off Mecanum Drive
        
        Shooter.unwindWinchAuto();
        
        //The robot will exit autonomous mode if !isAutonomous
        if (!isAutonomous())
        {
            return;
        }
        
        //This stops the winch
        Shooter.stopWinch();
        
        UI.output();
        UI.dash();
        
        Shooter.shootAuto();
        Shooter.reload();
    }
    
    /** 
     * This function is called once each time the robot enters operator control.
     */

    public void operatorControl() 
    {
        compressor.start();  
        while(isOperatorControl() && isEnabled())
        {            
            UI.output();
            UI.dash();

            //Controls the shooter winches with the xbox triggers
            Shooter.setWinches(controller.getRawAxis(Constants.axis_triggers));

            //Button A will shoot the ball when pressed
            if (controller.getRawButton(Constants.button_A)) 
            {
                Shooter.shoot();
            }

            //A Specified button will release winch to a specified distance when pressed
            if(controller.getRawButton(Constants.button_leftBumper))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("superLong_BeltLength", -18.0));
            }
            if (controller.getRawButton(Constants.button_Start)) 
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("long_BeltLength", -10.0));
            }
            if(controller.getRawButton(Constants.button_Y))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("mediumLong_BeltLength", -7.0));
            }
            if(controller.getRawButton(Constants.button_X))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("mediumShort_BeltLength", -4.5));
            }
            if(controller.getRawButton(Constants.button_B))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("short_BeltLength", -2.0));
            }
            
            //If the Back Button (Button 7) is pressed, it will reload
            if (controller.getRawButton(Constants.button_Back))
            {
                Shooter.reload();   
            }
            
            //While the Right Bumper is pressed, it will pick up the ball
            if (controller.getRawButton(Constants.button_rightBumper))
            {
                Loader.pickUpBall();
            }
            //While the Right Bumper is not pressed, it will go back to its original position
            else
            {
                Loader.resetArms();
            }

            //This will drive the robot with respect to controller inputs
            DriveTrain.drive(lowerSensitivity(controller.getMagnitude()),
                            controller.getDirectionDegrees(),
                            lowerSensitivity(controller.getRawAxis(Constants.axis_rightStick_X)));

            Timer.delay(0.01);
        }
        
        compressor.stop();   
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test()
    {
        
    }
    
    /*
     * These are some misc utility methods that might be called by various tasks
     */
    
    //This lowers the sensitivity of the input (the controls)
    public double lowerSensitivity(double magnitude)
    {
        double f = (com.sun.squawk.util.MathUtils.pow(magnitude,3));
        return f;
    }
    
    public double addDeadZone(double speed)
    {
        double deadzone = .1;
        if(Math.abs(speed)<deadzone)
        {
            speed = 0;
        }
        return speed;
    }
}

