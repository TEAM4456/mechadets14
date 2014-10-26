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

//test serge



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    //the following two lines create the variables to hold the Autonomous command
    Command autonomousCommand; 
    SendableChooser autoChooser;
    //public static SendableChooser autoChooser;
    
    Joystick controller;
    Gyro gyro;
    ADXL345_I2C accel;
    DriverStationLCD robot = DriverStationLCD.getInstance();     //Drivers Station output box declaration
    Compressor compressor;
    Servo servoVertical, servoHorizontal;
    AxisCamera camera;

    //Relay ledLight;
    //left front 1
    //left back 2
    //right front 3
    //right back 4
    
    
    private int printCounter = 0;

    
    CriteriaCollection cc;
    
    private double visionDistance;
    private double VisionCounter;
    
    //This allows the user of the driver netbook to manually adjust the distance he or she thinks they are from the target. Devin 
    //Preferences prefs;
    double distanceGuess;
    //double beltLength;
    Timer timer = new Timer();

    
    //declares and initializes components of robot
  
    
    public void robotInit()
    {
        SmartDashboard.putNumber("superLong_BeltLength", -18.0); //the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("long_BeltLength", -10.0); //the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("mediumLong_BeltLength", -7.0);//the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("mediumShort_BeltLength", -4.5);//the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("short_BeltLength", -2.0);//the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        
        controller = new Joystick(1);
        
        compressor = new Compressor(4,1); //Assigns Compressor
        
        
        
        timer.start(); //Initialize timer
        
        
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() 
    {
        
        DriveTrain.setChassisSafteyEnabled(false);
        
        //drive fwd for 1.2 seconds
        DriveTrain.drive(1.0, 0.0, 0.0); //Mecanum Drive forward full speed
        timer.delay(1.2);
        DriveTrain.drive(0.0,0.0,0.0); //Turn off Mecanum Drive
        
        Shooter.unwindWinchAuto();
        
        //will exit the autonomous method if !isAutonomous
        if (!isAutonomous())
        {
            return;
        }
        
        //stop the winch
        Shooter.stopWinch();
        
        Output();
        dash();
        
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
            Output();
            dash();

            // Controls the shooter winches with the xbox triggers
            Shooter.setWinches(controller.getRawAxis(Constants.axis_triggers));

            //Button A will disengage the hook
            if (controller.getRawButton(Constants.button_A)) 
            {
                Shooter.shoot();
            }
            
            
            //specified button will release winch to specified distance
            if(controller.getRawButton(Constants.button_leftBumper))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("superLong_BeltLength", -18.0));
            }
            if (controller.getRawButton(Constants.button_Start)) 
            {
                //Shooter.releaseWinch(prefs.getDouble("belt", -10.0));
                Shooter.releaseWinch(SmartDashboard.getNumber("long_BeltLength", -10.0));
            }      
            if(controller.getRawButton(Constants.button_X))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("mediumShort_BeltLength", -4.5));
            }
            if(controller.getRawButton(Constants.button_B))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("short_BeltLength", -2.0));
            }
            if(controller.getRawButton(Constants.button_Y))
            {
                Shooter.releaseWinch(SmartDashboard.getNumber("mediumLong_BeltLength", -7.0));
            }
            
            
            //if the Back Button (Button 7) is pressed, it will reload
            if (controller.getRawButton(Constants.button_Back))
            {
                Shooter.reload();   
            }
            
            //while the Right Bumper is pressed, it will pick up the ball
            if (controller.getRawButton(Constants.button_rightBumper))
            {
                Loader.pickUpBall();
            }
            //while the Right Bumper is not pressed, it will go back to its original position
            else
            {
                Loader.resetArms();
            }

            //drive the robot with respect to controller inputs
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
     * Primary control methods including camera, shooting, driving, picking up, 
     * and output which are called once per loop through operator control 
     */
    
    public void Output() 
    {
        printCounter++;

        if (printCounter == 100) 
        {       
            //this will prevent the print screen from getting cluttered
            robot.println(DriverStationLCD.Line.kUser1, 1, "                                        ");
            robot.println(DriverStationLCD.Line.kUser2, 1, "                                        ");
            robot.println(DriverStationLCD.Line.kUser3, 1, "                                        ");
            robot.println(DriverStationLCD.Line.kUser4, 1, "                                        ");
            robot.println(DriverStationLCD.Line.kUser5, 1, "                                        ");
            robot.println(DriverStationLCD.Line.kUser6, 1, "                                        ");
            robot.updateLCD(); //this updates the drivers station output screen, allowing everything to show up correctly
            printCounter = 0;
        } 

        robot.println(DriverStationLCD.Line.kUser6, 1, "VP Distance: " + visionDistance);
        //robot.println(DriverStationLCD.Line.kUser1, 1, "GyroAngle " + gyro.getAngle());
        robot.println(DriverStationLCD.Line.kUser2, 1, "Encoder Winch " + Shooter.getWinchDistance());
        //robot.println(DriverStationLCD.Line.kUser3, 1, "AccelX: " + accel.getAcceleration(ADXL345_I2C.Axes.kX));
        //robot.println(DriverStationLCD.Line.kUser4, 1, "AccelY: " + accel.getAcceleration(ADXL345_I2C.Axes.kY));
        //robot.println(DriverStationLCD.Line.kUser5, 1, "AccelZ: " + accel.getAcceleration(ADXL345_I2C.Axes.kZ));
        robot.updateLCD();
    }
            
    
    //displays dashboard values
    public void dash()
    {
        SmartDashboard.putNumber("controllerA1", controller.getRawAxis(Constants.axis_leftStick_X));
        //SmartDashboard.putNumber("AccelX", accel.getAcceleration(ADXL345_I2C.Axes.kX));
        //SmartDashboard.putNumber("AccelY", accel.getAcceleration(ADXL345_I2C.Axes.kY));
        //SmartDashboard.putNumber("AccelZ", accel.getAcceleration(ADXL345_I2C.Axes.kZ));
        //SmartDashboard.putNumber("Angle of Ladder", com.sun.squawk.util.MathUtils.atan2((double)accel.getAcceleration(ADXL345_I2C.Axes.kX), (double)accel.getAcceleration(ADXL345_I2C.Axes.kY)));
        SmartDashboard.putNumber("winchEncoder", Shooter.getWinchDistance();
        //SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putNumber("Distance", distanceGuess);

        SmartDashboard.putNumber("Super Long Belt Length", Shooter.superLong_BeltLength);
        SmartDashboard.putNumber("Long Belt Length", Shooter.long_BeltLength);
        SmartDashboard.putNumber("Medium Short Belt Length", Shooter.mediumShort_BeltLength);
        SmartDashboard.putNumber("Medium Long Belt Length", Shooter.mediumLong_BeltLength);
        SmartDashboard.putNumber("Short Belt Length", Shooter.short_BeltLength);


        SmartDashboard.putNumber("winchEncoderDisPerPulse", Shooter.winchEncoder.getRaw());
        SmartDashboard.putBoolean("Limit Switch", Shooter.limitSwitch.get() );
        SmartDashboard.putNumber("Distance from Target", distanceGuess);
    }

    private void distanceCalculate() 
    {
        ColorImage image = null;
        BinaryImage thresholdRGBImage = null;
        BinaryImage thresholdHSIImage = null;
        BinaryImage bigObjectsImage= null;
        BinaryImage convexHullImage=null;
        BinaryImage filteredImage = null;
        
        try 
        {
            image = camera.getImage();
            camera.writeBrightness(50);
            image.write("originalImage.jpg");
            thresholdRGBImage = image.thresholdRGB(0, 45, 175, 255, 0, 47);
            
            thresholdRGBImage.write("thresholdRGBImage.bmp");
            thresholdHSIImage = image.thresholdHSI(0, 255, 0, 255, 200, 255);
            thresholdHSIImage.write("thresholdHSIImage.bmp");
            bigObjectsImage = thresholdHSIImage.removeSmallObjects(false, 2);
            bigObjectsImage.write("bigObjectsImage.bmp");
            convexHullImage = bigObjectsImage.convexHull(false);
            convexHullImage.write("convexHullImage.bmp");
            filteredImage = convexHullImage.particleFilter(cc);
            filteredImage.write("filteredImage.bmp");
            ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();
            
            String output;
//            for(int i = 0; i<reports.length+1; i++) {
//                robot.println(DriverStationLCD.Line.kUser6, 1, ""+reports[i].center_mass_x);
//                System.out.println(reports[i].center_mass_x);
//            }
            if (reports.length > 0) 
            {
                double pixelsHeight = reports[0].boundingRectHeight;
//                double centerX = reports[0].center_mass_x_normalized;
//                double centerY = reports[0].center_mass_y_normalized;
//                double targetAngle = 47*(centerX);
                double angle = pixelsHeight/Constants.pixelsV*(Constants.vertAngle*Math.PI/180);
                double Vdistance = Constants.targetHeight/angle;
                visionDistance = Vdistance;

                SmartDashboard.putNumber("Distance: ", Vdistance);
            } 
            else 
            {
//                robot.println(DriverStationLCD.Line.kUser6, 1, "no targets. :(");
            }
        }
        catch (Exception ex) 
        {
            ex.printStackTrace();
        }
        finally
        {
            
        }
        
        
        try 
        {
            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            //thresholdRGBImage.free();
            thresholdHSIImage.free();
            image.free();
        } 
        catch (NIVisionException ex) 
        {
            ex.printStackTrace();
        }
    }
    
    /*
     * Misc utility methods that might be called by various tasks
    */
    
    // This lowers the sensitivity of the input (the controls)
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

