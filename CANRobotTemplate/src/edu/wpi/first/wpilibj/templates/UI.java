package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Mech Cadets
 */
public class UI
{
    //Declares the Driver Station output box
    private DriverStationLCD outputBox;
    //Declares the print counter
    private int printCounter;
    
    
    public UI()
    {
        //The output box initializes itself
        outputBox = DriverStationLCD.getInstance();
        printCounter = 0;
        
        //The doubles that are written in the following lines are default values and can be changed.
        //Change the default values to the starting values of the robot in autonomous mode
        SmartDashboard.putNumber("superLong_BeltLength", -18.0);
        SmartDashboard.putNumber("long_BeltLength", -10.0);
        SmartDashboard.putNumber("mediumLong_BeltLength", -7.0);
        SmartDashboard.putNumber("mediumShort_BeltLength", -4.5);
        SmartDashboard.putNumber("short_BeltLength", -2.0);
    }
    
    public void output(RobotTemplate robot) 
    {
        //Increments the print counter
        printCounter++;

        if (printCounter == 100) 
        {       
            //Once the print counter reachers 100, it will prevent the print screen from getting cluttered
            outputBox.println(DriverStationLCD.Line.kUser1, 1, "                                        ");
            outputBox.println(DriverStationLCD.Line.kUser2, 1, "                                        ");
            outputBox.println(DriverStationLCD.Line.kUser3, 1, "                                        ");
            outputBox.println(DriverStationLCD.Line.kUser4, 1, "                                        ");
            outputBox.println(DriverStationLCD.Line.kUser5, 1, "                                        ");
            outputBox.println(DriverStationLCD.Line.kUser6, 1, "                                        ");
            //this updates the driver's station output screen, allowing everything to show up correctly
            outputBox.updateLCD();
            //The print counter gets reset back to 0
            printCounter = 0;
        } 
        
        outputBox.println(DriverStationLCD.Line.kUser1, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser2, 1, "Encoder Winch " + robot.shooter.getWinchDistance()); //prints the encode winch distance
        outputBox.println(DriverStationLCD.Line.kUser3, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser4, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser5, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser6, 1, "VP Distance: " /* + robot.visionDistance*/); // prints the vision distance
        //this updates the driver's station output screen, allowing everything to show up correctly
        outputBox.updateLCD();
    }
    
    //This displays the dashboard values
    public void dashDisplay(RobotTemplate robot)
    {
        SmartDashboard.putNumber("controllerA1", robot.controller.getRawAxis(Constants.axis_leftStick_X));
        
        SmartDashboard.putNumber("winchEncoder", robot.shooter.getWinchDistance());
        
        SmartDashboard.putNumber("Distance", robot.distanceGuess);

        SmartDashboard.putNumber("Super Long Belt Length", robot.shooter.superLong_BeltLength);
        SmartDashboard.putNumber("Long Belt Length", robot.shooter.long_BeltLength);
        SmartDashboard.putNumber("Medium Short Belt Length", robot.shooter.mediumShort_BeltLength);
        SmartDashboard.putNumber("Medium Long Belt Length", robot.shooter.mediumLong_BeltLength);
        SmartDashboard.putNumber("Short Belt Length", robot.shooter.short_BeltLength);

        SmartDashboard.putNumber("winchEncoderDisPerPulse", robot.shooter.winchEncoder.getRaw());
        SmartDashboard.putBoolean("Limit Switch", robot.shooter.limitSwitch.get() );
        SmartDashboard.putNumber("Distance from Target", robot.distanceGuess);
    }
}