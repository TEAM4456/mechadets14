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
    private static DriverStationLCD outputBox;
    //Declares the print counter
    private int printCounter;
    
    
    public static void init()
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
    
    public static void output() 
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
        outputBox.println(DriverStationLCD.Line.kUser2, 1, "Encoder Winch " + Shooter.getWinchDistance()); //prints the encode winch distance
        outputBox.println(DriverStationLCD.Line.kUser3, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser4, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser5, 1, "                                        ");
        outputBox.println(DriverStationLCD.Line.kUser6, 1, "VP Distance: " + RobotTemplate.visionDistance); // prints the vision distance
        //this updates the driver's station output screen, allowing everything to show up correctly
        outputBox.updateLCD();
    }
    
    //This displays the dashboard values
    public static void dash()
    {
        SmartDashboard.putNumber("controllerA1", controller.getRawAxis(Constants.axis_leftStick_X));
        
        SmartDashboard.putNumber("winchEncoder", Shooter.getWinchDistance();
        
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
}