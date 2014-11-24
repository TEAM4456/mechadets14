package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * 
 * @author Mech Cadets
 */

public class DriveTrain
{
    private Jaguar MotorLF, MotorLB, MotorRF, MotorRB;
    private RobotDrive chassis;
    
    public DriveTrain()
    {
        MotorLF = new Jaguar(1);
        MotorLB = new Jaguar(2);
        MotorRF = new Jaguar(3);
        MotorRB = new Jaguar(4);
        chassis = new RobotDrive(MotorLF, MotorLB, MotorRF, MotorRB);
    }
    
    //This will set chassisSafteyEnabled to boolean enabled
    //Turns off safety mechanism to allow drive train motors to say on more than 0.1s
    public void setChassisSafteyEnabled(boolean enabled)
    {
        chassis.setSafetyEnabled(enabled);
    }
    
    //This will drive the robot given a magnitude, direciton, and location
    //Arguments are usually given by the controller
    public void drive(double magnitude, double direction, double rotation)
    {
        chassis.mecanumDrive_Polar(magnitude, direction, rotation);
    }   
}