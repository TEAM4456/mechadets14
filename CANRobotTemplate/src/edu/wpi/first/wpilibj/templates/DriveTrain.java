/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * 
 * Class for DriveTrain
 *
 * @author samega15
 */

public class DriveTrain
{
    
    private Jaguar MotorLF, MotorLB, MotorRF, MotorRB;
    private RobotDrive chassis;
    
    
    public void init()
    {
        MotorLF = new Jaguar(1);
        MotorLB = new Jaguar(2);
        MotorRF = new Jaguar(3);
        MotorRB = new Jaguar(4);
        chassis = new RobotDrive(MotorLF, MotorLB, MotorRF, MotorRB);
    }
    
    //will set chassisSafteyEnabled to boolean enabled
    //Turns off safety mechanism to allow drive train motors to say on more than 0.1s
    public void setChassisSafteyEnabled(boolean enabled)
    {
        chassis.setSafetyEnabled(enabled);
    }
    
    //given a magnitude, direciton, and location, this will drive the robot.
    //Arguments are usually given by the controller.
    public void drive(double magnitude, double direction, double rotation)
    {
        chassis.mecanumDrive_Polar(magnitude, direction, rotation);
    }
    
}
