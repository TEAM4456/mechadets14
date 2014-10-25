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
    
    RobotDrive chassis;
    Joystick controller;
    Gyro gyro;
    ADXL345_I2C accel;
    DriverStationLCD robot = DriverStationLCD.getInstance();     //Drivers Station output box declaration
    Jaguar winch1, winch2, loaderArm1, loaderArm2, MotorLF, MotorLB, MotorRF, MotorRB; //MotorRF(R)ight(F)ront, MotorLB(L)eft(B)ack
    Compressor compressor;
    DoubleSolenoid hook, loaderPiston;
    Servo servoVertical, servoHorizontal;
    AxisCamera camera;
    DigitalInput limitSwitch;
    Encoder winchEncoder;
    //Relay ledLight;
    //left front 1
    //left back 2
    //right front 3
    //right back 4
    
    private final int HOOK_READY = 0;
    private final int HOOK_WAIT_1 = 1;
    private final int HOOK_WAIT_2 = 2;
    
    private int hookUpdateState = HOOK_READY;
    private double hookUpdateStartTime = 0.0;
    
    private int printCounter = 0;
    private int encoderCounter;
    private Value pistonPrevState;
    
    //Vision Processing
    final double pixelsT = 640;
    final double pixelsV = 480;
    final double visionR = 47*Math.PI/180;
    final double lengthG = 36/2;
    final double vertAngle = 32.5;
    final double targetHeight = 32.0;//this height is from last year's reflective board. Must measure this year's goals!! Devin
    final double desiredDistance = 60;
    CriteriaCollection cc;
    
    private double visionDistance;
    private double VisionCounter;
    
    //This allows the user of the driver netbook to manually adjust the distance he or she thinks they are from the target. Devin 
    //Preferences prefs;
    double distance;
    //double beltLength;
    double superLong_BeltLength;
    double long_BeltLength;
    double mediumLong_BeltLength;
    double mediumShort_BeltLength;
    double short_BeltLength;
    final double WINCH_OVERSHOOT = 0.4;
    Timer timer = new Timer();

    //Defines Xbox Buttons
    final int button_A = 1;
    final int button_B = 2;
    final int button_X = 3;
    final int button_Y = 4;
    final int button_leftBumper = 5;
    final int button_rightBumper = 6;
    final int button_Back = 7;
    final int button_Start = 8;
    final int button_leftStick = 9;
    final int button_rightStick = 10;
    final int axis_leftStick_X = 1;
    final int axis_leftStick_Y = 2;
    final int axis_triggers = 3;
    final int axis_rightStick_X = 4;
    final int axis_rightStick_Y = 5;
    final int axis_dPad_X = 6;
    private Value latched;
    private Value unlatched;
    
    //declares and initializes components of robot
  
    
    public void robotInit(){
        //prefs = Preferences.getInstance();
        
        SmartDashboard.putNumber("superLong_BeltLength", -18.0); //the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("long_BeltLength", -10.0); //the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("mediumLong_BeltLength", -7.0);//the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("mediumShort_BeltLength", -4.5);//the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("short_BeltLength", -2.0);//the two doubles that are written in the following line are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        //Assign Drive Train Motors
            MotorLF = new Jaguar(1);
            MotorLB = new Jaguar(2);
            MotorRF = new Jaguar(3);
            MotorRB = new Jaguar(4);
            chassis = new RobotDrive(MotorLF, MotorLB, MotorRF, MotorRB);
            
            winch1 = new Jaguar(5);
            winch2= new Jaguar(6);
        
         // Assign Pick Up Arm Motors
            loaderArm1 = new Jaguar(7);
            loaderArm2 = new Jaguar(8);
        
        controller = new Joystick(1);
        limitSwitch = new DigitalInput(3);//true = open; false = close
        //gyro = new Gyro(1);
        //accel = new ADXL345_I2C(1, ADXL345_I2C.DataFormat_Range.k4G);
        winchEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k1X);
        winchEncoder.setDistancePerPulse(1.0/750);//this pulse rate is for the competition robot. devin
        //winchEncoder.setDistancePerPulse(1.0/100);//this pulse rate is for the practice robot. devin
        //ledLight = new Relay(2);
        compressor = new Compressor(4,1); //Assigns Compressor
        hook = new DoubleSolenoid(1,2);  //Assigns Solenoid to Control Shooter Hook
            latched = DoubleSolenoid.Value.kReverse; //Assigns variable to Shooter Hook Solenoid to unhook
            unlatched = DoubleSolenoid.Value.kForward; //Assigns variable to Shooter Hook Solenoid to hook
        loaderPiston = new DoubleSolenoid(3,4); //Assigns Solenoid to Control Pick Up Arm
            pistonPrevState = DoubleSolenoid.Value.kOff; //Assigns Variable to Piston to Default state
        timer.start(); //Initialize timer
        winchEncoder.start();
        
        //Vision Processing
        //camera = AxisCamera.getInstance();
        //cc = new CriteriaCollection();
        //cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
        //cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false);
        //visionDistance = 0;
        //VisionCounter = 0;
        //cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_ORIENTATION, -0.2, 0.2, false);

        //keep the following four lines
//        autoChooser = new SendableChooser();
//        autoChooser.addDefault("One Ball Autonomous", new OneBallAutonomous());
//        autoChooser.addObject("Two Ball Autonomous", new TwoBallAutonomous());
//        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */

   
    public void autonomous() {   //need to work on this.
        //the following two lines should remain unchanged!
//        autonomousCommand = (Command)autoChooser.getSelected();
//        autonomousCommand.start();
         // gyro.reset();
//        distanceCalculate();
//        compressor.start();
          chassis.setSafetyEnabled(false);  //Turns off safety mechanism to allow drive train motors to say on more than 0.1s
          winchEncoder.reset();  //Used to make sure encoder is set to 0 at the start
//        
           chassis.mecanumDrive_Polar(1.0, 0.0, 0.0); //Mecanum Drive forward full speed
           timer.delay(1.2);
           chassis.mecanumDrive_Polar(0.0,0.0,0.0); //Turn off Mecanum Drive
                          
          while(winchEncoder.getDistance() > -7.0 && isAutonomous()){ //Unwind winch to belt length necessary to shot from starting position
               winch1.set(1.0);
               winch2.set(1.0);     
                }
                if (!isAutonomous())
                    return; 
                winch1.set(0.0);
                winch2.set(0.0);
                Output();
                dash();
                loaderPiston.set(DoubleSolenoid.Value.kForward);  
                timer.delay(0.5);
                hook.set(unlatched); //The mechanism will launch ball after getting to desired encoder value
                timer.delay(0.5);
                reload();
                
//           loaderPiston.set(DoubleSolenoid.Value.kForward);
//           loaderArm1.set(-.75);
//            loaderArm2.set(.75);
//            timer.delay(1.0);
//            loaderArm1.set(0.0);
//            loaderArm2.set(0.0);
//            loaderPiston.set(DoubleSolenoid.Value.kReverse);
//            timer.delay(1.0);
               
//                while(winchEncoder.getDistance() > -7.0 && isAutonomous()){ //Unwind winch to belt length necessary to shot from starting position
//               winch1.set(1.0);
//               winch2.set(1.0);     
//                }
//                if (!isAutonomous())
//                    return; 
//                winch1.set(0.0);
//                winch2.set(0.0);
//                Output();
//                dash();
//                loaderPiston.set(DoubleSolenoid.Value.kForward);  
//                timer.delay(0.5);
//                hook.set(unlatched); //The mechanism will launch ball after getting to desired encoder value
//                timer.delay(0.5);
//                reload();
                
//        // Need vision processing to determine if goal is hot or not placed here
//            
////  /*optional autonomous*/      
////            loaderPiston.set(DoubleSolenoid.Value.kForward);
////           loaderArm1.set(-.75);
////            loaderArm2.set(.75);
////            timer.delay(1.0);
////            loaderArm1.set(0.0);
////            loaderArm2.set(0.0);
////            loaderPiston.set(DoubleSolenoid.Value.kReverse);
////            timer.delay(1.0);
////            
////        
////        
////                while(winchEncoder.getDistance() > -4.5){ //Unwind winch to belt length necessary to shot from starting position
////                    winch1.set(1.0);
////                    winch2.set(1.0);
////                }
////                winch1.set(0.0);
////                winch2.set(0.0);
////                Output();
////                dash();
////             
////        
////        loaderPiston.set(DoubleSolenoid.Value.kForward);  
////        hook.set(unlatched); //The mechanism will launch ball after getting to desired encoder value
//// /*end of optional autonomous*/
//       
   }

    public void reload(){
                   //make sure the latch is open
                    hook.set(unlatched);
                    winchEncoder.start();
                    // *while the limit switch is pressed, push the winch negative
                    while(limitSwitch.get()  && (isAutonomous() || isOperatorControl())){
                        winch1.set(-1.0);
                        winch2.set(-1.0);
                    }//*set winch to 0
                    winch1.set(0);
                    winch2.set(0);
                    hook.set(latched);
                    loaderPiston.set(DoubleSolenoid.Value.kReverse);  
                    Timer.delay(0.8);
                    winchEncoder.reset();
                    //while(winchEncoder.getDistance() > -4.8){
                      //  winch1.set(1.0);
                      //  winch2.set(1.0);
                      // }
                        
    }
    
    public void releaseWinch(double distanceOfWinch){//use the encoder values from testing. Devin
        
        
            if(winchEncoder.getDistance() < distanceOfWinch){
                distanceOfWinch-= WINCH_OVERSHOOT;

                while(winchEncoder.getDistance() < distanceOfWinch){
                    winch1.set(-1.0);
                    winch2.set(-1.0);
                }
            }
            else{
                distanceOfWinch+= WINCH_OVERSHOOT;

                while(winchEncoder.getDistance() > distanceOfWinch){
                winch1.set(1.0);
                winch2.set(1.0);
            }
            }
                winch1.set(0.0);
                winch2.set(0.0);
    }
    
        /** 
     * This function is called once each time the robot enters operator control.
     */
    
    public void operatorControl() {
        compressor.start();  
        while(isOperatorControl() && isEnabled()){            
            //double angle = gyro.getAngle();
                     
            //ledLight.set(Relay.Value.kReverse);
            Output();
            dash();
//            if(VisionCounter >=100){
//                distanceCalculate();
//                VisionCounter = 0;
//            }
//            VisionCounter++;
                winch1.set(controller.getRawAxis(axis_triggers));
                winch2.set(controller.getRawAxis(axis_triggers));
          
            if (controller.getRawButton(button_A)){//this means that you press the A  button to disengage the hook. (allow the ladder to move forward) Devin 
              loaderPiston.set(DoubleSolenoid.Value.kForward);  
              chassis.mecanumDrive_Polar(0.0,0.0,0.0); //Turn off Mecanum Driv
              Timer.delay(0.70);
              hook.set(unlatched);
              Timer.delay(0.5);
              reload();
              // startHookRelease();
               //}else if (controller.getRawButton(button_B)){ //this means that you press the A  button to engage the hook. (allow the ladder to back) Devin
               //    startHookLatch();
               }
            //hookUpdate();       
            if(controller.getRawButton(button_leftBumper)){
                releaseWinch(SmartDashboard.getNumber("superLong_BeltLength", -18.0));
            }
            if (controller.getRawButton(button_Start)) {
                //releaseWinch(prefs.getDouble("belt", -10.0));
                releaseWinch(SmartDashboard.getNumber("long_BeltLength", -10.0));
            }      
            if(controller.getRawButton(button_X)){
                releaseWinch(SmartDashboard.getNumber("mediumShort_BeltLength", -4.5));
            }
            if(controller.getRawButton(button_B)){
                releaseWinch(SmartDashboard.getNumber("short_BeltLength", -2.0));
            }
            if(controller.getRawButton(button_Y)){
                releaseWinch(SmartDashboard.getNumber("mediumLong_BeltLength", -7.0));
            }
              //*if button 7 (Back button) is pressed, 
            if (controller.getRawButton(button_Back)){
                reload();   
            }
        
        if (controller.getRawButton(button_rightBumper)){
                    loaderPiston.set(DoubleSolenoid.Value.kForward);
                    loaderArm1.set(-.75);
                    loaderArm2.set(.75);
        }
        else{
                    loaderPiston.set(DoubleSolenoid.Value.kReverse);
                    loaderArm1.set(0);
                    loaderArm2.set(0);
        }
                
        
//            try {
//                Shooting();
//            } catch (CANTimeoutException ex) {
//                ex.printStackTrace();
//            }
            Driving();
           
            Timer.delay(0.01);
         }
        
        compressor.stop();   
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test(){
    }
    
    /*
     * Primary control methods including camera, shooting, driving, picking up, and output which are called once per loop through operator control 
    */
    
    public void Output() {
        printCounter++;

          if (printCounter == 100) {       //this will prevent the print screen from getting cluttered
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
        robot.println(DriverStationLCD.Line.kUser2, 1, "Encoder Winch " + winchEncoder.getDistance());
        //robot.println(DriverStationLCD.Line.kUser3, 1, "AccelX: " + accel.getAcceleration(ADXL345_I2C.Axes.kX));
        //robot.println(DriverStationLCD.Line.kUser4, 1, "AccelY: " + accel.getAcceleration(ADXL345_I2C.Axes.kY));
        //robot.println(DriverStationLCD.Line.kUser5, 1, "AccelZ: " + accel.getAcceleration(ADXL345_I2C.Axes.kZ));
        robot.updateLCD();
    }
        
    public void Driving(){
            //chassis.mecanumDrive_Polar(left.getMagnitude(), left.getDirectionDegrees(), right.getX());
            // chassis.mecanumDrive_Polar(/*addDeadZone*/(lowerSensitivity(controller.getMagnitude())), controller.getDirectionDegrees(), controller.getRawAxis(axis_rightStick_X));
            chassis.mecanumDrive_Polar(/*addDeadZone*/(lowerSensitivity(controller.getMagnitude())), controller.getDirectionDegrees(), /*addDeadZone*/(lowerSensitivity(controller.getRawAxis(axis_rightStick_X))));
            //chassis.mecanumDrive_Cartesian(/*addDeadZone*/(lowerSensitivity(controller.getX())), /*addDeadZone*/(lowerSensitivity(controller.getY())), controller.getTwist(), gyro.getAngle());
    }
    
    //function to start hooking the latch
    private void startHookLatch(){
        if (hookUpdateState == HOOK_READY){
            hook.set(latched);
        }
    }
    
    //function to start the hook release process
    private void startHookRelease(){
        if (hookUpdateState !=HOOK_READY)
            return;
        loaderPiston.set(DoubleSolenoid.Value.kForward);  
        hookUpdateStartTime = timer.get();
        hookUpdateState = HOOK_WAIT_1;
        }
    
    //update the hook status
    private void hookUpdate(){
        if (hookUpdateState == HOOK_READY){
            //Do Nothing...
        }
            else if ((hookUpdateState == HOOK_WAIT_1) && ((timer.get() - hookUpdateStartTime) > 0.2)){
                hook.set(unlatched);
                hookUpdateState = HOOK_WAIT_2;
            }
            else if ((hookUpdateState == HOOK_WAIT_2) && ((timer.get() - hookUpdateStartTime) >1.2)){
                reload();
                hookUpdateState = HOOK_READY;
            }
    }
            
    
    public void Shooting(){
//Encoder number
        int EncoderShootingValue = 0;
        
        if(controller.getRawButton(button_leftBumper)){//lb
            if(!limitSwitch.get() && hook.get() != DoubleSolenoid.Value.kForward){
                hook.set(DoubleSolenoid.Value.kForward);
            }
            if(limitSwitch.get()){
                winch1.set(.5);
                winch2.set(.5);
            }
            
            else if(!limitSwitch.get() && hook.get() == DoubleSolenoid.Value.kForward && winchEncoder.get() < EncoderShootingValue){
                winch1.set(-.5);
                winch2.set(-.5);
            }
        }
        
        if(controller.getRawButton(button_rightBumper)){
            hook.set(DoubleSolenoid.Value.kReverse);
        }
        pistonPrevState = hook.get();
        
        if (controller.getRawButton(button_Back)){
            /*while(limitSwitch.get()){
                winch1.set(-.25);
                winch2.set(-.25);
            }
            winch1.set(0);
            *///winch2.set(0);
            //winchEncoder.free();
            winchEncoder.reset();
        }
        //loaderArm1.set(controller.getRawAxis(axis_triggers));
        
        //loaderArm2.set(-controller.getRawAxis(axis_triggers));
    }
        public void dash(){ //displays dashboard values
        
            SmartDashboard.putNumber("controllerA1", controller.getRawAxis(axis_leftStick_X));
            //SmartDashboard.putNumber("AccelX", accel.getAcceleration(ADXL345_I2C.Axes.kX));
            //SmartDashboard.putNumber("AccelY", accel.getAcceleration(ADXL345_I2C.Axes.kY));
            //SmartDashboard.putNumber("AccelZ", accel.getAcceleration(ADXL345_I2C.Axes.kZ));
            //SmartDashboard.putNumber("Angle of Ladder", com.sun.squawk.util.MathUtils.atan2((double)accel.getAcceleration(ADXL345_I2C.Axes.kX), (double)accel.getAcceleration(ADXL345_I2C.Axes.kY)));
            SmartDashboard.putNumber("winchEncoder", winchEncoder.getDistance());
            //SmartDashboard.putNumber("Gyro", gyro.getAngle());
            SmartDashboard.putNumber("Distance", distance);
            
            SmartDashboard.putNumber("Super Long Belt Length", superLong_BeltLength);
            SmartDashboard.putNumber("Long Belt Length", long_BeltLength);
            SmartDashboard.putNumber("Medium Short Belt Length", mediumShort_BeltLength);
            SmartDashboard.putNumber("Medium Long Belt Length", mediumLong_BeltLength);
            SmartDashboard.putNumber("Short Belt Length", short_BeltLength);


            SmartDashboard.putNumber("winchEncoderDisPerPulse", winchEncoder.getRaw());
            SmartDashboard.putBoolean("Limit Switch",limitSwitch.get() );
            SmartDashboard.putNumber("Distance from Target", distance);

        }

    private void distanceCalculate() {
        ColorImage image = null;
        BinaryImage thresholdRGBImage = null;
        BinaryImage thresholdHSIImage = null;
        BinaryImage bigObjectsImage= null;
        BinaryImage convexHullImage=null;
        BinaryImage filteredImage = null;
        
        try {
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
            if (reports.length > 0) {
                double pixelsHeight = reports[0].boundingRectHeight;
//                double centerX = reports[0].center_mass_x_normalized;
//                double centerY = reports[0].center_mass_y_normalized;
//                double targetAngle = 47*(centerX);
                double angle = pixelsHeight/pixelsV*(vertAngle*Math.PI/180);
                double Vdistance = targetHeight/angle;
                visionDistance = Vdistance;

                SmartDashboard.putNumber("Distance: ", Vdistance);
            } else {
//                robot.println(DriverStationLCD.Line.kUser6, 1, "no targets. :(");
            }
        }catch (Exception ex) {
            ex.printStackTrace();
        }finally{
        }
        try {
            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            //thresholdRGBImage.free();
            thresholdHSIImage.free();
            image.free();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }
    }
    
    /*
     * Misc utility methods that might be called by various tasks
    */
    
    public double lowerSensitivity(double magnitude){
        double f = (com.sun.squawk.util.MathUtils.pow(magnitude,3));
        return f;
    }
    
    public double addDeadZone(double speed){
        double deadzone = .1;
        if(Math.abs(speed)<deadzone){
            speed = 0;
        }
        return speed;
    }
}

