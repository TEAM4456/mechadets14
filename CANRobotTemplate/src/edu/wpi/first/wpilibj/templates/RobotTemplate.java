/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//CANRobotTemplate

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
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    RobotDrive chassis;
    Joystick controller;
    Gyro gyro;
    ADXL345_I2C accel;
    //Drivers Station output box declaration
    DriverStationLCD robot = DriverStationLCD.getInstance();
    CANJaguar winch1, winch2, loaderArm1, loaderArm2, MotorLF, MotorLB, MotorRF, MotorRB; //MotorRF(R)ight(F)ront, MotorLB(L)eft(B)ack
    Compressor compressor;
    DoubleSolenoid hook, loaderPiston;
    Servo servoVertical, servoHorizontal;
    AxisCamera camera;
    DigitalInput limitSwitch;
    Encoder winchEncoder;
    Relay ledLight;
    
    private int printCounter = 0;
    private int encoderCounter;
    private Value pistonPrevState;
    private String CANJaguarConnectionFailure = "Can't connect to CANJag: "; 
    
    private boolean motorsInitialized = false;
    private boolean winchesInitialized = false;
    private boolean armsInitialized = false;
    
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
    double beltLength;
    Timer timer = new Timer(); 
    
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
    
    //declares components of robot
    public void robotInit(){
        //prefs = Preferences.getInstance();
        
        //the two doubles that are written in the following two lines are default values and can be changed. Change the default values to the starting values of the robot in autonamous. Devin
        SmartDashboard.putNumber("beltLength", -10.5);
        limitSwitch = new DigitalInput(3);//true = open; false = close;
        
        try {
            MotorLF = new CANJaguar(13);
            MotorLB = new CANJaguar(6);
            MotorRF = new CANJaguar(15);
            MotorRB = new CANJaguar(16);
            motorsInitialized = true;
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
        
        if (motorsInitialized) {
            chassis = new RobotDrive(MotorLF, MotorLB, MotorRF, MotorRB);
        }
        
        controller = new Joystick(1);
        gyro = new Gyro(1);
        accel = new ADXL345_I2C(1, ADXL345_I2C.DataFormat_Range.k4G);
        winchEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k1X);
        //winchEncoder = new Encoder(1, 2);
        winchEncoder.setDistancePerPulse(1.0/750.0);
        ledLight = new Relay(2);
              
        try {
            winch1 = new CANJaguar(7);
            winch2= new CANJaguar(10);
            winchesInitialized = true;
        } catch (CANTimeoutException ex) {
            CANJaguarConnectionFailure += "5";
            ex.printStackTrace();
        }
        
        //Assigns Compressor and hook.
        compressor = new Compressor(4,1);
        hook = new DoubleSolenoid(1,2);
        
        try {
            loaderArm1 = new CANJaguar(12);
            loaderArm2 = new CANJaguar(9);
            armsInitialized = true;
        } catch (CANTimeoutException ex) {
            CANJaguarConnectionFailure += ", 9";
            ex.printStackTrace();
        }

        
        loaderPiston = new DoubleSolenoid(3,4);
        
        /*
        ------------------------------------------------------------
        */
        servoVertical = new Servo(7);
        servoHorizontal = new Servo(8);
        //Vision Processing
        camera = AxisCamera.getInstance();
        cc = new CriteriaCollection();
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false);
        visionDistance = 0;
        VisionCounter = 0;
        //cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_ORIENTATION, -0.2, 0.2, false);
        pistonPrevState = DoubleSolenoid.Value.kOff;
        timer.start();
        
    }
    
    //initializes components of robot
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    
    //need to work on this.
    
    //This is for the hook 
    DoubleSolenoid.Value latched = DoubleSolenoid.Value.kReverse;
    DoubleSolenoid.Value unlatched = DoubleSolenoid.Value.kForward;

    public void autonomous() {
//        gyro.reset();
//        distanceCalculate();
//        ledLight.set(Relay.Value.kOn);
//        winchEncoder.reset();
//        while(winchEncoder.getDistance() > -9.8){
//            try {
//                winch1.setX(.5);
//                winch2.setX(.5);
//            } catch (CANTimeoutException ex) {
//                ex.printStackTrace();
//            }
//        }
//        hook.set(unlatched);
//        chassis.mecanumDrive_Polar(1.0, 0, 0);
//        timer.delay(1.0);
//        chassis.mecanumDrive_Polar(0,0,0);
        
    }

    /** 
     * This function is called once each time the robot enters operator control.
     */
    
    public void reload() {
        try{
                if (this.winchesInitialized) {
                    // make sure the latch is open
                    hook.set(unlatched);

                    // *while the limit switch is pressed, push the winch negative
                    if(limitSwitch.get()){
                        winch1.setX(-.5);
                        winch2.setX(-.5);
                    }else{//*set winch to 0
                        winch1.setX(0);
                        winch2.setX(0);
                        hook.set(latched);
                        Timer.delay(0.5);
                        winchEncoder.reset();
                        if(winchEncoder.getDistance() > -4.8){
                            winch1.setX(.5);
                            winch2.setX(.5);
                        }
                    }
                    
                }
            }
            catch(CANTimeoutException ex){
                ex.printStackTrace();
            }
                        
            //winchEncoder.free();
            
            //*reset the encoder
    }
    public void releaseWinch(double distanceOfWinch){//use the encoder values from testing. Devin
        try{
            if(winchEncoder.getDistance() > distanceOfWinch){
                winch1.setX(.5);
                winch2.setX(.5);
            }else{
                winch1.setX(0.0);
                winch2.setX(0.0);
            }
        }
        catch(CANTimeoutException ex){
            ex.printStackTrace();
        }   
    }
    
    public void operatorControl() {
        
        while(isOperatorControl() && isEnabled()){            
            double angle = gyro.getAngle();
            compressor.start();            
            winchEncoder.start();
            ledLight.set(Relay.Value.kReverse);
            Output();
            dash();
//            if(VisionCounter >=100){
//                distanceCalculate();
//                VisionCounter = 0;
//            }
//            VisionCounter++;
            try {
                //these if elses test if bumpers are pressed in order to enable the winch. Will be set to zero if no bumpers are pressed.
//            if(controller.getRawButton(button_leftBumper)){
//                try {
//                    if (this.winchesInitialized) {
//                        winch1.setX(1);
//                        winch2.setX(1);
//                    }
//                } catch (CANTimeoutException ex) {
//                    ex.printStackTrace();
//                } 
//            }
//            else if(controller.getRawButton(button_rightBumper)){
//                try {
//                    if (this.winchesInitialized) {
//                        winch1.setX(-1);
//                        winch2.setX(-1);    
//                    }
//                } catch (CANTimeoutException ex) {
//                    ex.printStackTrace();
//                }
//            }
//            else{
//                try {
//                    if (this.winchesInitialized) {
//                        winch1.setX(0);
//                        winch2.setX(0);    
//                    }
//                } catch (CANTimeoutException ex) {
//                    ex.printStackTrace();
//                }
//            }
                
                winch1.setX(controller.getRawAxis(axis_triggers));
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
            try {
                winch2.setX(controller.getRawAxis(axis_triggers));
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
          
            if (controller.getRawButton(button_A)){//this means that you press the A  button to disengage the hook. (allow the ladder to move forward) Devin
                    loaderPiston.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(0.2);
                    hook.set(unlatched);
                    Timer.delay(1.0);
                    reload();

                } else if (controller.getRawButton(button_B)){//this means that you press the B button to engage the hook. (stop the ladder from moving forward) Devin
                    hook.set(latched);
                }
           
        if (controller.getRawButton(button_rightBumper)) {
            //releaseWinch(prefs.getDouble("belt", -10.0));
            releaseWinch(SmartDashboard.getNumber("beltLength", -10.0));
        }      
              //*if button 7 (Back button) is pressed, 
        if (controller.getRawButton(button_Back)){
            reload();
            
        }
        
        // if button 8 is pressed, 
        if (controller.getRawButton(button_Start)){
            if(loaderPiston.get()==DoubleSolenoid.Value.kReverse){
                try{
                    if (this.armsInitialized) {
                        loaderPiston.set(DoubleSolenoid.Value.kForward);
                        loaderArm1.setX(-1*controller.getRawAxis(axis_dPad_X));
                        loaderArm2.setX(controller.getRawAxis(axis_dPad_X));
                    }
                }
                catch(CANTimeoutException ex){
                    ex.printStackTrace();
                }
            }
            else if(loaderPiston.get()==DoubleSolenoid.Value.kForward){
                try{
                    if (this.armsInitialized) {
                        loaderPiston.set(DoubleSolenoid.Value.kReverse);
                        loaderArm1.setX(0);
                        loaderArm2.setX(0);
                    }
                }
                catch(CANTimeoutException ex){
                    ex.printStackTrace();
                }
            }
        }
        
//            try {
//                Shooting();
//            } catch (CANTimeoutException ex) {
//                ex.printStackTrace();
//            }
            Driving();
            Arms_Of_Glory(); // can someone please tell me what this method does and why it's here? _____________________
            Timer.delay(0.01);
         }
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
        robot.println(DriverStationLCD.Line.kUser1, 1, "GyroAngle " + gyro.getAngle());
        robot.println(DriverStationLCD.Line.kUser2, 1, "Encoder Winch " + winchEncoder.getDistance());
        robot.println(DriverStationLCD.Line.kUser3, 1, "AccelX: " + accel.getAcceleration(ADXL345_I2C.Axes.kX));
        robot.println(DriverStationLCD.Line.kUser4, 1, "AccelY: " + accel.getAcceleration(ADXL345_I2C.Axes.kY));
        robot.println(DriverStationLCD.Line.kUser5, 1, "AccelZ: " + accel.getAcceleration(ADXL345_I2C.Axes.kZ));
        if(CANJaguarConnectionFailure.length()>25){
            robot.println(DriverStationLCD.Line.kUser5, 1, CANJaguarConnectionFailure);
        }
        robot.updateLCD();
    }
        
    public void Driving(){
        if (this.motorsInitialized) {
            //chassis.mecanumDrive_Polar(left.getMagnitude(), left.getDirectionDegrees(), right.getX());
            // chassis.mecanumDrive_Polar(/*addDeadZone*/(lowerSensitivity(controller.getMagnitude())), controller.getDirectionDegrees(), controller.getRawAxis(axis_rightStick_X));
            chassis.mecanumDrive_Polar(/*addDeadZone*/(lowerSensitivity(controller.getMagnitude())), controller.getDirectionDegrees(), /*addDeadZone*/(lowerSensitivity(controller.getRawAxis(axis_rightStick_X))));
            //chassis.mecanumDrive_Cartesian(/*addDeadZone*/(lowerSensitivity(controller.getX())), /*addDeadZone*/(lowerSensitivity(controller.getY())), controller.getTwist(), gyro.getAngle());
        }
    }
    
    public void Shooting() throws CANTimeoutException{
//Encoder number
        int EncoderShootingValue = 0;
        
        if(controller.getRawButton(button_leftBumper)){//lb
            if(!limitSwitch.get() && hook.get() != DoubleSolenoid.Value.kForward){
                hook.set(DoubleSolenoid.Value.kForward);
            }
            if(limitSwitch.get()){
                winch1.setX(.5);
                winch2.setX(.5);
            }
            
            else if(!limitSwitch.get() && hook.get() == DoubleSolenoid.Value.kForward && winchEncoder.get() < EncoderShootingValue){
                winch1.setX(-.5);
                winch2.setX(-.5);
            }
        }
        
        if(controller.getRawButton(button_rightBumper)){
            hook.set(DoubleSolenoid.Value.kReverse);
        }
        pistonPrevState = hook.get();
        
        if (controller.getRawButton(button_Back)){
            /*while(limitSwitch.get()){
                winch1.setX(-.25);
                winch2.setX(-.25);
            }
            winch1.setX(0);
            *///winch2.setX(0);
            //winchEncoder.free();
            winchEncoder.reset();
        }
        //loaderArm1.setX(controller.getRawAxis(axis_triggers));
        
        //loaderArm2.setX(-controller.getRawAxis(axis_triggers));
    }
        public void dash(){ //displays dashboard values
        
            SmartDashboard.putNumber("controllerA1", controller.getRawAxis(axis_leftStick_X));
            SmartDashboard.putNumber("AccelX", accel.getAcceleration(ADXL345_I2C.Axes.kX));
            SmartDashboard.putNumber("AccelY", accel.getAcceleration(ADXL345_I2C.Axes.kY));
            SmartDashboard.putNumber("AccelZ", accel.getAcceleration(ADXL345_I2C.Axes.kZ));
            SmartDashboard.putNumber("Angle of Ladder", com.sun.squawk.util.MathUtils.atan2((double)accel.getAcceleration(ADXL345_I2C.Axes.kX), (double)accel.getAcceleration(ADXL345_I2C.Axes.kY)));
            SmartDashboard.putNumber("winchEncoder", winchEncoder.getDistance());
            SmartDashboard.putNumber("Gyro", gyro.getAngle());
            //SmartDashboard.putNumber("Distance", distance);
            //SmartDashboard.putNumber("Belt Length", beltLength);

            
            //SmartDashboard.putNumber("winchEncoderDisPerPulse", winchEncoder.get);

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
    
    public void Arms_Of_Glory(){
        //picking up arms
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

//4456