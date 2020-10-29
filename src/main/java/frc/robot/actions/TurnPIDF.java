package frc.robot.actions;

import frc.robot.subsystems.Drivebase;

public class TurnPIDF extends Action {
    // Temporary, will be replaced with a reference
    Drivebase drivebase = new Drivebase();

    //Use 0 degrees as forward and 90 degrees as to the right
    //Should range from -180 to 180
    public double angle;
    //Speed will be used to multiply the PIDF output
    public double speed;
    //When finding encoder values try to make sure that both wheels are turning at the same speed
    //Encoder value for top left wheel when turned to 90 degrees
    private static double encoderTo90L;
    //Encoder value for top right wheel when turned to 90 degrees
    private static double encoderTo90R;

    private double desiredEncoderL;
    private double desiredEncoderR;
    private double currentEnconderL;
    private double currentEncoderR;
    private CANEncoder enconderL;
    private CANEncoder encoderR;

    CANSparkMax topLeftWheel;
    CANSparkMax topRightWheel;
    CANSparkMax bottomLeftWheel;
    CANSparkMax bottomRightWheel;
    
    private PIDF leftPIDF;
    private PIDF rightPIDF;

    private long pastTime;
    private double deltaTime;

    public TurnPIDF(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
        pastTime = System.currentTimeMillis();
    }

    // TODO: Control the drivebase using a PIDF in order to 
    // TODO: make the robot turn to a given angle

    @Override
    public void start() {
        // TODO Auto-generated method stub
        encoderTo90L = 1000;
        encoderTo90R = 1000;
        //Gets the values for each encoder
        currentEnconderL = encoderL.getEncoder();
        currentEncoderR = encoderR.getEncoder();
        //Adds the desired end encoder value with the current encoder value
        desiredEncoderL = ((angle/90) * encoderTo90L) + currentEnconderL;
        desiredEncoderR = ((angle/90) * encoderTo90R) + currentEncoderR;
        //Need to tune PIDs
        leftPIDF = new PIDF(0, 0, 0, 0);
        rightPIDF = new PIDF(0, 0, 0, 0);
        pastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // TODO Auto-generated method stub

        //Get both current encoder values
        currentEnconderL = encoderL.getEncoder();
        currentEncoderR = encoderR.getEncoder();

        deltaTime = System.currentTimeMillis() - pastTime;

        //Change value based on encoder accuracy
        double marginOfError = 10;

        //While actual encoder values don't equal around (Margin of error depends on typical encoder values) the desired values
        if (!((currentEnconderL < desiredEncoderL + marginOfError) || (currentEnconderL > desiredEncoderL - marginOfError)) && !((currentEnconderR < desiredEncoderR + marginOfError) || (currentEnconderR > desiredEncoderR - marginOfError))) {
            //Sets values for the speeds that the motors on each side of the robot should follow
            double leftMotorSpeed = speed * leftPIDF.update(desiredEncoderL, currentEncoderL, deltaTime);
            double rightMotorSpeed = speed * rightPIDF.update(desiredEncoderR, currentEncoderR, deltaTime);
            topLeftWheel.set(leftMotorSpeed);
            topRightWheel.set(rightMotorSpeed);
            bottomLeftWheel.set(leftMotorSpeed);
            bottomRightWheel.set(rightMotorSpeed);
            pastTime = System.currentTimeMillis();
        }
        else {
            topLeftWheel.set(0);
            topRightWheel.set(0);
            bottomLeftWheel.set(0);
            bottomRightWheel.set(0);
        }
    }

    @Override
    public ActionState getState() {
        // TODO Auto-generated method stub
        return null;
    }
}
