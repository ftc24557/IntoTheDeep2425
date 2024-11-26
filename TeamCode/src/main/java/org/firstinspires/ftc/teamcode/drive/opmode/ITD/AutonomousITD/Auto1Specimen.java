package org.firstinspires.ftc.teamcode.drive.opmode.ITD.AutonomousITD;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
@Autonomous
public class Auto1Specimen extends LinearOpMode {


    DcMotor encoderY;
    DcMotor encoderX;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    double WHEEL_RADIUS = 1.89/2;
    double GEAR_RATIO = 1;
    double TICKS_PER_REV = 2000.0;

    private Servo servoSlider;
    private Servo servoClaw;
    private Servo servoPivot;

    public void runOpMode(){

        servoSlider = hardwareMap.servo.get("servoSlider");
        servoClaw = hardwareMap.servo.get("claw");
        servoPivot = hardwareMap.servo.get("pivotIntake");

        encoderX = hardwareMap.get(DcMotor.class, "perpendicularEncoder");
        encoderY = hardwareMap.get(DcMotor.class, "parallelEncoder"); //odometria
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);
        imu.resetYaw();

        //setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart();
        servoSlider.setPosition(1);
        servoPivot.setPosition(1);
        servoClaw.setPosition(1);
        driveForward(30);
        servoClaw.setPosition(0);
        driveForward(-10);
    }
    public double mod(double input){
        return input*(input<0?-1:1);
    }
    public void driveForward(double distance){
        resetPositions();
        double y = getEncoderY();
        double distanceError = distance-y;
        double treshold = 1;

        double kPdistance = 0.015;
        double kPangle = 0.005;


        while (mod(distanceError)>treshold) {
            y = getEncoderY();
            distanceError = distance - y;

            double distP = distanceError*kPdistance;

            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double angleError = 0-currentAngle;


            double angleP = angleError*kPangle;

            frontLeftMotor.setPower(distP-angleP);
            frontRightMotor.setPower(distP+angleP);
            backLeftMotor.setPower(distP-angleP);
            backRightMotor.setPower(distP+angleP);
        }
    }
    public void driveStop(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void resetPositions(){
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getEncoderY(){
        double ticks = encoderY.getCurrentPosition();

        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public double getEncoderX(){
        double ticks = encoderX.getCurrentPosition();

        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
