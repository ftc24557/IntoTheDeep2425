package org.firstinspires.ftc.teamcode.drive.opmode.ITD.TeleOpITD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.Servo;
@Config

@TeleOp(group="TeleOp - Into The Deep")
public class TeleOpITD1 extends LinearOpMode {
    public static double kpPivot = 0;
    public static double kiPivot = 0;
    public static double kdPivot = 0;
    public static double kfPivot = 0.035;
    public static double setPointPivot = 0;
    private PIDController pidPivotArm = new PIDController(kpPivot, kiPivot, kdPivot);
    private ColorSensor colorSensor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu;
    private DcMotor pivot;
    private Servo servoSlider;
    private Servo servoClaw;
    private Servo servoPivot;
    private String mode;

    double servoPivotPos = 0.2;
    @Override
    public void runOpMode(){
        String redEmojis = "\uD83D\uDFE5";
        String blueEmojis = "\uD83D\uDFE6";
        String yellowEmojis = "\uD83D\uDFE8";

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);
        imu.resetYaw();

        pivot = hardwareMap.dcMotor.get("pivotArm");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servoSlider = hardwareMap.servo.get("servoSlider");
        servoClaw = hardwareMap.servo.get("claw");
        servoPivot = hardwareMap.servo.get("pivotIntake");


        waitForStart();

        while (opModeIsActive()){

            pivot.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            extendSlider();

            drive();

            handleModesChange();

            pivotSubSystem();
            if (gamepad2.x){
                servoClaw.setPosition(1);
            } else {
                servoClaw.setPosition(0);
            }
            telemetryColors();
        }
    }

    private void telemetryColors(){
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        String detectedColor = "null";
        if (red > green && red > blue) {
            detectedColor = "red";
        } else if (green > red && green > blue) {

            detectedColor = "yellow";
        } else if (blue > red && blue > green) {
            detectedColor = "blue";
        }
        telemetry.addData("color: ", detectedColor);
        telemetry.addData("distance: ", (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)));
        telemetry.update();
    }
    private void pivotSubSystem(){
        servoPivotPos+=gamepad2.left_stick_y*0.01;
        if (servoPivotPos<0){
            servoPivotPos = 0;
        } else if (servoPivotPos>1){
            servoPivotPos = 1;
        }
        servoPivot.setPosition(servoPivotPos);
    }


    /* private void PIDpivotArm(){
        double ticksInDegree = 8192/360;
        setPointPivot+=((gamepad2.dpad_down?-1:0)+(gamepad2.dpad_up?+1:0))*0.1;

        double ff = Math.cos(Math.toRadians(setPointPivot/ticksInDegree))*kfPivot;

        if (setPointPivot<100){
            ff = 0;
            kpPivot = 0;
            kiPivot = 0;
            kdPivot = 0;
        }
        pidPivotArm.setPID(kpPivot,kiPivot,kdPivot);
        double pidOutput = pidPivotArm.calculate(pivot.getCurrentPosition(), setPointPivot);
        pivot.setPower(pidOutput+ff);
        telemetry.addData("CurrentPos", pivot.getCurrentPosition());
        telemetry.addData("target", setPointPivot);

        telemetry.update(); // Show PID output for debugging
    } */



    private void handleModesChange(){
        if (gamepad2.dpad_up){
            mode = "outtake";
        } else if (gamepad2.dpad_down){
            mode = "intake";
        }
    }



    private void extendSlider(){
        servoSlider.setPosition(1);
    }

    private void distendSlider(){
        servoSlider.setPosition(0);
    }

    private void closeClaw(){
        servoClaw.setPosition(1);
    }

    private void openClaw(){
        servoClaw.setPosition(0);
    }

    private void drive(){
        if (gamepad1.start){
            imu.resetYaw();
        }
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x*0.7;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
