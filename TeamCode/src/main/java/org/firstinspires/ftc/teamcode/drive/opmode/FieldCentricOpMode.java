package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
@TeleOp
public class FieldCentricOpMode extends LinearOpMode {
    /* public static double kpPivot = 0;
    public static double kiPivot = 0;
    public static double kdPivot = 0;
    public static double kfPivot = 0; */
    // public static double setPointPivot = 0;
    double currentPositionPivot;
    // PIDFController pidPivot = new PIDFController(kpPivot,kiPivot, kdPivot, kfPivot);
    @Override
    public void runOpMode() throws InterruptedException {

        // drive motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();

        // pivot arm motors


        DcMotor pivot = hardwareMap.dcMotor.get("pivotArm");
        Servo servoSlider = hardwareMap.servo.get("servoSlider");
        Servo servoClaw = hardwareMap.servo.get("claw");
        Servo servoPivot = hardwareMap.servo.get("pivotIntake");
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double positionPivot = 0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.start){
                imu.resetYaw();
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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


            pivot.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            if (gamepad1.right_bumper) {
                servoClaw.setPosition(1);
            } else {
                servoClaw.setPosition(0);
            }
            if (gamepad2.right_bumper){
                servoSlider.setPosition(1);
            } else {
                servoSlider.setPosition(0.25);
            }
            positionPivot+= gamepad2.left_stick_y*0.05;
            if (positionPivot<0){
                positionPivot = 0;
            } else if (positionPivot>1){
                positionPivot = 1;
            }
            servoPivot.setPosition(positionPivot);
            currentPositionPivot = pivot.getCurrentPosition();

            // pidPivot.setPIDF(kpPivot, kiPivot, kdPivot, kfPivot);
            // double outputPivot = pidPivot.calculate(currentPositionPivot, setPointPivot);
            // pivot.setPower(outputPivot);
            telemetry.addData("CurrentPosition", positionPivot);
            // telemetry.addData("TargetPosition", setPointPivot);


            telemetry.update();
        }
    }

}