package org.firstinspires.ftc.teamcode.drive.opmode.ITD.TeleOpITD;

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
@TeleOp(group="TeleOp - Into The Deep")
public class TesteBlumenal extends LinearOpMode {
    public static double kpPivot = 0.0008;
    public static double kiPivot = 0.00002;
    public static double kdPivot = 0;
    public static double kfPivot = 0;
    public static double setPointPivot = 100;
    double currentPositionPivot;
    PIDFController pidPivot = new PIDFController(kpPivot,kiPivot, kdPivot, kfPivot);
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
        boolean bumperClaw = false;
        float positionClaw = 0;
        boolean bumperSlider = false;
        double positionSlider = 0;
        boolean bracoemcima = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Bumper da Claw
            if (!bumperClaw  && gamepad2.x){

                bumperClaw = true;
                if (positionClaw == 0){
                    positionClaw = 60;
                } else{
                    positionClaw = 0;
                }
            }
            if (bumperClaw && !gamepad2.x){

                bumperClaw = false;
            }
            servoClaw.setPosition(positionClaw);

            //Bumper do Slider
            if (!bumperSlider && gamepad2.right_bumper){
                bumperSlider = true;
                if (positionSlider == 0 && !bracoemcima){
                    positionSlider = 1;
                } else if (!bracoemcima){
                    positionSlider = 0;
                }
            }

            if (bumperSlider && !bracoemcima && !gamepad2.right_bumper){

                bumperSlider = false;
            }
            if (positionSlider == 0){
                kpPivot = 0;
                kiPivot = 0;
                kdPivot = 0;
                kfPivot = 0;
            } else if (positionSlider == 1){
                kpPivot = 0.0008;
                kiPivot = 0.00002;
                kdPivot = 0;
                kfPivot = 0;
            }


            servoSlider.setPosition(positionSlider);

                if (gamepad2.dpad_up && bracoemcima == false && positionSlider == 1){
                    setPointPivot = 1120;
                    bracoemcima = true;
                }

                if (gamepad2.dpad_down && bracoemcima == true){
                    setPointPivot = 100;
                    positionSlider = 0.25;
                    bumperSlider = false;
                    bracoemcima = false;
                }

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
            frontRightMotor.setPower(frontRightPower)   ;
            backRightMotor.setPower(backRightPower);


            //pivot.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            positionPivot+= gamepad2.left_stick_y*0.01;
            if (positionPivot<0){
                positionPivot = 0;
            } else if (positionPivot>1){
                positionPivot = 1;
            }

            servoPivot.setPosition(positionPivot);

            currentPositionPivot = pivot.getCurrentPosition();


            pidPivot.setPIDF(kpPivot, kiPivot, kdPivot, kfPivot);
            double outputPivot = pidPivot.calculate(currentPositionPivot, setPointPivot);
            pivot.setPower(outputPivot);
            telemetry.addData("CurrentPosition", currentPositionPivot);
            telemetry.addData("TargetPosition", setPointPivot);


            telemetry.update();
        }
    }

}