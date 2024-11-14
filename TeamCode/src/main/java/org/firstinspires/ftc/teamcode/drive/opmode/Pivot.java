package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "Pivot")
public class Pivot extends LinearOpMode {

    private Servo servoSlider;
    private Servo servoSlider2;
    private CRServo servoIntake;
    private Servo servoPivotIntake;
    @Override
    public void runOpMode() throws InterruptedException {

        servoPivotIntake = hardwareMap.get(Servo.class, "pivotIntake1");
        servoIntake = hardwareMap.get(CRServo.class, "intake");
        servoSlider = hardwareMap.get(Servo.class, "servoSlider");
        servoSlider2 = hardwareMap.get(Servo.class, "servoSlider2");
        servoSlider.setPosition(0);
        servoSlider2.setPosition(0);

        double positionSlider = 0;
        double positionPivotIntake = 0;
        double power = 0;

        waitForStart();

        while (opModeIsActive()) {
            positionSlider = 0.35;
            positionPivotIntake = 0;
            power = 0;
            if (gamepad1.right_trigger>0.5){
                positionSlider = 1;
                power = -1;
                positionPivotIntake = 1;

            } else if (gamepad1.left_trigger>0.5){

                positionSlider = 1;
                positionPivotIntake = 0.5;

                if (gamepad1.x){
                    power = 1;
                }
            }
            servoPivotIntake.setPosition(positionPivotIntake);
            servoIntake.setPower(power);
            servoSlider.setPosition(positionSlider);
            servoSlider2.setPosition(positionSlider);
            telemetry.addData("Posição:", positionSlider);
            telemetry.update();

        }
    }

}
