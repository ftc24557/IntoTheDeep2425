package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "PivotIntake")
public class PivotIntake extends LinearOpMode {

    private Servo servoSlider;

    @Override
    public void runOpMode() throws InterruptedException {

        servoSlider = hardwareMap.get(Servo.class, "pivotIntake1");
        servoSlider.setPosition(0);

        double position = 0;

        waitForStart();

        while (opModeIsActive()) {

            // Com JoyStick
            position+=gamepad1.left_stick_x*0.01;
            if (position<0){
                position = 0;
            } else if (position>1){
                position = 1;
            }

            servoSlider.setPosition(position);

            telemetry.addData("Posição:", position);
            telemetry.update();
        }
    }
}
