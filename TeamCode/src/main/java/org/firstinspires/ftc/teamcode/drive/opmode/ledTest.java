package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "ledTest")
public class ledTest extends LinearOpMode {

    private Servo led1;
    private Servo led2;
    private Servo led3;

    @Override
    public void runOpMode() throws InterruptedException {
        led1 = hardwareMap.get(Servo.class, "red");
        led2 = hardwareMap.get(Servo.class, "green");
        led3 = hardwareMap.get(Servo.class, "blue");

        led1.setPosition(0);
        led2.setPosition(0);
        led3.setPosition(0);

        boolean acesa1 = false;
        boolean acesa2 = false;
        boolean acesa3 = false;

        waitForStart();

            while (opModeIsActive()) {

                if (gamepad1.x){
                    acesa1 = true;
                } else {
                    acesa1 = false;
                }
                if (gamepad1.a){
                    acesa2 = true;
                } else {
                    acesa2 = false;
                }
                if (gamepad1.b){
                    acesa3 = true;
                } else {
                    acesa3 = false;
                }

           if (acesa1) {
                led1.setPosition(1);
            } else if (!acesa1){
                led1.setPosition(0);
            }

            if (acesa2) {
                led2.setPosition(1);
            } else if (!acesa2){
                led2.setPosition(0);
            }

            if (acesa3) {
                led3.setPosition(1);
            } else if (!acesa3){
                led3.setPosition(0);
            }

            telemetry.addData("acesa1", acesa1);
            telemetry.addData("acesa2", acesa2);
            telemetry.addData("acesa3", acesa3);
            telemetry.update();
        }
    }
}