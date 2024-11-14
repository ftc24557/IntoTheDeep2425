package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "BracoHeitor")
public class BracoHeitor extends LinearOpMode {

    private Servo servoSlider;
    private Servo servoSlider2;

    @Override
    public void runOpMode() throws InterruptedException {

        servoSlider = hardwareMap.get(Servo.class, "servoSlider");
        servoSlider2 = hardwareMap.get(Servo.class, "servoSlider2");
        servoSlider.setPosition(1);
        servoSlider2.setPosition(1);

        double position = 0.35;
        double position2 = 0.35;
        boolean apertadaMonstra = false;

        waitForStart();

        while (opModeIsActive()) {
            /* Com Bumper
            if (gamepad1.right_bumper) {
                if (!apertadaMonstra) {
                    position = 0.35;
                    apertadaMonstra = true;
                    sleep(200);
                } else {
                    position = 1;
                    apertadaMonstra = false;
                    sleep(200);
                }
*/
            // Com JoyStick
            position+=gamepad1.left_stick_x*0.01;
            if (position>1){
                position = 1;
            } else if (position<0.35){
                position = 0.35;
            }


                position2 = position;

                servoSlider.setPosition(position);
                servoSlider2.setPosition(position2);

                telemetry.addData("Posição:", position);
                telemetry.update();
            }
        }
    }
