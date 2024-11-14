package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TesteIntake")
public class TesteIntake extends LinearOpMode {

    private Servo servoSlider;
    private Servo clawPower;
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        servoSlider = hardwareMap.get(Servo.class, "servoSlider");
        clawPower = hardwareMap.get(Servo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        servoSlider.setPosition(0.35);
        claw.setPosition(1);
        clawPower.setPosition(1);

        double sliderPos = 0.35;
        double clawPos = 1;
        double clawPower2 = 1;

        boolean modoClaw = false;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                modoClaw = !modoClaw;
            }

            double apertadaMonstra = gamepad1.right_trigger;

            if (apertadaMonstra == 0) {
                sliderPos = 0.35;
                clawPos = 1;
                clawPower2 = 1;
            } else if (apertadaMonstra >= 0.5) {
                sliderPos = 1;
                clawPos = 0;
                clawPower2 = 0;
            }
            if (modoClaw){
                clawPos = 0;
                clawPower2 = 0;

            } else if (!modoClaw){
                clawPos = 1;
                sleep(20);
                clawPower2 = 1;
            }

            servoSlider.setPosition(sliderPos);
            clawPower.setPosition(clawPower2);
            claw.setPosition(clawPos);

            telemetry.addData("Apertada:", apertadaMonstra);
            telemetry.addData("Modo:", modoClaw);
            telemetry.addData("Posição Claw:", clawPower2);
            telemetry.addData("Posição Slider:", sliderPos);
            telemetry.addData("Posição Claw2:", clawPos);
            telemetry.update();
        }
    }
}