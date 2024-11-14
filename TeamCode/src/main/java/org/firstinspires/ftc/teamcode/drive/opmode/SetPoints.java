package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "SetPoints")

public class SetPoints extends LinearOpMode {

    DcMotor elevador;
    DcMotor elevador2;

    ElapsedTime timer = new ElapsedTime();

    double lastTimestamp = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lastTimestamp = timer.seconds();
        elevador = hardwareMap.dcMotor.get("elevador");
        elevador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTimestamp = timer.seconds();
        elevador2 = hardwareMap.dcMotor.get("elevador2");
        elevador2.setDirection(DcMotor.Direction.REVERSE);
        elevador2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevador.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        elevador2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        waitForStart();

        while (opModeIsActive()) {
            elevador.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            elevador2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            telemetry.addData("elevador", elevador.getCurrentPosition());
            telemetry.update();
        }
    }
}
