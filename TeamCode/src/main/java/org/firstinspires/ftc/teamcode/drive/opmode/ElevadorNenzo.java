package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Config
@TeleOp(name = "ElevadorNenzo")

public class ElevadorNenzo extends LinearOpMode {

    public static double kP = 0.045;
    public static double kI = 0.0025;
    public static double kD = 0.0005;
    public static double kF = 0;
    public static double positionLift = 0;
    double position = 0;
    DcMotor elevador;
    DcMotor elevador2;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    ElapsedTime timer = new ElapsedTime();

    double setPoint = 0;
    double lastError = 0;
    double integral = 0;
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

        waitForStart();

        while (opModeIsActive()) {
            double currentPosition = elevador.getCurrentPosition();
            pidf.setP(kP);
            pidf.setI(kI);
            pidf.setD(kD);
            pidf.setF(kF);
            double output = pidf.calculate(currentPosition, positionLift);
            elevador.setPower(output);
            elevador2.setPower(output);
            telemetry.addData("currentPosition", currentPosition);
            telemetry.addData("positionLift", positionLift);
            telemetry.update();
        }
    }
}
