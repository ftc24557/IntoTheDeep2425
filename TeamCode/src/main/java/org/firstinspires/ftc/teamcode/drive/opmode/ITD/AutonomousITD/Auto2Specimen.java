package org.firstinspires.ftc.teamcode.drive.opmode.ITD.AutonomousITD;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
@Autonomous
public class Auto2Specimen extends LinearOpMode {
    ColorSensor distanceSensor;

    DcMotor encoderY;
    DcMotor encoderX;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    double WHEEL_RADIUS = 1.89/2;
    double GEAR_RATIO = 1;
    double TICKS_PER_REV = 2000.0;

    private Servo servoSlider;
    private Servo servoClaw;
    private Servo servoPivot;
    private DcMotor pivotArm;

    public void runOpMode(){
        distanceSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        pivotArm = hardwareMap.get(DcMotor.class, "pivotArm");
        servoSlider = hardwareMap.servo.get("servoSlider");
        servoClaw = hardwareMap.servo.get("claw");
        servoPivot = hardwareMap.servo.get("pivotIntake");

        encoderX = hardwareMap.get(DcMotor.class, "perpendicularEncoder");
        encoderY = hardwareMap.get(DcMotor.class, "parallelEncoder"); //odometria
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);
        imu.resetYaw();

        //setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        pivotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        armUp();
        servoSlider.setPosition(1);
        servoPivot.setPosition(0.6);
        servoClaw.setPosition(1);
        sleep(700);
        driveForward(30);
        servoClaw.setPosition(0);
        driveForward(-14);
        turnToAngle(180);
        servoPivot.setPosition(0.45);
        armDown();
        driveSideways(-30);
        servoClaw.setPosition(0);
        sleep(1500);
        servoClaw.setPosition(1);
        sleep(1000);
        armUp();
        turnToAngle(180);;
        servoPivot.setPosition(0.6);
        driveSideways(-30);
        driveForward(12
        );
        servoClaw.setPosition(0);
        driveForward(-20);
        while (!isStopRequested()){

        }
    }
    public double mod(double input){
        return input*(input<0?-1:1);
    }
    public void armOff(){
        pivotArm.setPower(0);
        pivotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void armUp(){
        int setpoint = 10;
        pivotArm.setTargetPosition(setpoint);
        pivotArm.setPower(0.07);
    }
    public void armDown(){
        int setpoint = -900;
        pivotArm.setTargetPosition(setpoint);
        pivotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double treshold = 100;
        double kp = 0.00075;
        double error = setpoint - pivotArm.getCurrentPosition();
        while (mod(error)>treshold){

            error = setpoint - pivotArm.getCurrentPosition();
            pivotArm.setPower(error*kp);
            telemetry.addLine(""+ pivotArm.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addLine("got to position");
        telemetry.update();
        pivotArm.setPower(0.1);
    }
    public void alignToIntake() {
        double targetDistance = 9;  // Distância desejada em cm
        double threshold = 1.0;  // Limite de erro aceitável em cm
        double kP = 0.0;  // Proporcionalidade do controlador para ajustar a distância
        double currentDistance = ((OpticalDistanceSensor) distanceSensor).getLightDetected();

        double error = targetDistance - currentDistance;
        // Iniciar o alinhamento com o sensor de distância
        while (Math.abs(error) > threshold) {
            currentDistance = ((OpticalDistanceSensor) distanceSensor).getLightDetected();
            telemetry.addLine("" + currentDistance);
            telemetry.update();
            // Verifica a diferença entre a distância atual e a desejada
            error = targetDistance - currentDistance;

            // Se a diferença for maior que o limiar de erro, continue ajustando
            if (Math.abs(error) > threshold) {
                // Ajusta a potência dos motores com base no erro de distância
                double power = error * kP;

                // Controla a movimentação do robô para alinhar com a parede
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(power);
                backLeftMotor.setPower(power);
                backRightMotor.setPower(power);
            }

        }
        telemetry.addLine("DONE");
        telemetry.update();
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public void turnToAngle(double angle){
        double treshold = 2/2;
        double kP = 0.005;
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = angle - currentHeading;
        while (mod(error)>treshold){
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = angle - currentHeading;
            frontLeftMotor.setPower(-error*kP);
            frontRightMotor.setPower(+error*kP);
            backLeftMotor.setPower(-error*kP);
            backRightMotor.setPower(+error*kP);
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);



    }
    public void driveSideways(double distance){
        resetPositions();
        double x = getEncoderX();  // Usa o encoder X para medir a distância percorrida lateralmente
        double distanceError = distance - x;
        double threshold = 1;

        double kPdistance = 0.01; // Proporcional para a distância
        double kPangle = 0.006;    // Proporcional para o controle de ângulo

        while (mod(distanceError) > threshold) {
            x = getEncoderX(); // Atualiza a posição do encoder X
            distanceError = distance - x; // Calcula o erro de distância

            double distP = distanceError * kPdistance; // Controle proporcional para a distância

            // Pegue o ângulo atual do robô
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double angleError = 0 - currentAngle; // Queremos manter o ângulo em 0 para movimento lateral

            double angleP = angleError * kPangle; // Controle proporcional para o ângulo

            // Aqui ajustamos os motores para mover lateralmente
            frontLeftMotor.setPower(distP-angleP);
            frontRightMotor.setPower(-distP+angleP);
            backLeftMotor.setPower(-distP-angleP);
            backRightMotor.setPower(distP+angleP); // Lado direito (movimento oposto ao esquerdo)
        }

        // Quando atingir a distância, pare os motores
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void driveForward(double distance){
        resetPositions();
        double y = getEncoderY();
        double distanceError = distance-y;
        double treshold = 1;

        double kPdistance = 0.015;
        double kPangle = 0.005;


        while (mod(distanceError)>treshold) {
            y = getEncoderY();
            distanceError = distance - y;

            double distP = distanceError*kPdistance;

            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double angleError = 0-currentAngle;


            double angleP = angleError*kPangle;

            frontLeftMotor.setPower(distP-angleP);
            frontRightMotor.setPower(distP+angleP);
            backLeftMotor.setPower(distP-angleP);
            backRightMotor.setPower(distP+angleP);
        }
    }
    public void driveStop(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void resetPositions(){
        imu.resetYaw();
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getEncoderY(){
        double ticks = encoderY.getCurrentPosition();

        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public double getEncoderX(){
        double ticks = encoderX.getCurrentPosition();

        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
