package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Tragetoria6 extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Cria uma trajetória com o método correto
        drive.setPoseEstimate(new Pose2d(22.57, -65.00, Math.toRadians(90)));
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(22.57, -65.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(0.45, -40.00), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(36.56, -40.00))
                .splineToConstantHeading(new Vector2d(39.95, -14.00), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(48.75, -14.00))
                .lineToConstantHeading(new Vector2d(48.75, -65.00))
                .build();

        // Aguardar o início do modo autônomo
        waitForStart();

        if (isStopRequested()) return;

        // Seguir a trajetória criada
        drive.followTrajectorySequence(trajectory0);

    }
}