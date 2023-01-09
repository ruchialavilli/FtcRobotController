package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="test", group="AutoOpModes")
public class test extends BaseAutoOpMode {

    public void runOpMode() throws InterruptedException {

        //for testing small changes before adding

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //red2

        robot.arm = new DejaVuArm();
        robot.arm.init(hardwareMap, true);
        if(isStopRequested()) return;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("startX", poseEstimate.getX());
        telemetry.addData("startY", poseEstimate.getY());
        telemetry.addData("startHeading", poseEstimate.getHeading());
        telemetry.update();

        Pose2d startPose = new Pose2d(-63.375, 39, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1_1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0, 39))
                .build();
        waitForStart();



        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("setX", poseEstimate.getX());
        telemetry.addData("setY", poseEstimate.getY());
        telemetry.addData("setHeading", poseEstimate.getHeading());
        telemetry.update();

        drive.followTrajectory(traj1_1);

        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

    }
}

