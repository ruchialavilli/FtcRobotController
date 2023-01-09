package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(name="test3", group="AutoOpModes")
public class test3 extends BaseAutoOpMode {
//old red 2 code-> angles errors
    public void runOpMode() throws InterruptedException {


            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //red2

            robot.arm = new DejaVuArm();
            robot.arm.init(hardwareMap, true);


        Pose2d startPose = new Pose2d(-63.375, 39, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-63.375, 34))
                .lineTo(new Vector2d(0, 34))
                .turn(Math.toRadians(-90))

                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(4);
                    telemetry.addData("Trajectory", " moved to level 4");
                    telemetry.update();
                })

                .lineTo(new Vector2d(0, 30))

                .addDisplacementMarker(() -> {
                    robot.arm.closePos();
                    telemetry.addData("Trajectory", " release cone");
                    telemetry.update();
                })
                .waitSeconds(0.2)

                .lineTo(new Vector2d(0, 34))

                .build();


        TrajectorySequence traj1_1 = drive.trajectorySequenceBuilder(traj1.end())
                .lineTo(new Vector2d(-12, 34))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                    telemetry.addData("Trajectory", " moved to level 1");
                    telemetry.update();
                })
                .build();
        TrajectorySequence traj1_2 = drive.trajectorySequenceBuilder(traj1_1.end())
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(-12, 53))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(0);
                    robot.arm.openPos();
                    telemetry.addData("Trajectory", " moved to level 0 and pick up cone");
                    telemetry.update();
                })
                .waitSeconds(0.2)
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                    telemetry.addData("Trajectory", " moved to level 1");
                    telemetry.update();
                })
                .lineTo(new Vector2d(-12, 12))
                //.turn(Math.toRadians(-3))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1_2.end())
                .strafeTo(new Vector2d(0, 12))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(4);
                })
                .waitSeconds(0.2)
                .lineTo(new Vector2d(0, 18))
                .addDisplacementMarker(() -> {
                    // drop cone
                    robot.arm.closePos();
                })
                .lineTo(new Vector2d(0, 12))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                })
                //.turn(Math.toRadians(3))
                .lineTo(new Vector2d(-12, 12))
                .lineTo(new Vector2d(-12, 53))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(0);
                    // pick up cone
                    robot.arm.openPos();
                })
                .waitSeconds(0.2)
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                })
                .lineTo(new Vector2d(-12, 12))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        robot.arm.openPos();
        robot.arm.moveArmToLevel(1);

        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(traj1_1);
        drive.followTrajectorySequence(traj1_2);

//        for (int i = 0; i < 1; i++) {
//
//            if (isStopRequested()) return;
//
//            drive.followTrajectorySequence(traj2);
//
//
//
//        }


    }
}

