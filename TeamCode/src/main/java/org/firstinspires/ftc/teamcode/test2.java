package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="test2", group="AutoOpModes")
public class test2 extends BaseAutoOpMode {

    public void runOpMode() throws InterruptedException {

        //TODO: redo heading pid for accuracy
        //TODO: finish up Red2 and Blue 1 Opmode angle edits, etc FRI MAX
        //TODO: Finish fixing Red1 and Blue2 Opmode angle edits, etc FRI MAx
        //TODO: VISION NEXT FRI

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //red2

            robot.arm = new DejaVuArm();
            robot.arm.init(hardwareMap, true);


        Pose2d startPose = new Pose2d(63.375, 39, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(63.375, 34))
                .lineTo(new Vector2d(0, 34))
                .turn(Math.toRadians(270))
                .lineTo(new Vector2d(0, 30))
                .lineTo(new Vector2d(0, 34))
                .lineTo(new Vector2d(12, 34))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(12, 53))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(0);
                    robot.arm.openPos();
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                })
                .lineTo(new Vector2d(12, 12))
                //.turn(Math.toRadians(-3))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .strafeTo(new Vector2d(0, 12))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(4);
                })
                .lineTo(new Vector2d(0, 18))
                .addDisplacementMarker(() -> {
                    robot.arm.closePos();
                })
                .lineTo(new Vector2d(0, 12))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                })
                .strafeTo(new Vector2d(12, 12))
                .lineTo(new Vector2d(12, 53))
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(0);
                    robot.arm.openPos();
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.arm.moveArmToLevel(1);
                })
                .lineTo(new Vector2d(12, 12))
                //.turn(Math.toRadians(3))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        robot.arm.openPos();
        robot.arm.moveArmToLevel(1);

        drive.followTrajectorySequence(traj);

        for (int i = 0; i < 3; i++) {

            if (isStopRequested()) return;

            drive.followTrajectorySequence(traj2);



        }


    }
}

