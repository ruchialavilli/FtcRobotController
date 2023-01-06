package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="LineTest", group="AutoOpModes")
public class LineTest extends BaseAutoOpMode {

    public void runOpMode() throws InterruptedException {


            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            //robot.arm = new DejaVuArm();
            //robot.arm.init(hardwareMap, true);


        Pose2d startPose = new Pose2d(-63.375, 39, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0, 39))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(traj);

           //strafe to nearest pole
                //drive.followTrajectory(traj2);
                //aligning


    }
}

