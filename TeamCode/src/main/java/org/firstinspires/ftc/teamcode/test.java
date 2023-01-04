package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="test", group="AutoOpModes")
public class test extends BaseAutoOpMode {

    public void runOpMode() throws InterruptedException {
            //the auton file to be made for Red1 and Blue2
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            //robot.arm = new DejaVuArm();
            //robot.arm.init(hardwareMap, true);


        Pose2d startPose = new Pose2d(-66, 36, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-9, 36))
                .build();
            waitForStart();
            if(isStopRequested()) return;

            while (opModeIsActive()) {
                drive.followTrajectory(traj1);
                //strafe to nearest pole
                //drive.followTrajectory(traj2);
                //aligning

                }
            }
        }

