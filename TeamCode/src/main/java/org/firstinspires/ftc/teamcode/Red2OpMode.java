package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red2OpMode", group="AutoOpModes")
public class Red2OpMode extends BaseAutoOpMode {

    public void runOpMode() throws InterruptedException {
            //the auton file for Red2 and Blue1
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            robot.arm = new DejaVuArm();
            robot.arm.init(hardwareMap, true);

            Pose2d startPose = new Pose2d(-66, 36, Math.toRadians(90));
            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .strafeTo(new Vector2d(-9, 36))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineTo(new Vector2d(-9, 15))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(1, 15))
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .lineTo(new Vector2d(1, 18))
                    .build();

            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineTo(new Vector2d(1, 15))
                    .build();

            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .strafeTo(new Vector2d(-9, 15))
                    .build();

            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineTo(new Vector2d(-9, 63))
                    .build();

            Trajectory traj8 = drive.trajectoryBuilder(traj7.end().plus(new Pose2d(0, 0, Math.toRadians(-3))))
                    .lineTo(new Vector2d(-9, 15))
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                if(isStopRequested()) return;

                robot.arm.openPos();
                robot.arm.moveArmToLevel(1);
                drive.followTrajectory(traj1);
                //strafe to nearest pole
                drive.followTrajectory(traj2);
                //backing up
                for (int i = 0; i < 3; i++) {

                    if(isStopRequested()) return;

                    drive.followTrajectory(traj3);
                    robot.arm.moveArmToLevel(4);
                    //aligning
                    drive.followTrajectory(traj4);
                    robot.arm.closePos();
                    //aligning and drop
                    drive.followTrajectory(traj5);
                    robot.arm.moveArmToLevel(1);
                    //backing up
                    drive.followTrajectory(traj6);
                    //strafe away from pole
                    drive.followTrajectory(traj7);
                    robot.arm.moveArmToLevel(0);
                    robot.arm.openPos();
                    robot.arm.moveArmToLevel(1);
                    //getting cone
                    drive.followTrajectory(traj8);
                    //coming right back
                    }
                }

        }

}
