package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red1OpMode", group="AutoOpModes")
public class Red1OpMode extends BaseAutoOpMode {

    public void runOpMode() throws InterruptedException {
            //the auton file to be made for Red1 and Blue2
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            robot.arm = new DejaVuArm();
            robot.arm.init(hardwareMap, true);

            Pose2d startPose = new Pose2d(66, 36, Math.toRadians(270));
            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .strafeTo(new Vector2d(-1, 36))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .lineTo(new Vector2d(-1, 30))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .lineTo(new Vector2d(-1, 36))
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .strafeTo(new Vector2d(9, 36))
                    .build();

            Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(183))))
                    .lineTo(new Vector2d(9, 53))
                    .build();

            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .lineTo(new Vector2d(9, 15))
                    .build();

            Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0, 0, Math.toRadians(5))))
                    .strafeTo(new Vector2d(-1, 15))
                    .build();

            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                    .lineTo(new Vector2d(-1, 18))
                    .build();

            Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                    .lineTo(new Vector2d(-1, 15))
                    .build();

            Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                    .strafeTo(new Vector2d(9, 15))
                    .build();

            Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                    .lineTo(new Vector2d(9, 53))
                    .build();

            Trajectory traj12 = drive.trajectoryBuilder(traj11.end().plus(new Pose2d(0, 0, Math.toRadians(-3))))
                    .lineTo(new Vector2d(9, 15))
                    .build();

            waitForStart();
            if(isStopRequested()) return;

            while (opModeIsActive()) {
                robot.arm.openPos();
                robot.arm.moveArmToLevel(1);
                drive.followTrajectory(traj1);
                robot.arm.moveArmToLevel(4);
                //strafe to nearest pole
                drive.followTrajectory(traj2);
                robot.arm.closePos();
                //aligning
                drive.followTrajectory(traj3);
                robot.arm.moveArmToLevel(1);
                //dropping cone and moving back
                drive.followTrajectory(traj4);
                drive.turn(Math.toRadians(180));
                //backing up
                drive.followTrajectory(traj5);
                robot.arm.moveArmToLevel(0);
                robot.arm.openPos();
                robot.arm.moveArmToLevel(1);
                //turning to new cone
                drive.followTrajectory(traj6);
                //moving to drop next cone
                int counter = 0;
                while (opModeIsActive() && counter < 3){
                    drive.followTrajectory(traj7);
                    robot.arm.moveArmToLevel(4);
                    //aligning
                    drive.followTrajectory(traj8);
                    robot.arm.closePos();
                    //align and drop
                    drive.followTrajectory(traj9);
                    robot.arm.moveArmToLevel(1);
                    //backing up
                    drive.followTrajectory(traj10);
                    //still backing up
                    drive.followTrajectory(traj11);
                    //turning to new cone
                    drive.turn(Math.toRadians(-3));
                    robot.arm.moveArmToLevel(0);
                    robot.arm.openPos();
                    robot.arm.moveArmToLevel(1);
                    drive.followTrajectory(traj12);
                    //going back
                    counter++;
                }
            }
        }

}
