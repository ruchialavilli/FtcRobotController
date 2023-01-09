package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red1OpMode", group="AutoOpModes")
public class Red1OpMode extends BaseAutoOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    String name = "Red1 Opmode";


    public void runOpMode() throws InterruptedException {

            /* TODO: ensure that robot hits wall to pick up cone in full field and get vision */

            //the auton file to be used for Red1 and Blue2
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



            robot.arm = new DejaVuArm();
            robot.arm.init(hardwareMap, true);

            Pose2d startPose = new Pose2d(63.375, 32, Math.toRadians(180));
            drive.setPoseEstimate(startPose);

            Trajectory traj0 = drive.trajectoryBuilder(startPose)
                    .strafeTo(new Vector2d(-8, 32))
                    .build();

            Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                    .lineTo(new Vector2d(14, 32))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(43))))
                    .forward(11)
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .back(11)
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-133))))
                    .strafeTo(new Vector2d(14, 59))
                    .build();

            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineTo(new Vector2d(14, 32))
                    .build();

            //looping code here

            Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0, 0, Math.toRadians(133))))
                    .forward(11)
                    .build();

            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                    .back(11)
                    .build();

            Trajectory traj8 = drive.trajectoryBuilder(traj7.end().plus(new Pose2d(0, 0, Math.toRadians(-133))))
                    .strafeTo(new Vector2d(14, 59))
                    .build();

            Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                    .lineTo(new Vector2d(14, 32))
                    .build();



            robot.arm.closePos();
            telemetry.addData(name, " Robot ready for run");
            telemetry.update();

            waitForStart();

            if(isStopRequested()) return;

            robot.arm.openPos();
            sleep(500);
            robot.arm.moveArmToLevel(2);
            sleep(500);
            drive.followTrajectory(traj0);
            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(43));
            robot.arm.moveArmToLevel(4);
            telemetry.addData("Trajectory", " moved to level 4");
            telemetry.update();
            sleep(500);
            drive.followTrajectory(traj2);
            robot.arm.closePos();
            telemetry.addData("Trajectory", " release cone");
            telemetry.update();
            drive.followTrajectory(traj3);
            robot.arm.moveArmToLevel(2);
            telemetry.addData("Trajectory", " moved to level 2");
            telemetry.update();
            sleep(500);
            drive.turn(Math.toRadians(-133));
            drive.followTrajectory(traj4);
            robot.arm.moveArmToLevel(6);
            sleep(500);
            robot.arm.openPos();
            telemetry.addData("Trajectory", " moved to level 0 and pick up cone");
            telemetry.update();
            sleep(500);
            robot.arm.moveArmToLevel(2);
            telemetry.addData("Trajectory", " moved to level 2");
            telemetry.update();
            drive.followTrajectory(traj5);

//loop from here if necessary
            drive.turn(Math.toRadians(133));
            robot.arm.moveArmToLevel(4);
            telemetry.addData("Trajectory", " moved to level 4");
            telemetry.update();
            sleep(500);
            drive.followTrajectory(traj6);
            robot.arm.closePos();
            telemetry.addData("Trajectory", " release cone");
            telemetry.update();
            drive.followTrajectory(traj7);
            robot.arm.moveArmToLevel(2);
            telemetry.addData("Trajectory", " moved to level 2");
            telemetry.update();
            sleep(500);
            drive.turn(Math.toRadians(-133));
            drive.followTrajectory(traj8);
            robot.arm.moveArmToLevel(6);
            sleep(500);
            robot.arm.openPos();
            telemetry.addData("Trajectory", " moved to level 2.5 and pick up cone");
            telemetry.update();
            sleep(500);
            robot.arm.moveArmToLevel(2);
            telemetry.addData("Trajectory", " moved to level 2");
            telemetry.update();
            drive.followTrajectory(traj9);


       }

}
