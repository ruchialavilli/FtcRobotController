package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="TestOpmode", group="AutoOpModes")
public class TestOpmode extends LinearOpMode {
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            DejaVuBot robot = new DejaVuBot();

            Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(10)
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .splineTo(new Vector2d(5, 6), 0)
                    .splineTo(new Vector2d(9, -10), 0)
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(5, 6), 0)
                    .splineTo(new Vector2d(9, -10), 0)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);

        }

}
