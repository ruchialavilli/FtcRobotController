package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red2OpMode", group="AutoOpModes")
public class Red2OpMode extends BaseAutoOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    String name = "Red2 Opmode";

    /* TO DO:
    * ensure robot hits pole, etc
    * ensure robot spends enough time waiting (use displacement markers)
    * Runs 3 times consistently*/

    public void runOpMode() throws InterruptedException {
            //the auton file for Red2 and Blue1
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            robot.arm = new DejaVuArm();
            robot.arm.init(hardwareMap, true);



        Pose2d startPose = new Pose2d(-63.375, 39, Math.toRadians(0));
            drive.setPoseEstimate(startPose);

            Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-63.375, 36))
                .build();

            Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                    .lineTo(new Vector2d(0, 36))
                    .build();

            Trajectory traj1_5 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-12, 36))
                .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1_5.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                    .lineTo(new Vector2d(-12, 12))
                    .build();

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(0, 12))
                    .build();

            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .lineTo(new Vector2d(0, 18))
                    .build();

            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineTo(new Vector2d(0, 12))
                    .build();

            Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                    .strafeTo(new Vector2d(-12, 12))
                    .build();

            Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineTo(new Vector2d(-12, 53))
                    .build();

            Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                    .lineTo(new Vector2d(-12, 12))
                    .build();

            telemetry.addData(name, " Robot ready for run");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                if(isStopRequested()) return;

                robot.arm.openPos();
                robot.arm.moveArmToLevel(1);
                drive.followTrajectory(traj0);
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.followTrajectory(traj1);
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //strafe to nearest pole

                telemetry.addData(name, " Angle Difference" + Math.toDegrees(drive.getRawExternalHeading()));
                telemetry.update();

                drive.followTrajectory(traj1_5);
                drive.followTrajectory(traj2);
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //backing up
                for (int i = 0; i < 3; i++) {

                    if(isStopRequested()) return;

                    drive.followTrajectory(traj3);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    robot.arm.moveArmToLevel(4);
                    //aligning
                    drive.followTrajectory(traj4);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    robot.arm.closePos();
                    //aligning and drop
                    drive.followTrajectory(traj5);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    robot.arm.moveArmToLevel(1);
                    //backing up
                    drive.followTrajectory(traj6);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    //strafe away from pole
                    drive.followTrajectory(traj7);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    robot.arm.moveArmToLevel(0);
                    robot.arm.openPos();
                    robot.arm.moveArmToLevel(1);
                    //getting cone
                    drive.followTrajectory(traj8);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    //coming right back
                    }
                }

        }

}
