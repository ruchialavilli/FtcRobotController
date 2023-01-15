package org.firstinspires.ftc.teamcode;

import android.util.Base64;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "Red2OpMode", group = "AutoOpModes")
public class Red2OpMode extends BaseAutoVisionOpMode {
    private String TAG = "Red2OpMode";
    private ElapsedTime runtime = new ElapsedTime();
    private Thread parkingLocationFinderThread;
    String name = "Red2Opmode";

    // parking location 1
    // pc
    protected static Vector2d location1 = new Vector2d(-14, 55);
    // gear
    protected static Vector2d location2 = new Vector2d(-14, 32.25);
    // tool
    protected static Vector2d location3 = new Vector2d(-14, 8);

    public void runOpMode() throws InterruptedException {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.0, 16.0/9.0);
        }

        //the auton file used for Red2 and Blue1
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.arm = new DejaVuArm();
        robot.arm.init(hardwareMap, true);

//an inch too close to 0
        Pose2d startPose = new Pose2d(-63.375, 32, Math.toRadians(0));//todo: measure on competition field
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(8, 32))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(-14, 32))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(-47))))
                .forward(12)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(12)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(137))))
                .lineTo(new Vector2d(-14, 59))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(-14, 32))
                .build();

        //looping code trajectories...

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0, 0, Math.toRadians(-137))))
                .forward(12)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(12)
                .build();


//        Trajectory traj8 = drive.trajectoryBuilder(traj7.end().plus(new Pose2d(0, 0, Math.toRadians(135))))
//                .lineTo(new Vector2d(-14, 59))
//                .build();




        //end of loop

        robot.arm.closePos();
        telemetry.addData(name, " Robot ready for run");
        telemetry.update();

        waitForStart();
        runtime.reset();

        Log.d(TAG, "starting thread 1");
        parkingLocationFinderThread = new Thread(parkingLocationFinderRunnable);
        parkingLocationFinderThread.start();

        // now wait for the threads to finish before returning from this method
        Log.d(TAG, "waiting for threads to finish...");
        parkingLocationFinderThread.join();
        Log.d(TAG, "thread joins complete");


        // always deactivate
        if (tfod != null) {
            tfod.deactivate();
        }

        if (isStopRequested()) return;

        robot.arm.openPos();
        sleep(500);
        robot.arm.moveArmToLevel(2);
        sleep(500);
        if (isStopRequested()) return;
        drive.followTrajectory(traj0);
        if (isStopRequested()) return;
        drive.followTrajectory(traj1);
        if (isStopRequested()) return;
        drive.turn(Math.toRadians(-47));
        robot.arm.moveArmToLevel(4);
        telemetry.addData("Trajectory", " moved to level 4");
        telemetry.update();
        if (isStopRequested()) return;
        sleep(500);
        drive.followTrajectory(traj2);
        robot.arm.closePos();
        telemetry.addData("Trajectory", " release cone");
        telemetry.update();
        if (isStopRequested()) return;
        drive.followTrajectory(traj3);
        robot.arm.moveArmToLevel(2);
        telemetry.addData("Trajectory", " moved to level 2");
        telemetry.update();
        if (isStopRequested()) return;
        sleep(500);
        drive.turn(Math.toRadians(137));
        drive.followTrajectory(traj4);
        robot.arm.moveArmToLevel(6);
        if (isStopRequested()) return;
        sleep(500);
        robot.arm.openPos();
        telemetry.addData("Trajectory", " moved to level 2.5 and pick up cone");
        telemetry.update();
        sleep(500);
        robot.arm.moveArmToLevel(2);
        telemetry.addData("Trajectory", " moved to level 2");
        telemetry.update();
        drive.followTrajectory(traj5);
        if (isStopRequested()) return;
        //loop from here if necessary
        drive.turn(Math.toRadians(-137));
        robot.arm.moveArmToLevel(4);
        telemetry.addData("Trajectory", " moved to level 4");
        telemetry.update();
        if (isStopRequested()) return;
        sleep(500);
        drive.followTrajectory(traj6);
        robot.arm.closePos();
        telemetry.addData("Trajectory", " release cone");
        telemetry.update();
        if (isStopRequested()) return;
        drive.followTrajectory(traj7);
        robot.arm.moveArmToLevel(2);
        telemetry.addData("Trajectory", " moved to level 2");
        telemetry.update();
        sleep(500);

        drive.turn(Math.toRadians(-43));
        telemetry.addData("Trajectory", " moved to level 2");
        telemetry.update();
        if (isStopRequested()) return;
        //going to found location
        telemetry.addData("Going to parking location:", locationToPark.toString());
            telemetry.update();
            drive.followTrajectory(
                    drive.trajectoryBuilder(traj7.end().plus(new Pose2d(0, 0, Math.toRadians(-42.5))))
                    .lineTo(BaseAutoVisionOpMode.locationToPark)
                    .build());
        robot.arm.moveArmToLevel(0);

    }

    private Runnable parkingLocationFinderRunnable = () -> {
        //driveForwardByInches(4, robot, DejaVuBot.TPS);
        //Find the level in 10 attempts. If not detected set level to 3.
        if (opModeIsActive() && tfod != null) {
            telemetry.addData(">", "Detecting parking location using vision");
            telemetry.update();
            findParking();
            switch(parkingPosition){
                case 1:
                    locationToPark = location1;
                    break;
                case 2:
                    locationToPark = location2;
                    break;
                case 3:
                    locationToPark = location3;
                    break;
                default:
                    locationToPark = location2;
                    break;
            }
            telemetry.addData("Found parking", locationToPark);
            if(locationToPark != null){
                Log.i(TAG, " Found parking ="+ locationToPark);
            }
            if(locationToPark == location1) {
                telemetry.addLine(" location1");
            } else if(locationToPark == location2) {
                telemetry.addLine(" location2");
            } else if(locationToPark == location3) {
                telemetry.addLine(" location3");
            } else {
                telemetry.addLine(" UNKNOWN - defaulting to location2");
                locationToPark = location2;
            }
            telemetry.update();
        } else{
            telemetry.addData(">", "Could not init vision code - defaulting to TOP");
            telemetry.update();
            locationToPark = location2;
        }
        Log.d(TAG, "Thread 1 finishing up");
    };

}


