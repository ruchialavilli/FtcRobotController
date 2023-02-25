package org.firstinspires.ftc.teamcode;

import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Blue2OpMode", group = "AutoOpModes")
public class Blue2OpMode extends BaseAutoVisionOpMode {
    private String TAG = "Blue2OpMode";
    private ElapsedTime runtime = new ElapsedTime();
    String name = "Blue2OpMode";
    private Thread parkingLocationFinderThread;
    public static String ACTION_GOTO_LEVEL = "goto_level";
    public static String ACTION_PICKUP_CONE = "pickup";
    public static String ACTION_RELEASE_CONE = "release";
    private HandlerThread mHandlerThread;
    private Handler armHandler;
    // parking locations
    // stop sign
    protected static Vector2d location1 = new Vector2d(10, 8);
    // traffic lights
    protected static Vector2d location2 = new Vector2d(10, 32.25);
    // teddy bear
    protected static Vector2d location3 = new Vector2d(10, 58);

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
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        //the auton file to be used for Red1 and Blue2
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        robot.arm = new DejaVuArm();
        robot.arm.init(hardwareMap, true);
        // initialize handler thread to move robot arm
        mHandlerThread = new HandlerThread("armThread");
        mHandlerThread.start();
        armHandler = new Handler(mHandlerThread.getLooper()) {
            @Override
            public void handleMessage(@NonNull Message msg) {
                super.handleMessage(msg);
//                Log.d(TAG, "executing arm action: " + action);
                switch (msg.what){
                    case 999:
//                        Log.d(TAG, "executing arm action: Open");
                        robot.arm.openPos();
//                        Log.d(TAG, "executing arm action: Open Done");
                        break;
                    case 888:
//                        Log.d(TAG, "executing arm action: Close");
                        robot.arm.closePos();
//                        Log.d(TAG, "executing arm action: Close Done");
                        break;
                    default:
//                        Log.d(TAG, "executing arm action going to level: " + msg.what);
                        robot.arm.moveArmToLevelAuton(msg.what);
//                        Log.d(TAG, "executing arm action: going to level Done");
                        break;
                }

            }
        };


        Pose2d startPose = new Pose2d(63.375, 32, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory traj0 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(10, 32))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj0.end().plus(new Pose2d(0, 0, Math.toRadians(41))))
                .forward(12)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(12)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-131))))
                .strafeTo(new Vector2d(10, 60))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(10, 32))
                .build();

        //looping code here

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0, 0, Math.toRadians(128))))
                .forward(12)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(12)
                .build();

        sendMessage(ACTION_PICKUP_CONE);
        sendToTelemetry(name, " Robot ready for run");

        waitForStart();
        runtime.reset();

//        Log.d(TAG, "starting thread 1");
        parkingLocationFinderThread = new Thread(parkingLocationFinderRunnable);
        parkingLocationFinderThread.start();

        // now wait for the threads to finish before returning from this method
//        Log.d(TAG, "waiting for threads to finish...");
        parkingLocationFinderThread.join();
//        Log.d(TAG, "thread joins complete");
        // always deactivate
        if (tfod != null) {
            tfod.deactivate();
        }

        if (isStopRequested()) {
            //armHandler.
            mHandlerThread.quitSafely();
            return;
        }

        sendMessage(ACTION_GOTO_LEVEL, 4);
        drive.followTrajectory(traj0);
        drive.turn(Math.toRadians(41));
        sendToTelemetry("Trajectory", " moved to level 4");

        drive.followTrajectory(traj2);
        sendMessage(ACTION_RELEASE_CONE);
        sendToTelemetry("Trajectory", " release cone");
        drive.followTrajectory(traj3);

        //loop from here
        sendMessage(ACTION_GOTO_LEVEL, 5);
        sendToTelemetry("Trajectory", " moved to level 2");

        drive.turn(Math.toRadians(-131));
        drive.followTrajectory(traj4);
        sendMessage(ACTION_GOTO_LEVEL, 6);
        sleep(250);
        sendMessage(ACTION_PICKUP_CONE);
        sendToTelemetry("Trajectory", " moved to level 0 and pick up cone");

        sleep(350);
        sendMessage(ACTION_GOTO_LEVEL, 4);
        sendToTelemetry("Trajectory", " moved to level 4");

        drive.followTrajectory(traj5);
        drive.turn(Math.toRadians(128));
        drive.followTrajectory(traj6);
        sendMessage(ACTION_RELEASE_CONE);
        sendToTelemetry("Trajectory", " release cone");

        drive.followTrajectory(traj7);
        sendMessage(ACTION_GOTO_LEVEL, 8);
        sendToTelemetry("Trajectory", " moved to level 2");

        drive.turn(Math.toRadians(-131));
        drive.followTrajectory(traj4);
        sendMessage(ACTION_GOTO_LEVEL, 7);
        sleep(250);
        sendMessage(ACTION_PICKUP_CONE);
        sendToTelemetry("Trajectory", " moved to level 0 and pick up cone");

        sleep(350);
        sendMessage(ACTION_GOTO_LEVEL, 4);
        sendToTelemetry("Trajectory", " moved to level 4");

        drive.followTrajectory(traj5);
        drive.turn(Math.toRadians(128));
        drive.followTrajectory(traj6);
        sendMessage(ACTION_RELEASE_CONE);
        sendToTelemetry("Trajectory", " release cone");
        drive.followTrajectory(traj7);



        //to end
        //drive.turn(Math.toRadians(52));
        //going to found location
        sendToTelemetry("Going to parking location:", locationToPark.toString());

        drive.followTrajectory(
                drive.trajectoryBuilder(traj7.end().plus(new Pose2d(0, 0, Math.toRadians(53))))
                        .lineTo(BaseAutoVisionOpMode.locationToPark)
                        .build());

        //quitting thread
        mHandlerThread.quitSafely();
    }

    private void sendMessage(String action) {
        sendMessage(action, -1);
    }
    private void sendMessage(String action, int level) {
        if(ACTION_RELEASE_CONE.equals(action)){
            armHandler.sendEmptyMessage(888);
        } else if(ACTION_PICKUP_CONE.equals(action)){
            armHandler.sendEmptyMessage(999);
        } else {
            armHandler.sendEmptyMessage(level);
        }

    }

    private Runnable parkingLocationFinderRunnable = () -> {
        //driveForwardByInches(4, robot, DejaVuBot.TPS);
        //Find the level in 10 attempts. If not detected set level to 3.
        if (opModeIsActive() && tfod != null) {
            sendToTelemetry(">", "Detecting parking location using vision");

            findParking();
            switch (parkingPosition) {
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
            sendToTelemetry("Found parking", locationToPark.toString());
            if (locationToPark != null) {
                Log.i(TAG, " Found parking =" + locationToPark);
            }
            if (locationToPark == location1) {
                telemetry.addLine(" location1");
            } else if (locationToPark == location2) {
                telemetry.addLine(" location2");
            } else if (locationToPark == location3) {
                telemetry.addLine(" location3");
            } else {
                telemetry.addLine(" UNKNOWN - defaulting to location2");
                locationToPark = location2;
            }
        } else {
            sendToTelemetry(">", "Could not init vision code - defaulting to TOP");
            locationToPark = location2;
        }
//        Log.d(TAG, "Thread 1 finishing up");
    };


}
