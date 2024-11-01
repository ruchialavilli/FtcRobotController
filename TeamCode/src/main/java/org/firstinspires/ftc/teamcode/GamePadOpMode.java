package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GamePadOpMode", group="Linear OpMode")
public class GamePadOpMode extends LinearOpMode {
    private static final String TAG = "GamePadOpMode";
    DejaVuBot robot = new DejaVuBot();
    private ElapsedTime runtime = new ElapsedTime();
    //look below at isBlue variable every gamepad run
    private boolean isBlue = true;
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    private Thread gamepad1Thread;
    private Thread gamepad2Thread;
    private Thread pickUpThread;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,false);
        robot.chassisEncoderOff();
        robot.arm.moveArmToLevel(0);
        robot.arm.setTelemetry(telemetry);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready for gamepad run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        Log.d(TAG, "starting thread 1");
        gamepad1Thread = new Thread(gp1Runnable);
        gamepad1Thread.start();

        Log.d(TAG, "starting thread 2");
        gamepad2Thread = new Thread(gp2Runnable);
        gamepad2Thread.start();

        Log.d(TAG, "starting thread 3");
        pickUpThread = new Thread(pickupRunnable);
        pickUpThread.start();



        // now wait for the threads to finish before returning from this method
        Log.d(TAG, "waiting for threads to finish...");
        gamepad1Thread.join();
        gamepad2Thread.join();
        pickUpThread.join();
        Log.d(TAG, "thread joins complete");

        // Step 4:  Stop and close the claw.
        robot.stopRobot();
        telemetry.addData("Gamepad", "Threads for 1 & 2 Complete");
        telemetry.update();
    }

    private Runnable gp1Runnable = new Runnable() {
        @Override
        public void run() {
            double leftPower, rightPower;
            double drive;
            double turn;
            double turnPower;

            while (opModeIsActive()) {
                drive = (gamepad1.left_stick_y)*0.75;
                turn = (-gamepad1.left_stick_x)*0.75;
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
                turnPower = Range.clip((-gamepad1.right_stick_x)*0.8, -1.0, 1.0);

//                telemetry.addData("GP1 drive set to:", "" + drive);
//                telemetry.addData("GP1 turn set to:", "" + turn);

                if (gamepad1.left_trigger > 0.5) {
                    leftPower = leftPower *0.4;
                    rightPower = rightPower *0.4;
                    turnPower = turnPower *0.4;
                }

                if (gamepad1.right_stick_x != 0) {
                    robot.turnRobot(turnPower);
                } else {
                    robot.leftFrontMotor.setPower(leftPower);
                    robot.rightFrontMotor.setPower(rightPower);
                    robot.rightBackMotor.setPower(leftPower);
                    robot.leftBackMotor.setPower(rightPower);
                }


                telemetry.addData("GP1 Status", "Completed");
//                telemetry.addData("GP1 Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.addData("GP1 GamePadOpMode", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            } //end of while loop
            Log.d(TAG, "Thread 1 finishing up");
        }
    };

    private Runnable gp2Runnable = new Runnable() {
        public void run() {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    telemetry.addData("GP2 Input", "Y");
                    telemetry.addData("GP2 Input level", "4 - Level 3");
                    robot.arm.moveArmToLevel(4);
                } else if (gamepad2.x) {
                    telemetry.addData("GP2 Input", "X");
                    telemetry.addData("GP2 Input level", "3 - Level 2");
                    robot.arm.moveArmToLevel(3);
                } else if (gamepad2.b) {
                    telemetry.addData("GP2 Input", "B");
                    telemetry.addData("GP2 Input level", "2 - Level 1");
                    robot.arm.moveArmToLevel(2);
                } else if (gamepad2.a) {
                    telemetry.addData("GP2 Input", "A");
                    telemetry.addData("GP2 Input level", "1 - Picking Up");
                    robot.arm.moveArmToLevel(1);
                } else if(gamepad2.right_bumper){
                    telemetry.addData("GP2 Input", "Right Bumper");
                    telemetry.addData("GP2 Input level", "0 - Home");
                    robot.arm.moveArmToLevel(0);
                } else {
                    telemetry.addData("GP2 Input", "Unknown Ignoring");
                }

//                if (gamepad2.dpad_down) {
//                    robot.arm.closePos();
//                }
//
//                if (gamepad2.dpad_up) {
//                    robot.arm.openPos();
//                }

                telemetry.addData("GP2 Status", "Completed");
//                telemetry.addData("GP2 armMotor encoder value", robot.arm.armMotor.getCurrentPosition());
//                telemetry.addData("GP2 servo Position", robot.arm.bucketServo.getPosition());
                telemetry.addData("GP2 GamePadOpMode", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            } //end of while loop
            Log.d(TAG, "Thread 2 finishing up");
        }
    };

    private Runnable pickupRunnable = new Runnable() {
        public void run() {
            while (opModeIsActive()) {
                if (gamepad2.dpad_down) {
                    telemetry.addData("GP2 Status", "Dropping");
                    robot.arm.closePos();
                }

                if (gamepad2.dpad_up) {
                    telemetry.addData("GP2 Status", "Picking up");
                    robot.arm.openPos();
                }
                telemetry.update();
            } //end of while loop
            Log.d(TAG, "Thread 3 finishing up");
        }
    };
}
