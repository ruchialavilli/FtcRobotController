package org.firstinspires.ftc.teamcode.auto22;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DejaVuBot;

/*
    Red1 and Blue 1 strategy.--- 26 points
    Robot will spin the duck table 1 time -10
    Robot will deliver the freight at the top level - 6 points
    Park in warehouse- 10 points
*/
/*  Action Items In Order Of Priority

    Advanced Paths -> RoadRunner
    Strafe -> Gyro sensor
    Vision -> OpenCV
*/
@Disabled
@Autonomous(name="AutoBlue1OpMode", group="AutoOpModes")
public class AutoBlue1OpMode extends BaseAutoOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    String name = "AutoBlue1OpMode";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(name, " Robot ready for run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //Notes: after dropping, ram into wall, go back 3 inches, turn, move to duck spinner

        driveForwardByInches(46, robot, DejaVuBot.TPS);
        turnToPID(90,robot);
        telemetry.addData(name, "Turned to hub  ");
        telemetry.update();
        driveForwardByInches(-14/2, robot, DejaVuBot.TPS);

        //Drop the piece here and reset the arm to initial position
        robot.arm.moveArmToLevel(2);
        sleep(500);
        sleep(1000);
        sleep(500);
        robot.arm.moveArmToLevel(0);
        telemetry.addData(name, " Dropped the freight ");
        telemetry.update();

        //Move the robot to spin the duck
        driveForwardByInches(37, robot, DejaVuBot.TPS);
        turnToPID(-90,robot);
        telemetry.addData(name, " Driving to wall ");
        telemetry.update();

        driveForwardByInches(-34, robot, DejaVuBot.TPS);
        driveForwardByInches(-2, robot, DejaVuBot.TPS/2);

        driveForwardByInches(21, robot, DejaVuBot.TPS);
        turnToPID(90,robot);

        telemetry.addData(name, " Duck spinned ");
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        telemetry.addData(name, "Parked in warehouse");
        telemetry.update();
    }
}