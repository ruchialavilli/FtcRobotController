package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Hashtable;

public class DejaVuArm {
    /* Public OpMode members. */
    public DcMotorEx armMotor = null;
    static final double PULSES_PER_REVOLUTION = 751.8;
    public static final int TOP_LEVEL=3;
    public static final int MID_LEVEL=2;
    public static final int BOTTOM_LEVEL=1;
    public static final int GROUND_LEVEL=0;
    private Telemetry telemetry;

    //max rpm for our arm motor is 1,850, here we're using 1750 rpm
    public static double SLIDER_TPS = 5000.0; //5959.5 MAX
    static HashMap<Integer, Integer> level_map = new HashMap<>();
    {
        level_map.put(0, 0);//ground
        level_map.put(1, 350);//16 inches
        level_map.put(2, 1350);//26 inches
        level_map.put(3, 2350);//36 inches
        //100 = 1 inch

    }

    private int currentLevel = 0;
    public int armMotorBasePos;
    private boolean isAuton;
    private HardwareMap hwMap = null;

    public DejaVuArm() {    }

    //Initialize the arm
    public void init(HardwareMap hMap, boolean isAuton) {
        this.isAuton = isAuton;
        this.hwMap = hMap;
        this.armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.moveArmToLevel(0);
        this.currentLevel = 0;
    }


    public void moveArmToLevel(int level) {

        sendToTelemetry("moveArmToLevel:" + level);
        if(level != currentLevel) {
            //GO ONE LEVEL DOWN AT FULL speed\
            int height = level_map.get(level);
            //checking if going up
            sendToTelemetry("currentPosition:" + armMotor.getCurrentPosition());
            sendToTelemetry("setting to height:" + height);
            armMotor.setTargetPosition(height);
            //setting the armMotor's target
            sendToTelemetry("starting motor");
            this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            while (armMotor.isBusy()) {
                armMotor.setVelocity(SLIDER_TPS);
                armMotor.setPower(1);
                sendToTelemetry("busy busy");
            }
            sendToTelemetry("motor completed");
            //motor done/break
            sendToTelemetry("Applying Brakes!");
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            sendToTelemetry("turning off motor power");
//            armMotor.setPower(0);


//          // this is to auto-correct if we went beyond the level we need to go - MIGHT NOT NEED IT
//            if (armMotor.getCurrentPosition() != level_map.get(level)) {
//                armMotor.setTargetPosition(level_map.get(level));
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (armMotor.isBusy()) {
//                    armMotor.setVelocity(SLIDER_TPS/4);
//                }
//                armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//                armMotor.setPower(0);
//            }
            currentLevel = level;
        } else {
            sendToTelemetry("already at level:" + level);
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    private void sendToTelemetry(String msg){
        if(telemetry != null){
            telemetry.addData("DejaVuArm", msg);
            telemetry.update();
//            try {
//                sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
        }
    }
}

