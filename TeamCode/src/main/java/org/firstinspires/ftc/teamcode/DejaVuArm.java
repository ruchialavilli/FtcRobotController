package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Hashtable;

public class DejaVuArm {
    /* Public OpMode members. */
    public DcMotorEx armMotor = null;
    public Servo gripperServo = null;
    static final double PULSES_PER_REVOLUTION = 751.8;
    public static final int TOP_LEVEL=4;
    public static final int MID_LEVEL=3;
    public static final int BOTTOM_LEVEL=2;
    public static final int LOAD_LEVEL=1;
    public static final int GROUND_LEVEL=0;
    private Telemetry telemetry;
    double zeroPosServo;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    //max rpm for our arm motor is 1,850, here we're using 1750 rpm
    public static double SLIDER_TPS = 2500.0; //5959.5 MAX
    public static double SLIDER_TPS_DOWN = 2500.0;
    static HashMap<Integer, Integer> level_map = new HashMap<>();
    private String TAG = "DejaVuArm";
    int heightOffset = 75;

    {
        //100 = 1 inch
        level_map.put(0, 0 );//ground
        level_map.put(1, 400);//5 inches off the ground (pick up)
        level_map.put(2, 1250+heightOffset);//16 inches - level 1
        level_map.put(3, 2100+heightOffset);// to be 26 inches - level 2
        level_map.put(4, 2925+heightOffset);//to be 36 inches - level 3
        level_map.put(6, 485);//1 inches off the ground (auton)
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
        resetDCMotor();
        this.gripperServo = hwMap.get(Servo.class, "gripperServo");
        gripperServo.resetDeviceConfigurationForOpMode();
        double zeroPosServo = gripperServo.getPosition();
        //this.openPos();
        this.moveArmToLevel(0);
        this.currentLevel = 0;
    }

    private void resetDCMotor(){
        Log.d(TAG, "Resetting DC Motor state");
        armMotor.resetDeviceConfigurationForOpMode();
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }


    public void moveArmToLevel(int level) {

        sendToTelemetry("moveArmToLevel:" + level);
        if(level != currentLevel) {
            //GO ONE LEVEL DOWN AT FULL speed\
            int height = level_map.get(level);
            if(level < 4 && (level < currentLevel)) {
                height+=20;
            }
            // set the zero power behavior
//            if(level == 0){
//                armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//            } else {
                armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            }
            //checking if going up
            sendToTelemetry("currentPosition:" + armMotor.getCurrentPosition());
            sendToTelemetry("setting to height:" + height);
            armMotor.setTargetPosition(height);
            //setting the armMotor's target
            sendToTelemetry("starting motor");
            this.armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sendToTelemetry("turning off motor power");
            while (armMotor.isBusy()) {
                if (level > currentLevel) {
                    armMotor.setVelocity(SLIDER_TPS);
                }else{
                    armMotor.setVelocity(SLIDER_TPS_DOWN);
                }
                armMotor.setPower(1);
                Log.d(TAG, "motor going to level (" + level + ") expected height ("
                        + height + ") current height:" + armMotor.getCurrentPosition());
//                if(level ==0 && armMotor.getCurrentPosition() < 10) {
//                    armMotor.setPower(0);
//                    Log.d(TAG, "reached 0 position - turning off power");
//                    try {
//                        sleep(100);
//                    } catch (InterruptedException e) {
//                        e.printStackTrace();
//                    }
//                }
            }
            sendToTelemetry("motor completed to level (" + level + ") current height:" + armMotor.getCurrentPosition());
            Log.d(TAG, "motor completed to level (" + level + ") expected height ("
                    + height + ") current height:" + armMotor.getTargetPosition());
            //motor done/break
            sendToTelemetry("Applying Brakes!");
//            if(level == 0){
//                armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//            } else {
//                armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            }
            sendToTelemetry("turning off motor power");
//            armMotor.setPower(0);


//            //this is to auto-correct if we went beyond the level we need to go - MIGHT NOT NEED IT
//            if (armMotor.getCurrentPosition() != level_map.get(level)) {
//                armMotor.setTargetPosition(level_map.get(level));
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (armMotor.isBusy()) {
//                    armMotor.setVelocity(SLIDER_TPS/4);
//                }
//                armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//            }
            currentLevel = level;
            if(level == 0){
                resetDCMotor();
            }
        } else {
            sendToTelemetry("already at level:" + level);
        }
    }

    // 0 - 300 degrees is 0 - 1
    private static final double moveByPosition = 0.0025;
    double gripPosition = MIN_POSITION;        // set grip to full open.
    // drop the object
    public void closePos() {
        gripperServo.setDirection(Servo.Direction.FORWARD);
        gripperServo.setPosition(0.5);
        sendToTelemetry("Sending to Pos Servo:" + gripperServo.getPosition());
    }

    // pick up object
    public void openPos() {
        gripperServo.setDirection(Servo.Direction.FORWARD);
        gripperServo.setPosition(0.27);
        sendToTelemetry("Sending to Pos Servo:" + gripperServo.getPosition());
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

