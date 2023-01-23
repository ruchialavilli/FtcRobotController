/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Custom TensorFlow Object Detection Webcam", group = "Concept")
public class CustomTensorFlowObjectDetectionWebcam extends LinearOpMode {

    private static final String TAG = "TensorFlow";

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    //private static final String TFOD_MODEL_ASSET  = "CustomTeamModel5.tflite"; //Training 2023 Color 3
    //private static final String TFOD_MODEL_ASSET  = "CustomTeamModel6.tflite"; //Training 2023 Color 4
    //private static final String TFOD_MODEL_ASSET  = "FinalModel.tflite"; //CompTraining2
    private static final String TFOD_MODEL_ASSET  = "ssd_mobilenet_v1_1_metadata_1.tflite";

    //    private static final String[] LABELS = {
//            "0 Gear",
//            "1 PC",
//            "2 Tools"
//    };
    private static final String[] LABELS = {
            "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","???","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","???","backpack","umbrella","???","???","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","???","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","???","dining table","???","???","toilet","???","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","???","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
    };


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AW9JKyj/////AAABmX2UV/5fn04JpsRM9uLXuEYQW29RXmviJEnGvXKmVlhEC3qszm0BbEJjR7kjfCbN3tHX37Pyei+8GICDehSPByjRlHFSf0Vz1NFx3go62FfegYiyB3/vT+7OnT8y2hCNHOlj7RypmGPS10rPpvqJxHJzs1Mz2Tt/HARIeeSiM9eO+nHisES89lFGaiyR1dpjcKLoXteIm6U8vzL/res0hm5tKwuJnWb0Ch8H5u0Vb2k1DnAMAnQwGiPyBn1gSwnQ8yH7Ro9ocO0Z3PCNBTvhh8X7QqICk9Bdg4lHQHxQ0WYTjbIlKUDTHvsQ+6QX7Mn8TzZVOQBo0DsFrrVzQIkrw8TFGRC1qp1RB8JjvgmHmif0";



    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
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

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5 , 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        telemetry.update();
                        findParking();
//                        // step through the list of recognitions and display image position/size information for each one
//                        // Note: "Image number" refers to the randomized image orientation/number
//                        for (Recognition recognition : updatedRecognitions) {
//                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
//                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
//
//                            telemetry.addData(""," ");
//                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
//                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
//                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
//                            telemetry.update();
//                        }
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.50f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 360;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    protected void findParking() {
        //TODO - is sleep needed ?
        sleep(500);
        int teddyCount = 0;
        int trafficCount = 0;
        int stopCount = 0;
        for(int i = 0; i < 3; i++) {
            List<Recognition> updatedRecognitions = tfod.getRecognitions();//tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() > 0) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    Log.d(TAG, "////************* " + i + ":");
                    Log.d(TAG, String.format("Image %s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100));
//                Log.d(TAG, String.format("- Position (Row/Col): %.0f / %.0f", row, col));
//                Log.d(TAG, String.format("- Size (Width/Height): %.0f / %.0f", width, height));
                    Log.d(TAG, "*************////");
                    if (recognition.getLabel().equals("teddy bear")) {
                        teddyCount++;
                    } else if (recognition.getLabel().equals("traffic light")) {
                        trafficCount++;
                    } else if (recognition.getLabel().equals("stop sign")) {
                        stopCount++;
                    }
                }
                telemetry.addLine(String.format("Found Teddy(%d), Traffic(%d), Stop(%d)",teddyCount
                        , trafficCount, stopCount));
                Log.d(TAG, String.format("Found Teddy(%d), Traffic(%d), Stop(%d)", teddyCount
                        , trafficCount, stopCount));
                telemetry.update();
            } else {
                Log.d(TAG, "No more recognitions found!");
            }
        }
        // if anything other than pc is found use it - otherwise pc
        if( teddyCount > 0) {
            telemetry.addLine("found teddy bear location 3");
        } else if(stopCount > 0){
            telemetry.addLine("found stop sign - location 1");
        } else {
            telemetry.addLine("found traffic lights - location 2");
        }
        telemetry.update();
    }
}
