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

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.auto22.BaseAutoOpMode;
import org.firstinspires.ftc.teamcode.DejaVuArm;

import java.util.List;

/**
 * This class represents the autonomous run from Red 2  position with webcam integration
 */
@Autonomous(name = "BaseAutoVisionOpMode", group = "AutoOpModes")
@Disabled

public class BaseAutoVisionOpMode extends BaseAutoOpMode {
    private String TAG = "BaseAutoVisionOpMode";
    // model trained on 1/11
    protected static final String TFOD_MODEL_ASSET = "FinalModel.tflite";
    // same as last year
    protected static final String VUFORIA_KEY =
            "AW9JKyj/////AAABmX2UV/5fn04JpsRM9uLXuEYQW29RXmviJEnGvXKmVlhEC3qszm0BbEJjR7kjfCbN3tHX37Pyei+8GICDehSPByjRlHFSf0Vz1NFx3go62FfegYiyB3/vT+7OnT8y2hCNHOlj7RypmGPS10rPpvqJxHJzs1Mz2Tt/HARIeeSiM9eO+nHisES89lFGaiyR1dpjcKLoXteIm6U8vzL/res0hm5tKwuJnWb0Ch8H5u0Vb2k1DnAMAnQwGiPyBn1gSwnQ8yH7Ro9ocO0Z3PCNBTvhh8X7QqICk9Bdg4lHQHxQ0WYTjbIlKUDTHvsQ+6QX7Mn8TzZVOQBo0DsFrrVzQIkrw8TFGRC1qp1RB8JjvgmHmif0";

    protected static final String[] LABELS = {
            "1 Gear",
            "2 PC",
            "3 Tools"
    };

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    // we have 3 locations - 1,2,3. Default is always 2.
    protected int parkingPosition = 2;
    //switch location 1 and 3 for red1

    protected static Vector2d locationToPark;
    /**
     * Initialize the Vuforia localization engine.
     */
    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfodParameters.maxFrameRate = 60;
        tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    protected void findParking() {
        //TODO - is sleep needed ?
        sleep(500);
        int gearCount = 0;
        int pcCount = 0;
        int toolCount = 0;
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
                    if (recognition.getLabel().equals(LABELS[0])) {
                        gearCount++;
                    } else if (recognition.getLabel().equals(LABELS[1])) {
                        pcCount++;
                    } else if (recognition.getLabel().equals(LABELS[2])) {
                        toolCount++;
                    }
                }
                telemetry.addLine(String.format("Found Gear(%d), PC(%d), Tool(%d)",
                        gearCount, pcCount, toolCount));
                Log.d(TAG, String.format("Found Gear(%d), PC(%d), Tool(%d)",
                        gearCount, pcCount, toolCount));
                telemetry.update();
            } else {
                Log.d(TAG, "No more recognitions found!");
            }
        }
        // if anything other than pc is found use it - otherwise pc
        if(gearCount > 0) {
            telemetry.addLine("found gear location2");
            parkingPosition = 2;
        } else if(toolCount > 0){
            telemetry.addLine("found tools location3");
            parkingPosition = 3;
        } else {
            telemetry.addLine("found PC location1");
            parkingPosition = 1;
        }
    }
}
