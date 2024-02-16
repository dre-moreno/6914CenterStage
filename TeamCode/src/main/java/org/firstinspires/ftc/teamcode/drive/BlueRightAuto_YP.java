package org.firstinspires.ftc.teamcode.drive;

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

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Comp: Blue Right - P&Y", group = "Blue Auto - YP")
public class BlueRightAuto_YP extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "blueMarkerV2.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/blueMarkerV2.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "BlueCenter",
            "BlueLeft",
            "BlueRight"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        Hardware drive = new Hardware(hardwareMap);
        drive.claw.setPosition(1);
        drive.backPixel.setPosition(0);
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

//                drive.spoolAngleRight.setPosition(.23);
                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        Hardware drive = new Hardware(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-34,70,3*Math.PI/2));

        //done needs testing - on field
        Trajectory purpleLeft = drive.trajectoryBuilder(new Pose2d(-34,70,3*Math.PI/2))
                .splineToLinearHeading(new Pose2d(-37,43,0),0)
                .build();

        //done needs testing - on field
        Trajectory purpleCenter = drive.trajectoryBuilder(new Pose2d(-34,70,3*Math.PI/2))
                .lineTo(new Vector2d(-38,45))
                .build();

        //done needs testing - on field
        Trajectory purpleRight = drive.trajectoryBuilder(new Pose2d(-34,70,3*Math.PI/2))
                .lineTo(new Vector2d(-44,55))
                .build();

        //all Blue***Back  done needs testing
        Trajectory blueLeftBack = drive.trajectoryBuilder(purpleLeft.end())
                .lineToLinearHeading(new Pose2d(-55,60,3*Math.PI/2))
                .build();

        Trajectory blueCenterBack = drive.trajectoryBuilder(purpleCenter.end())
                .lineTo(new Vector2d(-55,60))
                .build();

        Trajectory blueRightBack = drive.trajectoryBuilder(purpleRight.end())
                .lineTo(new Vector2d(-55,60))
                .build();


        //done needs testing
        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(new Pose2d(-55,60,3*Math.PI/2))
                .lineToLinearHeading(new Pose2d(-55,19,3*Math.PI/2))
                .lineTo(new Vector2d(-49,19))
                .turn(-Math.PI/2)
                .back(95)
                .build();

        Trajectory backdropLeft = drive.trajectoryBuilder(toBackdrop.end())
                .lineToLinearHeading(new Pose2d(52,48,Math.PI))
                .build();

        Trajectory backdropCenter = drive.trajectoryBuilder(toBackdrop.end())
                .lineToLinearHeading(new Pose2d(52,42,Math.PI))
                .build();


        Trajectory backdropRight = drive.trajectoryBuilder(toBackdrop.end())
                .lineToLinearHeading(new Pose2d(52,34,Math.PI))
                .build();


        TrajectorySequence toParkCenter = drive.trajectorySequenceBuilder(backdropCenter.end())
                .lineTo(new Vector2d(46,18))
                .build();
        TrajectorySequence toParkLeft = drive.trajectorySequenceBuilder(backdropLeft.end())
                .lineTo(new Vector2d(46,18))
                .build();
        TrajectorySequence toParkRight = drive.trajectorySequenceBuilder(backdropRight.end())
                .lineTo(new Vector2d(46,18))
                .build();

        Trajectory park = drive.trajectoryBuilder(toParkCenter.end())
                .back(15)
                .build();




        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());


            if(recognition.getLabel().equals("BlueLeft")){
                drive.spoolAngleRight.setPosition(.325);
                drive.followTrajectory(purpleLeft);
                sleep(50);
                drive.claw.setPosition(.7);
                sleep(50);
                drive.spoolAngleRight.setPosition(-.175);
                drive.followTrajectory(blueLeftBack);
                sleep(50);
                drive.followTrajectorySequence(toBackdrop);
                sleep(50);
                drive.followTrajectory(backdropLeft);
                drive.backPixel.setPosition(.5);
                sleep(2000);
                drive.backPixel.setPosition(0);
                drive.followTrajectorySequence(toParkLeft);
                drive.followTrajectory(park);
                sleep(100000000);

            } else if (recognition.getLabel().equals("BlueCenter")){
                drive.spoolAngleRight.setPosition(.325);
                drive.followTrajectory(purpleCenter);
                sleep(50);
                drive.claw.setPosition(.7);
                sleep(50);
                drive.spoolAngleRight.setPosition(-.175);
                drive.followTrajectory(blueCenterBack);
                sleep(50);
                drive.followTrajectorySequence(toBackdrop);
                sleep(50);
                drive.followTrajectory(backdropCenter);
                drive.backPixel.setPosition(.5);
                sleep(2000);
                drive.backPixel.setPosition(0);
                drive.followTrajectorySequence(toParkCenter);
                drive.followTrajectory(park);
                sleep(100000000);

            } else if (recognition.getLabel().equals("BlueRight")){
                drive.spoolAngleRight.setPosition(.325);
                drive.followTrajectory(purpleRight);
                sleep(50);
                drive.claw.setPosition(.7);
                sleep(50);
                drive.spoolAngleRight.setPosition(-.175);
                drive.followTrajectory(blueRightBack);
                sleep(50);
                drive.followTrajectorySequence(toBackdrop);
                sleep(50);
                drive.followTrajectory(backdropRight);
                drive.backPixel.setPosition(.5);
                sleep(2000);
                drive.backPixel.setPosition(0);
                drive.followTrajectorySequence(toParkRight);
                drive.followTrajectory(park);
                sleep(100000000);

            }


        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class