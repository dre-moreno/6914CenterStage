package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "AprilTagTest")
public class AprilTagTest extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException{

        Hardware6914 drive = new Hardware6914(hardwareMap);

        drive.setPoseEstimate(new Pose2d(12,70,3*Math.PI/2));
        drive.backPixel.setPosition(0);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280,720))
                .build();

        Trajectory backdrop = drive.trajectoryBuilder(new Pose2d(12,70,3*Math.PI/2))
                .lineToLinearHeading(new Pose2d(40,43,Math.PI))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",
                hardwareMap.appContext.getPackageName());


        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 2"), cameraMonitorViewId);

        YellowDetector detector = new YellowDetector(telemetry);

        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                //called if cannot be opened
            }
        });
        waitForStart();

        while(!isStopRequested() && opModeIsActive()){

                    drive.followTrajectory(backdrop);
                    sleep(1000);

                    AprilTagDetection tag = tagProcessor.getDetections().get(0);


                    Trajectory lineYellow = drive.trajectoryBuilder(backdrop.end())
                           .lineToLinearHeading(new Pose2d(40,43-(tag.ftcPose.x),Math.PI))
                           .build();

                    TrajectorySequence placeYellowL = drive.trajectorySequenceBuilder(lineYellow.end())
                            .lineTo(new Vector2d(40,43-(tag.ftcPose.x-1)))
                            .back(11)
                            .build();

                    TrajectorySequence placeYellowR = drive.trajectorySequenceBuilder(lineYellow.end())
                        .lineTo(new Vector2d(40,43-(tag.ftcPose.x+1)))
                            .back(11)
                        .build();


                    sleep(500);

            drive.followTrajectory(lineYellow);


                    switch(detector.getLocation()){

                        case ON:
                            drive.followTrajectorySequence(placeYellowR);
                            drive.backPixel.setPosition(0.5);

                            break;
                        case OFF:
                            drive.followTrajectorySequence(placeYellowL);
                            break;


                    }

                    sleep(50);
                    drive.backPixel.setPosition(0.5);
                    sleep(1000000);



//                telemetry.addData("x",tag.ftcPose.x);
//                telemetry.addData("y",tag.ftcPose.y);
//                telemetry.addData("z",tag.ftcPose.z);
//                telemetry.addData("roll",tag.ftcPose.roll);
//                telemetry.addData("pitch",tag.ftcPose.pitch);
//                telemetry.addData("yaw",tag.ftcPose.yaw);
//                telemetry.update();


        }


    }

    public static class YellowDetector extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location{
            ON,
            OFF

        }
        private Location location;

        static final Rect LEFTBOX = new Rect(
                new Point(400,130),
                new Point(600,400));



        public YellowDetector(Telemetry t){ telemetry = t;}

        @Override
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);

            //range of blue
            Scalar lowHSV = new Scalar(20, 100, 100);
            Scalar highHSV = new Scalar(30, 255, 255);

            //only displays blue pixels
            Core.inRange(mat,lowHSV,highHSV,mat);

            //creates boxes for blue detection
            Mat left = mat.submat(LEFTBOX);

            double avgL = Core.mean(left).val[0];


            left.release();


            telemetry.addData("Left Raw Value", (int) Core.sumElems(left).val[0]);


            telemetry.addData("Left %: ", Math.round(avgL*100) + "%");




            if(avgL > 50) {
                //on Left
                location = Location.ON;
                telemetry.addData("Prop Location: ", "ON");
            } else {
                location = Location.OFF;
                telemetry.addData("Prop Location: ", "OFF");
            }


            telemetry.update();

            Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

            Scalar edge = new Scalar(0, 0, 255);
            Scalar found = new Scalar(0, 255, 0);


            Imgproc.rectangle(mat,LEFTBOX,location == Location.ON? found:edge, avgL > 50? -1:2);



            return mat;
        }
        public Location getLocation(){
            return location;
        }

    }


}
