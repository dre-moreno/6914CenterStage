package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

@Autonomous(name = "Comp: Red Right - P&Y&W", group = "Red Auto - YPW")
public class RedRightAuto_YPW extends LinearOpMode {

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware6914 drive = new Hardware6914(hardwareMap);
        Pose2d start = new Pose2d(12,-70,Math.PI/2);
        drive.setPoseEstimate(start);


        Trajectory backdropLeft = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(52,-38.5,Math.PI))
                .build();

        Trajectory purpleLeft = drive.trajectoryBuilder(backdropLeft.end())
                .lineTo(new Vector2d(15 ,-40))
                .build();


        Trajectory backdropCenter = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(51.5,-44,Math.PI))
                .build();

        Trajectory purpleCenter = drive.trajectoryBuilder(backdropCenter.end())
                .lineTo(new Vector2d(30,-33))
                .build();


        Trajectory backdropRight = drive.trajectoryBuilder(start)
                .lineToLinearHeading(new Pose2d(51.5,-50,Math.PI))
                .build();

        Trajectory purpleRight = drive.trajectoryBuilder(backdropRight.end())
                .lineTo(new Vector2d(37,-40))
                .build();

        //white collection
        Trajectory lineWhiteLeft = drive.trajectoryBuilder(purpleLeft.end())
                .lineTo(new Vector2d(46,-19))
                .build();
        Trajectory lineWhiteCenter = drive.trajectoryBuilder(purpleCenter.end())
                .lineTo(new Vector2d(46,-19))
                .build();
        Trajectory lineWhiteRight = drive.trajectoryBuilder(purpleRight.end())
                .lineTo(new Vector2d(46,-19))
                .build();

        Trajectory goToWhite = drive.trajectoryBuilder(lineWhiteCenter.end())
                .lineTo(new Vector2d(-52,-19))
                .build();


        TrajectorySequence parkCorner = drive.trajectorySequenceBuilder(goToWhite.end())
                .back(99)
                .lineToLinearHeading(new Pose2d(50,-17,0))
                .forward(2)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(goToWhite.end())
                .back(99)
                .lineToLinearHeading(new Pose2d(50,-69,0))
                .forward(2)
                .build();

        TrajectorySequence park = null;

        telemetry.addData("Park Location?", "D-Pad Left for Middle : D-Pad Right for Corner");
        telemetry.update();
        if(gamepad1.dpad_left){
            park = parkMiddle;
            telemetry.addData("Park Location: ", "Middle");
            telemetry.update();
        } else if(gamepad1.dpad_right){
            park = parkCorner;
            telemetry.addData("Park Location: ", "Corner");
            telemetry.update();
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",
                hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);

        PropDetector detector = new PropDetector(telemetry);

        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920,1080,OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                //called if cannot be opened
            }
        });



        waitForStart();

        telemetry.addData("Analysis", detector.getLocation());
        telemetry.update();

        switch (detector.getLocation()){
            case LEFT:
                drive.followTrajectory(backdropLeft);
                sleep(50);
                drive.backPixel.setPosition(.5);
                sleep(2000);
                drive.backPixel.setPosition(0);
                drive.spoolAngleRight.setPosition(.325);
                drive.followTrajectory(purpleLeft);
                drive.claw.setPosition(.7);
                sleep(100);
                drive.spoolAngleRight.setPosition(0);
                sleep(50);
                //white collection
                drive.followTrajectory(lineWhiteLeft);
                drive.followTrajectory(goToWhite);
                drive.spoolAngleRight.setPosition(.25);
                sleep(1000);
                drive.claw.setPosition(.99);
                sleep(1000);
                drive.spoolAngleRight.setPosition(0);
                drive.followTrajectorySequence(park);
                drive.spoolAngleRight.setPosition(.325);
                sleep(100);
                drive.claw.setPosition(.7);

                sleep(100000000);
                break;
            case CENTER:
                drive.followTrajectory(backdropCenter);
                sleep(50);
                drive.backPixel.setPosition(.5);
                sleep(2000);
                drive.backPixel.setPosition(0);
                drive.spoolAngleRight.setPosition(.325);
                drive.followTrajectory(purpleCenter);
                sleep(100);
                drive.claw.setPosition(.7);
                sleep(100);
                drive.spoolAngleRight.setPosition(0);
                sleep(50);
                //white colleciton
                drive.followTrajectory(lineWhiteCenter);
                drive.followTrajectory(goToWhite);
                drive.spoolAngleRight.setPosition(.25);
                sleep(1000);
                drive.claw.setPosition(.99);
                sleep(1000);
                drive.spoolAngleRight.setPosition(0);
                drive.followTrajectorySequence(park);
                drive.spoolAngleRight.setPosition(.325);
                sleep(100);
                drive.claw.setPosition(.7);

                sleep(100000000);
                break;
            case RIGHT:
                drive.followTrajectory(backdropRight);
                sleep(50);
                drive.backPixel.setPosition(.5);
                sleep(2000);
                drive.backPixel.setPosition(0);
                drive.spoolAngleRight.setPosition(.325);
                drive.followTrajectory(purpleRight);
                sleep(100);
                drive.claw.setPosition(.7);
                sleep(100);
                drive.spoolAngleRight.setPosition(0);
                sleep(50);
                //white collection
                drive.followTrajectory(lineWhiteRight);
                drive.followTrajectory(goToWhite);
                drive.spoolAngleRight.setPosition(.25);
                sleep(1000);
                drive.claw.setPosition(.99);
                sleep(1000);
                drive.spoolAngleRight.setPosition(0);
                drive.followTrajectorySequence(park);
                drive.spoolAngleRight.setPosition(.325);
                sleep(100);
                drive.claw.setPosition(.7);

                sleep(100000000);
                break;
        }

    }





    public static class PropDetector extends OpenCvPipeline{
        Telemetry telemetry;
        Mat mat = new Mat();

        public enum Location{
            LEFT,
            CENTER,
            RIGHT
        }

        private Location location;

        static final Rect LEFTBOX = new Rect(
                new Point(100,130),
                new Point(250,395));
        static final Rect CENTERBOX = new Rect(
                new Point(830,25),
                new Point(980,300));
        static final Rect RIGHTBOX = new Rect(
                new Point(1400,175),
                new Point(1550,450));

        public PropDetector(Telemetry t){ telemetry = t;}

        @Override
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);

            //range of red
            Scalar lowHSV = new Scalar(100,100,20);
            Scalar highHSV = new Scalar(180,255,255);

            //only displays red pixels
            Core.inRange(mat,lowHSV,highHSV,mat);

            //creates boxes for red detection
            Mat left = mat.submat(LEFTBOX);
            Mat center = mat.submat(CENTERBOX);
            Mat right = mat.submat(RIGHTBOX);

            double avgL = Core.mean(left).val[0];
            double avgC = Core.mean(center).val[0];
            double avgR = Core.mean(right).val[0];

            left.release();
            center.release();
            right.release();

            telemetry.addData("Left Raw Value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Center Raw Value", (int) Core.sumElems(center).val[0]);
            telemetry.addData("Right Raw Value", (int) Core.sumElems(right).val[0]);

            telemetry.addData("Left %: ", Math.round(avgL*100) + "%");
            telemetry.addData("Center %: ", Math.round(avgC*100) + "%");
            telemetry.addData("Right %: ", Math.round(avgR*100) + "%");


            double maxOneTwo = Math.max(avgL,avgC);
            double max = Math.max(maxOneTwo,avgR);

            if(max == avgL){
                //on Left
                location = Location.LEFT;
                telemetry.addData("Prop Location: ", "LEFT");
            } else if(max == avgC){
                //on Center
                location = Location.CENTER;
                telemetry.addData("Prop Location: ", "CENTER");
            }else {
                //on Right
                location = Location.RIGHT;
                telemetry.addData("Prop Location: ", "RIGHT");
            }

            telemetry.update();

            Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

            //blue
            Scalar edge = new Scalar(0, 0, 255);
            //green
            Scalar found = new Scalar(0, 255, 0);


            Imgproc.rectangle(mat,LEFTBOX,location == Location.LEFT? found:edge, max == avgL? -1:2);
            Imgproc.rectangle(mat,CENTERBOX,location == Location.CENTER? found:edge,max == avgC? -1:2);
            Imgproc.rectangle(mat,RIGHTBOX,location == Location.RIGHT? found:edge,max == avgR? -1:2);


            return mat;
        }
        public Location getLocation(){
            return location;
        }

    }

}