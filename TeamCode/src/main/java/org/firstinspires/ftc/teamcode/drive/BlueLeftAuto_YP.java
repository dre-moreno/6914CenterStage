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

@Autonomous(name = "Comp: Blue Left - P&Y", group = "Blue Auto - YP")
public class BlueLeftAuto_YP extends LinearOpMode {

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware6914 drive = new Hardware6914(hardwareMap);

        drive.setPoseEstimate(new Pose2d(12,70,3*Math.PI/2));


        Trajectory backdropLeft = drive.trajectoryBuilder(new Pose2d(12,70,3*Math.PI/2))
                .lineToLinearHeading(new Pose2d(52,48,Math.PI))
                .build();

        Trajectory purpleLeft = drive.trajectoryBuilder(backdropLeft.end())
                .lineTo(new Vector2d(37 ,40))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(purpleLeft.end())
                .lineTo(new Vector2d(50,69))
                .back(10)
                .build();

        Trajectory backdropCenter = drive.trajectoryBuilder(new Pose2d(12,70,3*Math.PI/2))
                .lineToLinearHeading(new Pose2d(52,43,Math.PI))
                .build();

        Trajectory purpleCenter = drive.trajectoryBuilder(backdropCenter.end())
                .lineTo(new Vector2d(30,30))
                .build();
        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(purpleCenter.end())
                .lineTo(new Vector2d(50,69))
                .back(10)
                .build();

        Trajectory backdropRight = drive.trajectoryBuilder(new Pose2d(12,70,3*Math.PI/2))
                .lineToLinearHeading(new Pose2d(52,35,Math.PI))
                .build();

        Trajectory purpleRight = drive.trajectoryBuilder(backdropRight.end())
                .lineTo(new Vector2d(15,40))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(purpleRight.end())
                .lineTo(new Vector2d(50,69))
                .back(10)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",
                hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);

        PropDetector detector = new PropDetector(telemetry);

        webcam.setPipeline(detector);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
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
                sleep(50);
                drive.spoolAngleRight.setPosition(-.175);
                sleep(50);
                drive.followTrajectorySequence(parkLeft);
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
                sleep(50);
                drive.spoolAngleRight.setPosition(-.175);
                sleep(50);
                drive.followTrajectorySequence(parkCenter);
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
                sleep(50);
                drive.spoolAngleRight.setPosition(-.175);
                sleep(50);
                drive.followTrajectorySequence(parkRight);
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

            //range of blue
            Scalar lowHSV = new Scalar(105,100,20);
            Scalar highHSV = new Scalar(135,255,255);

            //only displays blue pixels
            Core.inRange(mat,lowHSV,highHSV,mat);

            //creates boxes for blue detection
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

            Scalar edge = new Scalar(0, 0, 255);
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
