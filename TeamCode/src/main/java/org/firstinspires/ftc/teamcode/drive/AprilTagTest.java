package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "AprilTagTest")
public class AprilTagTest extends LinearOpMode {

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
                .enableLiveView(true)
                .build();






        Trajectory backdrop = drive.trajectoryBuilder(new Pose2d(12,70,3*Math.PI/2))
                .lineToLinearHeading(new Pose2d(40,43,Math.PI))
                .build();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){



                    drive.followTrajectory(backdrop);
                    sleep(1000);
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);

                   Trajectory placeYellow = drive.trajectoryBuilder(backdrop.end())
                .lineToLinearHeading(new Pose2d(12,drive.getPoseEstimate().getY()+tag.ftcPose.x,Math.PI))
                           .back(12)
                .build();
                            sleep(500);
                            drive.updatePoseEstimate();
                            sleep(500);
                            drive.followTrajectory(placeYellow);
                            sleep(50);
                            drive.backPixel.setPosition(0.5);

                    sleep(1000000);





//            if(tagProcessor.getDetections().size() > 0){
//                AprilTagDetection tag = tagProcessor.getDetections().get(0);
//
//                telemetry.addData("x",tag.ftcPose.x);
//                telemetry.addData("y",tag.ftcPose.y);
//                telemetry.addData("z",tag.ftcPose.z);
//                telemetry.addData("roll",tag.ftcPose.roll);
//                telemetry.addData("pitch",tag.ftcPose.pitch);
//                telemetry.addData("yaw",tag.ftcPose.yaw);
//                telemetry.update();
//
//            }
        }


    }



}
