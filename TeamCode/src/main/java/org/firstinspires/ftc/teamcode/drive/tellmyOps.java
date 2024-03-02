package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOp - V3", group = "comp")
public class tellmyOps extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware6914 drive = new Hardware6914(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rotation = 384.5;

        telemetry.addData("Where Parked?", "Up: Blue Corner, Right: Blue Middle, Left: Red Middle, Down: Red Corner");

        if(gamepad1.dpad_up){
            drive.setPoseEstimate(new Pose2d(60,69,Math.PI));
            telemetry.addData("Where Parked?", "Up: Blue Corner");
        } else if (gamepad1.dpad_right){
            drive.setPoseEstimate(new Pose2d(60,18,Math.PI));
            telemetry.addData("Where Parked?", "Right: Blue Middle");
        } else if (gamepad1.dpad_left){
            drive.setPoseEstimate(new Pose2d(60,-18,Math.PI));
            telemetry.addData("Where Parked?", "Left: Red Middle");
        } else {
            drive.setPoseEstimate(new Pose2d(-60,69,Math.PI));
            telemetry.addData("Where Parked?", "Down: Red Corner");
        }



        waitForStart();

        while (!isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            //spool
            //stick to the left is out; stick to the right is in
            if (gamepad2.right_trigger > 0) {
                drive.spoolOut.setPower(-gamepad2.right_trigger);
                drive.spoolIn.setPower(-gamepad2.right_trigger * .5);
            } else if (gamepad2.left_trigger >= 0) {
                drive.spoolOut.setPower(gamepad2.left_trigger);
                drive.spoolIn.setPower(gamepad2.left_trigger * .3);
            }

            if (gamepad2.a) {
                drive.claw.setPosition(.7);

            } else {
                drive.claw.setPosition(.98);
            }

            //hangarms
            //always down hold to raise

            if (gamepad1.right_trigger > 0) {
                drive.hangLeft.setPower(gamepad1.right_trigger * .8);
                drive.hangRight.setPower(-gamepad1.right_trigger * .8);
            } else {
                drive.hangLeft.setPower(0);
                drive.hangRight.setPower(0);
            }

            if (gamepad1.left_bumper) {
                drive.planeShooter.setPower(1);
            } else {
                drive.planeShooter.setPower(0);
            }

            if (gamepad1.b) {
                drive.planeShooter.setPower(-1);
            }
            //drive.spoolAngleLeft.setPosition(-gamepad2.left_stick_y / 2 + .325);
            drive.spoolAngleRight.setPosition(gamepad2.left_stick_y / 2 + .325);






            if (gamepad1.dpad_up) {
                drive.hangLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                drive.hangRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                drive.hangLeft.setTargetPosition(-(int) (rotation * 4.5));
                drive.hangRight.setTargetPosition((int) (rotation * 4.5));

                drive.hangLeft.setPower(.5);
                drive.hangRight.setPower(.5);

                drive.hangLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                drive.hangRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                while (drive.hangLeft.isBusy() && drive.hangRight.isBusy()) {
                    //just prints while the lift is going up.
                    telemetry.addData("Status", "Lift Arms Rasing");
                    telemetry.update();
                }

                drive.hangLeft.setPower(0);
                drive.hangRight.setPower(0);

                //so we can lower the hangarms
                drive.hangLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                drive.hangRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            drive.update();

            if(getRuntime() < 88) {
                drive.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (getRuntime() < 108){
                drive.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else {
                drive.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            if(gamepad1.x){
                Trajectory paper = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(15,0,Math.PI))
                        .build();

                drive.followTrajectory(paper);
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("color detected: ", drive.yellowDetector.argb());
            telemetry.addData("claw angle: ", drive.spoolAngleRight.getPosition());
            telemetry.addData("time ran: ", getRuntime());
            telemetry.update();
        }
    }
}
