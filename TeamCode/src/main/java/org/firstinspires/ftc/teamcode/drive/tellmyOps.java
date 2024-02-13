package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp - 1/26/24", group = "comp")
public class tellmyOps extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware drive = new Hardware(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rotation = 384.5;

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
            if(gamepad2.right_trigger > 0) {
                drive.spoolOut.setPower(-gamepad2.right_trigger);
                drive.spoolIn.setPower(-gamepad2.right_trigger * .25);
            } else if (gamepad2.left_trigger >= 0){
                drive.spoolOut.setPower(gamepad2.left_trigger * .8);
                drive.spoolIn.setPower(gamepad2.left_trigger);
            }

            if(gamepad2.a){
                drive.claw.setPosition(.7);

            } else {
                drive.claw.setPosition(.95);
            }

            //hangarms
            //always down hold to raise

            if(gamepad1.left_trigger > 0) {
                drive.hangLeft.setPower(-gamepad1.left_trigger*.8);
                drive.hangRight.setPower(gamepad1.left_trigger*.8);
            } else if (gamepad1.right_trigger > 0){
                drive.hangLeft.setPower(gamepad1.right_trigger*.8);
                drive.hangRight.setPower(-gamepad1.right_trigger*.8);
            } else {
                drive.hangLeft.setPower(0);
                drive.hangRight.setPower(0);
            }

            if(gamepad1.left_bumper) {
                drive.planeShooter.setPower(-1);
            } else {
                drive.planeShooter.setPower(0);

            }
            drive.spoolAngleLeft.setPosition(-gamepad2.left_stick_y/2+.325);
            drive.spoolAngleRight.setPosition(gamepad2.left_stick_y/2+.325);


//            drive.backPixel.setPosition(.5 + gamepad2.left_stick_x/2);
     //       drive.backPixel.setPosition(gamepad2.left_stick_x/2);


            if(gamepad2.dpad_up){
                drive.hangLeft.setTargetPosition((int)(5*rotation));
                drive.hangRight.setTargetPosition((int)(5*rotation));

                drive.hangLeft.setPower(-.3);
                drive.hangRight.setPower(.3);

                drive.hangLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.hangRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(drive.hangLeft.isBusy() && drive.hangRight.isBusy()){
                    //just prints while the lift is going up.
                    telemetry.addData("Status", "Lift Arms Rasing");
                    telemetry.update();
                }

                drive.hangLeft.setPower(0);
                drive.hangRight.setPower(0);

                //so we can lower the hangarms
                drive.hangLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.hangRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("left_stick",gamepad2.left_stick_x);
            telemetry.addData("servo pos", drive.backPixel.getPosition());
            telemetry.update();
        }
    }
}
