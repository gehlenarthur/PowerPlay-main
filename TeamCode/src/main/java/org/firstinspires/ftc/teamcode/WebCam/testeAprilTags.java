/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.WebCam;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class testeAprilTags extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double METER = 1;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.0;

    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);



    public ElapsedTime runtime = null;
    // UNITS ARE METERS
    double tagsize = 0.166;

    int tag0 = 6; // Tag ID 18 from the 36h11 family
    int tag1 = 9;
    int tag2 = 12;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == tag0 || tag.id == tag1 || tag.id == tag2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        waitForStart();
        /* Update the telemetry */
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest.id == tag0) {
            telemetry.addLine("tag0 identificada");
            telemetry.update();
           drive(0.0, -20.86,  0.0, 0.0, 0.4, 0.0, 10.0);
           drive(-14.0, 0.0, 0.0, 0.4, 0.0, 0.0, 10.0);
            sleep(5000);

    }else if (tagOfInterest.id == tag1){
            telemetry.addLine("tag1 identificada");
            telemetry.update();
            drive(-14.0, 0.0, 0.0, 0.4, 0.0, 0.0, 10.0);
            sleep(5000);

        }else if (tagOfInterest.id == tag2){
            telemetry.addLine("tag2 identificada");
            drive(0.0, -20.86, 0.0, 0.0, 0.4, 0.0, 10.0);
            drive(-14.0, 0.0, 0.0,0.4,0.0,0.0,10.0);
            sleep(5000);



        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));


    }



    public void setWheelsPower(Double drive, Double strafe, Double twist) {

        List<Double> doubleList = new ArrayList<>();
        doubleList.add((drive + strafe + twist));
        doubleList.add((drive - strafe - twist));
        doubleList.add((drive - strafe + twist));
        doubleList.add((drive + strafe - twist));

        Double max = Math.abs(doubleList.get(0));
        for (int i = 1; i < doubleList.size(); i++) {
            if (max < Math.abs(doubleList.get(i))) {
                max = Math.abs(doubleList.get(i));
            }
        }
        if (max > 1) {
            for (int i = 0; i < doubleList.size(); i++) {
                doubleList.set(i, doubleList.get(i) / max);
            }
        }

        leftFront.setPower(doubleList.get(0));
        rightFront.setPower(doubleList.get(1));
        leftBack.setPower(doubleList.get(2));
        rightBack.setPower(doubleList.get(3));
    }


    public void drive(Double driveDist, Double strafeDist, Double twistDist, Double driveSpeed, Double strafeSpeed, Double twistSpeed, Double timeoutS) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist) * COUNTS_PER_INCH));
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist) * COUNTS_PER_INCH));
        leftFront.setTargetPosition(leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist) * COUNTS_PER_INCH));
        rightFront.setTargetPosition(leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist) * COUNTS_PER_INCH));

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        setWheelsPower(driveSpeed, strafeSpeed, twistSpeed);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy())) {

            Thread.yield();
        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setWheelsMode(DcMotor.RunMode runMode) {

        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public void initialize() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}



