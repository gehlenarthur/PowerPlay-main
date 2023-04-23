package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "testeMacanuns")
public class testeMecanunsDoisEncoders extends LinearOpMode {

    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.0;
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotor leftFront ;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();



            telemetry();
            //driveUsingEncoderTwoMotors(12.0,0.0,0.0,0.5,0.0,0.0,10.0);
            //sleep(4000);
              driveUsingEncoderTwoMotors(0.0,12.0,0.0,0.0,0.5,0.0,10.0);

                      while(opModeIsActive()){

                          telemetry.update();

        }


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

        leftBack.setPower(doubleList.get(0));
        rightBack.setPower(doubleList.get(1));

        while(leftBack.isBusy() && rightBack.isBusy()) {

            leftFront.setPower(doubleList.get(0));
            rightFront.setPower(doubleList.get(1));

        }

        leftFront.setPower(0);
        rightFront.setPower(0);


    }

    public void driveUsingEncoderTwoMotors(Double driveDist, Double strafeDist, Double twistDist, Double driveSpeed,
                                           Double strafeSpeed, Double twistSpeed, Double timeoutS) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        setWheelsPower(Math.abs(driveSpeed), Math.abs(strafeSpeed), Math.abs(twistSpeed));

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy())) {

            Thread.yield();
        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    private void setWheelsMode(DcMotor.RunMode runMode) {

        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void telemetry(){

        telemetry.addData("leftBack",leftBack.getPower());
        telemetry.addData("rightBack",rightBack.getPower());
        telemetry.addData("leftFront",leftFront.getPower());
        telemetry.addData("rightFront",rightFront.getPower());
        telemetry.update();
    }
}