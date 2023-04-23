package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class goBildaMotor extends LinearOpMode {

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    static final double COUNT_PER_REVOLTION = 560;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.0;
    public static final double COUNTS_PER_INCH =
            (COUNT_PER_REVOLTION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //5267.65
    static final double COUNTS_PER_REVOLUTION_GOBILDA = 384.5;
    static final double DRIVE_GEAR_REDUCTION_GOBILDA = 1;
    static final double COUNTS_PER_DEGREE_GOBILDA = (COUNTS_PER_REVOLUTION_GOBILDA * DRIVE_GEAR_REDUCTION_GOBILDA) / 360;

    double Power = 1;
    double targetPosition = 0;
    ElapsedTime runTime;
    public DcMotor GoBildaMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        GoBildaMotor = hardwareMap.get(DcMotor.class,"motor");
        GoBildaMotor.setDirection(DcMotor.Direction.REVERSE);
        GoBildaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GoBildaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        encoder(360 * 8.65, 5,0.5);
        sleep(1500);
        encoderDesaceleracao(0,10);



    }

    public void encoderDesaceleracao(double deegres, double time){



        targetPosition = deegres * COUNTS_PER_DEGREE_GOBILDA;
        GoBildaMotor.setTargetPosition((int)targetPosition);
        GoBildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime = new ElapsedTime();
        while (runTime.seconds() < time && GoBildaMotor.isBusy()) {

            returnpower();
            GoBildaMotor.setPower(Power);
            telemetry();
        }

        runTime = null;
    }

    public void encoder(double deegres, double time,double power){

        targetPosition = deegres * COUNTS_PER_DEGREE_GOBILDA;
        GoBildaMotor.setTargetPosition((int)targetPosition);
        GoBildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime = new ElapsedTime();
        while (runTime.seconds() < time && GoBildaMotor.isBusy()) {

            returnpower();
            GoBildaMotor.setPower(power);
            telemetry();
        }

        runTime.reset();
    }

    public void returnpower(){

        int error = (int) (targetPosition - GoBildaMotor.getCurrentPosition());


        Power = (error / targetPosition);

    }

    public void returnpowerBase(){

        double position = leftBack.getCurrentPosition() + leftFront.getCurrentPosition();

        int error = (int) (targetPosition - GoBildaMotor.getCurrentPosition());


        Power = (error / targetPosition);

    }
    public  void telemetry(){

        telemetry.addData("posição Atual : ",GoBildaMotor.getCurrentPosition());
        telemetry.addData("power : ",GoBildaMotor.getPower());
        telemetry.addData("target : ",GoBildaMotor.getTargetPosition());
        telemetry.update();
    }

    public void setWheelsPower(double drive, double strafe, double twist) {

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

    public void drive(Double driveDist, Double strafeDist, Double twistDist, Double driveSpeed,
                                           Double strafeSpeed, Double twistSpeed, Double timeoutS) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));
        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + (int) ((driveDist + strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist - strafeDist - twistDist)
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
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
    }

}

