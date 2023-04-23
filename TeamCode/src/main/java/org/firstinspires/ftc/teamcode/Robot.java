package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SeyiaTO2.COUNTS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.goBildaMotor.COUNTS_PER_DEGREE_GOBILDA;
import static org.firstinspires.ftc.teamcode.testeMecanunsDoisEncoders.COUNTS_PER_INCH;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.WebCam.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;


public class Robot {

    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx linear;
    public DcMotorEx lever;
    public DcMotorEx clawLinear;
    public DcMotorEx clawLever;

    public CRServo servoLinear;
    public CRServo servoLinear2;
    public CRServo servoLever;
    public CRServo servoLever2;

    int i = 3;

    double errorInicialLeftBack = 0;
    double errorInicialLeftFront = 0;
    double errorInicialRightBack = 0;
    double errorInicialRightFront = 0;

    double targetLinear = 0;
    double targetLever = 0;
    double targetLinearClaw = 0;
    double targetLeverClaw = 0;


    public BNO055IMU imu;
    public float lastHeadingAngleImu = 0;
    public double globalAngleImu = 0;

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

    // UNITS ARE METERS
    double tagsize = 0.166;

    int tag0 = 6; // Tag ID 18 from the 36h11 family
    int tag1 = 9;
    int tag2 = 12;

    public AprilTagDetection tagOfInterest = null;




    public HardwareMap hmap;

    public void setWheelsPower(double drive, double strafe, double twist) {
        // Cria uma lista dos motores e calcula qual deve ser a velocidade de giro do motor(power)
        List<Double> doubleList = new ArrayList<>();
        doubleList.add((drive - strafe + twist)); //0
        doubleList.add((drive + strafe - twist)); //1
        doubleList.add((drive + strafe + twist)); //2
        doubleList.add((drive - strafe - twist)); //3
        // leftFront      doubleList.add((drive - + 1 + twist)); //0 -1
        //  rightFront    doubleList.add((drive + +1 - twist)); //1 -1
        // leftBack       doubleList.add((drive + +1 + twist)); //2 +1
        // rightBack      doubleList.add((drive - +1 - twist)); //3 +1
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
        // Pega os valores de power calculados da lista e faz com que os motores girem com o devido power
        leftFront.setPower(doubleList.get(0));
        rightFront.setPower(doubleList.get(1));
        rightBack.setPower(doubleList.get(2));
        leftBack.setPower(doubleList.get(3));

    }

    public void initBase(HardwareMap hmap) {

        leftFront = hmap.get(DcMotorEx.class, "FLMotor");
        rightFront = hmap.get(DcMotorEx.class, "FRMotor");
        leftBack = hmap.get(DcMotorEx.class, "BLMotor");
        rightBack = hmap.get(DcMotorEx.class, "BRMotor");

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    public void initServos(HardwareMap hmap){
        servoLever = hmap.get(CRServo.class, "servoLever");
        servoLever2 = hmap.get(CRServo.class, "servoLever2");
        servoLinear= hmap.get(CRServo.class, "servoLinear");
        servoLinear2 = hmap.get(CRServo.class, "servoLinear2");

    }

    public void initMotors(HardwareMap hmap){
        linear = hmap.get(DcMotorEx.class, "linear");
        clawLinear = hmap.get(DcMotorEx.class, "clawLinear");
        lever = hmap.get(DcMotorEx.class, "lever");
        clawLever = hmap.get(DcMotorEx.class, "clawLever");

        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lever.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLever.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawLever.setDirection(DcMotorEx.Direction.REVERSE);
        lever.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void initWebCam(HardwareMap hMap) {

        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void identicacaoWebCam() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == tag0 || tag.id == tag1 || tag.id == tag2) {
                    tagOfInterest = tag;
                    break;
                }
            }
        }
    }

    public void setWheelsPowerSpeedControl(double drive, double strafe, double twist, double speed) {
        // Cria uma lista dos motores e calcula qual deve ser a velocidade de giro do motor(power)
        List<Double> doubleList = new ArrayList<>();
        doubleList.add((drive - strafe + twist)); //0
        doubleList.add((drive + strafe - twist)); //1
        doubleList.add((drive + strafe + twist)); //2
        doubleList.add((drive - strafe - twist)); //3
        // leftFront      doubleList.add((drive - + 1 + twist)); //0 -1
        //  rightFront    doubleList.add((drive + +1 - twist)); //1 -1
        // leftBack       doubleList.add((drive + +1 + twist)); //2 +1
        // rightBack      doubleList.add((drive - +1 - twist)); //3 +1
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
        // Pega os valores de power calculados da lista e faz com que os motores girem com o devido power
        leftFront.setVelocity(doubleList.get(0) * speed);
        rightFront.setVelocity(doubleList.get(1) * speed);
        rightBack.setVelocity(doubleList.get(2) * speed);
        leftBack.setVelocity(doubleList.get(3) * speed);

    }

    public void driveUsingEncoder(Double driveDist, Double strafeDist, Double twistDist, Double driveSpeed,
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
                leftFront.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        errorInicialLeftBack = leftBack.getCurrentPosition() - leftBack.getTargetPosition();
        errorInicialLeftFront = leftFront.getCurrentPosition() - leftFront.getTargetPosition();
        errorInicialRightBack = rightBack.getCurrentPosition() - rightBack.getTargetPosition();
        errorInicialRightFront = rightFront.getCurrentPosition() - rightFront.getTargetPosition();

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        setWheelsPower(Math.abs(driveSpeed), Math.abs(strafeSpeed), Math.abs(twistSpeed));

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy())) {

            java.lang.Thread.yield();
        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveUsingEncoderDesaceleracaoSpeedContorl(Double driveDist, Double strafeDist, Double twistDist, Double timeoutS, double speed) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));
        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        errorInicialLeftBack = leftBack.getCurrentPosition() - leftBack.getTargetPosition();
        errorInicialLeftFront = leftFront.getCurrentPosition() - leftFront.getTargetPosition();
        errorInicialRightBack = rightBack.getCurrentPosition() - rightBack.getTargetPosition();
        errorInicialRightFront = rightFront.getCurrentPosition() - rightFront.getTargetPosition();

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy())) {

            double driveSpeed = 0;
            double twistSpeed = 0;
            double strafeSpeed = 0;

            if (driveDist != 0.0) {

                driveSpeed = 0.25 + returnPowerBase(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (twistDist != 0.0) {

                twistSpeed = 0.25 + returnPowerBase(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (strafeDist != 0.0) {

                strafeSpeed = 0.25 + returnPowerBase(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }

            setWheelsPowerSpeedControl(Math.abs(driveSpeed), Math.abs(twistSpeed), Math.abs(strafeSpeed), speed);

        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveUsingEncoderDesaceleracao(Double driveDist, Double strafeDist, Double twistDist, Double timeoutS) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));
        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        errorInicialLeftBack = leftBack.getCurrentPosition() - leftBack.getTargetPosition();
        errorInicialLeftFront = leftFront.getCurrentPosition() - leftFront.getTargetPosition();
        errorInicialRightBack = rightBack.getCurrentPosition() - rightBack.getTargetPosition();
        errorInicialRightFront = rightFront.getCurrentPosition() - rightFront.getTargetPosition();

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy())) {

            double driveSpeed = 0;
            double twistSpeed = 0;
            double strafeSpeed = 0;

            if (driveDist != 0.0) {

                driveSpeed = 0.25 + returnPowerBase(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (twistDist != 0.0) {

                twistSpeed = 0.25 + returnPowerBase(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (strafeDist != 0.0) {

                strafeSpeed = 0.25 + returnPowerBase(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }

            setWheelsPower(Math.abs(driveSpeed), Math.abs(twistSpeed), Math.abs(strafeSpeed));

        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveUsingEncoderDesaceleracaoMetadeSpeedContorl(Double driveDist, Double strafeDist, Double twistDist, Double timeoutS, double speed) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));
        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        errorInicialLeftBack = leftBack.getCurrentPosition() - leftBack.getTargetPosition();
        errorInicialLeftFront = leftFront.getCurrentPosition() - leftFront.getTargetPosition();
        errorInicialRightBack = rightBack.getCurrentPosition() - rightBack.getTargetPosition();
        errorInicialRightFront = rightFront.getCurrentPosition() - rightFront.getTargetPosition();

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) || (leftFront.isBusy()/* && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy()*/)) {

            double driveSpeed = 0;
            double twistSpeed = 0;
            double strafeSpeed = 0;

            if (driveDist != 0.0) {

                driveSpeed = 0.4 + returnPowerBaseMetade(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (twistDist != 0.0) {

                twistSpeed = 0.4 + returnPowerBaseMetade(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (strafeDist != 0.0) {

                strafeSpeed = 0.4 + returnPowerBaseMetade(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }

            setWheelsPowerSpeedControl(Math.abs(driveSpeed), Math.abs(twistSpeed), Math.abs(strafeSpeed), speed);

        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void driveUsingEncoderDesaceleracaoMetade(Double driveDist, Double strafeDist, Double twistDist, Double timeoutS){

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));
        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        errorInicialLeftBack = leftBack.getCurrentPosition() - leftBack.getTargetPosition();
        errorInicialLeftFront = leftFront.getCurrentPosition() - leftFront.getTargetPosition();
        errorInicialRightBack = rightBack.getCurrentPosition() - rightBack.getTargetPosition();
        errorInicialRightFront = rightFront.getCurrentPosition() - rightFront.getTargetPosition();

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) || (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy())) {

            double driveSpeed = 0;
            double twistSpeed = 0;
            double strafeSpeed = 0;

            if (driveDist != 0.0) {

                driveSpeed = 0.25 + returnPowerBaseMetade(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (twistDist != 0.0) {

                twistSpeed = 0.25 + returnPowerBaseMetade(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }
            if (strafeDist != 0.0) {

                strafeSpeed = 0.25 + returnPowerBaseMetade(leftBack.getTargetPosition(), leftFront.getTargetPosition(), rightBack.getTargetPosition(), rightFront.getTargetPosition());
            }

            setWheelsPower(Math.abs(driveSpeed), Math.abs(twistSpeed), Math.abs(strafeSpeed));

        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void encoder(double degrees, double time, double power, double counts, double target, DcMotorEx motor) {

        target = degrees * counts;
        motor.setTargetPosition((int) target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double currentVelocity = motor.getVelocity();
            java.lang.Thread.yield();
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runTime = null;
    }



    public void encoderTeleop(double deegres,double power, double counts, double target, DcMotorEx motor) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

    }

    public void encoderTeleopSpeedControl(double deegres,double power, double counts, double target, DcMotorEx motor,double speed) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(power * speed);

    }


    public void encoderSpeedControl(double deegres, double time, double counts, double target,DcMotorEx motor, double speed){

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(speed);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            java.lang.Thread.yield();
        }

        runTime = null;
    }

    public void encoderDesaceleracao(double deegres, double time, double counts, double target, double adicional,DcMotorEx motor){

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double power = returnpower(target, errorInicial, motor);
            motor.setPower(power + adicional );
        }

        runTime = null;

    }

    public void encoderDesaceleracaoSpeedControl(double deegres, double time, double counts, double target, double adicional,DcMotorEx motor,double speed) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double power = returnpower(target, errorInicial, motor);
            motor.setVelocity((power + adicional) * speed);
        }

        runTime = null;
    }

    public void encoderDesaceleracaoTeleOPSpeedControl(double deegres,  double counts, double target, double adicional,DcMotorEx motor,double speed) {


        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = returnpower(target, errorInicial, motor);
        motor.setVelocity((power + adicional) * speed);

    }

    public void encoderDesaceleracaoLever(double deegres, double time, double counts, double target, double adicional,DcMotorEx motor) {

        setWheelsPower(0,0,0);

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() +(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double power = returnpowerDesaleracaoLever(target, errorInicial, motor);
            motor.setPower(power + adicional );
        }

        runTime = null;
    }

    public void encoderDesaceleracaoTeleOP(double deegres,  double counts, double target, double adicional,DcMotorEx motor) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = returnpower(target, errorInicial, motor);
        motor.setPower(power + adicional );

    }


    public void encoderAceleracao(double deegres, double time, double counts, double target,double adicional, DcMotorEx motor) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double power = returnpoweracelerao(target, errorInicial, motor);
            motor.setPower(power +  adicional);
        }

        runTime = null;
    }

    public double returnpoweracelerao(double target, double errorInicial, DcMotorEx motor) {

        return (  motor.getCurrentPosition() / target);

    }

    public void encoderDesaceleracaoMetade(double deegres, double time, double counts, double target,double adicional,
                                           DcMotorEx motor){

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double power = returnpowerMetade(target, errorInicial, motor);
            motor.setPower(power + adicional);

        }

        runTime = null;
    }

    public void encoderDesaceleracaoMetadeSpeedControl(double deegres, double time, double counts, double target,double adicional, DcMotorEx motor,double speed) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double power = returnpowerMetade(target, errorInicial, motor);
            motor.setVelocity((power + adicional) * speed);

        }

        runTime = null;
    }

    public void encoderDesaceleracaoMetadeTeleOP(double deegres, double counts, double target,double adicional, DcMotorEx motor) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = returnpowerMetade(target, errorInicial, motor);
        motor.setPower(power + adicional);

    }

    private void setWheelsMode(DcMotor.RunMode runMode) {

        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public double returnPowerBase(double targetLeftBack,double targetLeftFront,double targetRightBack,double targetRightFront){

        double powerLeftBack = returnpower(targetLeftBack, errorInicialLeftBack, leftBack);
        double powerLeftFront = returnpower(targetLeftFront, errorInicialLeftFront, leftFront);
        double powerRightBack = returnpower(targetRightBack, errorInicialRightBack, rightBack);
        double powerRightFront = returnpower(targetRightFront, errorInicialRightFront, rightFront);

        return (powerRightFront + powerRightBack + powerLeftFront + powerLeftBack) / 4;
    }

    public double returnPowerBaseMetade(double targetLeftBack, double targetLeftFront, double targetRightBack,
                                        double targetRighrFront){

        double powerLeftBack = returnpowerMetade(targetLeftBack, errorInicialLeftBack, leftBack);
        double powerLeftFront = returnpowerMetade(targetLeftFront, errorInicialLeftFront, leftFront);
        double powerRightBack = returnpowerMetade(targetRightBack, errorInicialRightBack, rightBack);
        double powerRightFront = returnpowerMetade(targetRighrFront, errorInicialRightFront, rightFront);

        return (powerRightFront + powerRightBack + powerLeftFront + powerLeftBack) / 4;
    }

    public void encoderDesaceleracaoMetadePower(double deegres,double power, double time, double counts, double target,double adicional, DcMotorEx motor) {

        target = deegres * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            double powerFull = returnpowerMetadePower(target,power, errorInicial, motor);
            motor.setPower(powerFull + adicional);

        }

        runTime = null;
    }

    public double returnpowerDesaleracaoLever(double target, double errorInicial, DcMotorEx motor) {



        int error = (int) (motor.getCurrentPosition() + (target));

        return (error / errorInicial);



    }

    public double returnpower(double target, double errorInicial, DcMotorEx motor) {

        if (target > motor.getCurrentPosition()) {

            int error = (int) (target - motor.getCurrentPosition());

            return (error / target);
        } else if (target < motor.getCurrentPosition()) {

            int error = (int) (motor.getCurrentPosition() - target);

            return (error / errorInicial);

        }
        return 1.0;
    }

    public double returnpowerMetade(double target, double errorInicial, DcMotorEx motor) {

        if (target > motor.getCurrentPosition()) {

            int error = (int) (target - motor.getCurrentPosition());
            if (error / target < 0.5) {

                double targetMetade = target / 2;
                double errorMeatade = targetMetade - (motor.getCurrentPosition() - targetMetade);

                return ((errorMeatade / targetMetade));
            }
            return 1;

        } else if (motor.getCurrentPosition() > target) {

            int error = (int) (motor.getCurrentPosition() - target);
            if (error / errorInicial < 0.5) {

                double power = error / errorInicial;
                double porcentagemPower = (power / 0.5) * 100;

                return 0.01 * porcentagemPower;
            }
            return 1;
        }
        return 1.0;
    }

    public double returnpowerMetadePower(double target,double power, double errorInicial, DcMotorEx motor) {
        if (target > motor.getCurrentPosition()) {

            int error = (int) (target - motor.getCurrentPosition());
            if (error / target < 0.5) {

                double targetMetade = target / 2;
                double errorMeatade = targetMetade - (motor.getCurrentPosition() - targetMetade);
                double porcentagem =  ((errorMeatade / targetMetade));
                return  porcentagem * power;
            }
            return power;
        } else if (motor.getCurrentPosition() > target) {

            int error = (int) (motor.getCurrentPosition() - target);
            if (error / errorInicial < 0.5) {

                double powerFull = error / errorInicial;
                double porcentagemPower = (power / 0.5) * 100;
                double porcentagem =  0.01 * porcentagemPower;
                return porcentagem * porcentagem;
            }
            return power;
        }
        return power;
    }

    public double returnpowerImu(double degrees, double errorInicial) {

        if (degrees > getAngle()) {

            int error = (int) (degrees - getAngle());

            return (error / degrees);
        } else if (degrees < getAngle()) {

            int error = (int) (getAngle() - degrees);

            return (error / errorInicial);

        }
        return 0.0;
    }

    public void MoveServosAuto(Servo servo, Servo servo2, double position, double position2) {
        ElapsedTime runtimeServo = new ElapsedTime();
        while (runtimeServo.seconds() < 5) {

            servo.setPosition(position);
            servo2.setPosition(position2);

        }
    }

    public void MoveServos(Servo servo, Servo servo2, double position, double position2) {
        ElapsedTime runtimeServo = new ElapsedTime();


        servo.setPosition(position);
        servo2.setPosition(position2);


    }

    public void setupImu(HardwareMap hMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.imu = hMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }

    public void rotateUsingImu(int degrees) {

        resetAngle();

        double errorInicial = getAngle() - degrees;

        if (degrees < 0) {
            while ((getAngle() >= degrees) || (getAngle() == 0)) {
                setWheelsPowerFromDegrees(degrees, errorInicial);
            }
        } else {
            while (getAngle() <= degrees) {
                setWheelsPowerFromDegrees(degrees, errorInicial);
            }
        }

        setWheelsPower(0.00, 0.00, 0.00);
    }

    public void resetAngle() {


        lastHeadingAngleImu = imu
                .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        globalAngleImu = 0;
    }

    public double getAngle() {

        float headingAngleImu = imu
                .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double deltaAngle = headingAngleImu - lastHeadingAngleImu;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngleImu += deltaAngle;
        lastHeadingAngleImu = headingAngleImu;

        return globalAngleImu;
    }

    public double checkDirection() {

        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0) {
            correction = 0;
        } else {
            correction = -angle;
        }

        correction = correction * gain;

        return correction;
    }

    private void setWheelsPowerFromDegrees(int degrees, double errorInicial) {

        // double power = 1 - (Math.abs(getAngle()) / Math.abs(degrees));

        setWheelsPowerSpeedControl(0.00, 0.00, degrees > 0 ? -(returnpowerImu(degrees, errorInicial) + 0.01) : returnpowerImu(degrees, errorInicial) + 0.01, 2.5 * 560);
    }

}