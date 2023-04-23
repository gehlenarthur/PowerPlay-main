package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.PortUnreachableException;
import java.util.ArrayList;
import java.util.List;
/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Sensor: MR Color2", group = "Sensor")

public class SensorMRColor2 extends LinearOpMode {

    static final double COUNTS_PER_REVOLUTION = 1680;
    static final double DRIVE_GEAR_REDUCTION_ULTRA = 2;
    static final double COUNTS_PER_DEGREE_ULTRA = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION_ULTRA) / 360;

    public static final double COUNTS_PER_MOTOR_CORE = 288;
    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.0;

    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public  static  final double COUNTS_PER_DEGREE_CORE = (COUNTS_PER_MOTOR_CORE) / 360;

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotorEx claw = null;
    public  double clawTarget = 0;

    ColorSensor colorSensor;    // Hardware Device Object

    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float[] hsvValues = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float[] values = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
        initialize();

        // wait for the start button to be pressed.
        waitForStart();

        driveUsingEncoderTwoMotors(15.3543, 0.0, 0.0, 0.8, 0.0, 0.0, 5.0);

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if (bCurrState && (bCurrState != bPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            sleep(3000);

            if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {

                driveUsingEncoderTwoMotors(0.0, 0.0, -15.15748, 0.0, 0.0, -0.4, 10.0);
                rotateClaw(30);
                sleep(1000);
                driveUsingEncoderTwoMotors(23.622, 0.0, 0.0, 0.9, 0.0, 0.0, 5.0);
                setWheelsPower(0.0, 0.0, 0.0);
                sleep(30000);

            } else if (colorSensor.blue() > colorSensor.green()) {

                driveUsingEncoderTwoMotors(0.0, 0.0, 15.15748, 0.0, 0.0, -0.4, 10.0);
                sleep(1000);
                driveUsingEncoderTwoMotors(23.622, 0.0, 0.0, 0.9, 0.0, 0.0, 5.0);
                setWheelsPower(0.0, 0.0, 0.0);
                sleep(30000);

            }

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
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
        leftBack.setPower(doubleList.get(2));
        rightBack.setPower(doubleList.get(3));

        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    public void setWheelsPowerNotEncoder(Double drive, Double strafe, Double twist) {

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


    public void driveUsingEncoderTwoMotors(Double driveDist, Double strafeDist, Double twistDist, Double driveSpeed, Double strafeSpeed, Double twistSpeed, Double timeoutS) {

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
        claw = hardwareMap.get(DcMotorEx.class, "Claw");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        claw.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotateClaw(double degrees){
        claw.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        clawTarget = (COUNTS_PER_DEGREE_CORE * degrees);

        claw.setTargetPosition((int) clawTarget);

        claw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        claw.setPower(0.5);
    }

}