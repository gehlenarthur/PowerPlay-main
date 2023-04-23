package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.testePunho.COUNTS_PER_DEGREE_CORE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "SeyiaTOnotDuck3")
public class seyianotduck3 extends LinearOpMode {

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor ArmMotor = null;
    public  DcMotor claw = null;
    public CRServo grappleServo = null;
    public CRServo grappleServo1 = null;

    int i = 3;
    double clawTarget = 0;

    public ElapsedTime runtime = null;
    private double armMotorDesiredPosition = 0;

    static final double COUNTS_PER_REVOLUTION = 1680;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / 360;

    private double shoulderDesiredPosition = 0;

    static final double COUNTS_PER_REVOLUTION_PULSE = 288;
    static final double COUNTS_PER_DEGREE_PULSE = (COUNTS_PER_REVOLUTION_PULSE) / 360;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();



        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            this.setWheelsPower(drive, -strafe, -twist);

            double diffM1 = ArmMotor.getCurrentPosition() - armMotorDesiredPosition;
            double diffM2 = claw.getCurrentPosition() - clawTarget;



        /*    if ((armMotorDesiredPosition > ArmMotor.getCurrentPosition() ? diffM1 >= -1.0 : diffM1 <= 1.0
                  )
            ) {*/

                if (gamepad1.a) {
                    i = 0;

                } else if (gamepad1.b) {
                    i = 1;

                } else if (gamepad1.x) {
                    i = 2;

                } else if (gamepad1.y) {
                    i = 3;

                }

           // }



                if (i == 0) {

                    grappleServo.setPower(0);
                    grappleServo1.setPower(0);
                    rotateArm(105);
                    rotateClaw(-50+30);

                } else if (i == 1) {

                    grappleServo.setPower(0);
                    grappleServo1.setPower(0);
                    rotateArm(69);
                    rotateClaw(-65+30);
                } else if (i == 2) {

                    grappleServo.setPower(0);
                    grappleServo1.setPower(0);
                    rotateArm(18);
                    rotateClaw(-17+30);
                } else if (i == 3) {

                    grappleServo.setPower(1);
                    grappleServo1.setPower(-1);
                    rotateClaw(30);
                    rotateArm(0);

                }

                if (gamepad1.right_bumper) {
                    setWheelsPower(0.0, 0.0, 0.0);
                    grappleServo.setPower(-1);
                    grappleServo1.setPower(1);
                    sleep(200);
                }

                if (gamepad1.left_bumper) {
                    setWheelsPower(0.0, 0.0, 0.0);
                    grappleServo.setPower(1);
                    grappleServo1.setPower(-1);
                    sleep(200);
                }




            telemetryAddData();
        }
    }
    public void setWheelsPower(Double drive, Double strafe, Double twist) {
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
    public void initialize() {
        leftFront = hardwareMap.get(DcMotor.class, "FLMotor");
        rightFront = hardwareMap.get(DcMotor.class, "FRMotor");
        leftBack = hardwareMap.get(DcMotor.class, "BLMotor");
        rightBack = hardwareMap.get(DcMotor.class, "BRMotor");
        claw = hardwareMap.get(DcMotor.class, "pulse");
        grappleServo = hardwareMap.get(CRServo.class, "grapple");
        grappleServo1 = hardwareMap.get(CRServo.class, "grapple1");
        ArmMotor = hardwareMap.get(DcMotor.class, "armMotor1");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setArmMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setArmMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
    public void rotateClaw(double degrees){

        clawTarget = (COUNTS_PER_DEGREE_CORE * degrees);

        claw.setTargetPosition((int) clawTarget);

        claw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        claw.setPower(0.25);

    }

    public void telemetryAddData() {
        telemetry.addData("Time in Mili", "%.2f", time);
        telemetry.addData("Desired position motor 1", "%.2f", armMotorDesiredPosition);
        telemetry.addData("Current position motor 1", "%d", ArmMotor.getCurrentPosition());
        telemetry.addData(">", "Press Stop to end test.");
        telemetry.update();
    }
    private void setArmMotorsMode(DcMotor.RunMode runModeArm) {

        ArmMotor.setMode(runModeArm);
    }

    private void rotateArm(double degrees) {

        armMotorDesiredPosition = COUNTS_PER_DEGREE * degrees;
        ArmMotor.setTargetPosition((int) (armMotorDesiredPosition));
        setArmMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmMotorsPower(ArmMotor.getTargetPosition() > ArmMotor.getCurrentPosition() ? 0.5 : -0.5);
    }

    private void setArmMotorsPower(double powerArm) {


        ArmMotor.setPower(powerArm);
    }
}