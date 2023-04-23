package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.nio.file.DirectoryNotEmptyException;
import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.onbotjava.OnBotJavaManager.initialize;
@TeleOp(name = "SeyiaTO")
public class SeyiaTO extends LinearOpMode {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor duck = null;
    public DcMotor shoulder = null;
    public DcMotor ArmMotor = null;
    public TouchSensor touch;
    public CRServo grappleServo = null;
    public CRServo grappleServo1 = null;
    int i = 3;
    public ElapsedTime runtime = null;
    private double armMotorDesiredPosition = 0;
    static final double COUNTS_PER_REVOLUTION = 1680;
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / 360;
    private double shoulderDesiredPosition = 0;
    static final double COUNTS_PER_REVOLUTION_SHOULDER = 288;
    static final double DRIVE_GEAR_REDUCTION_SHOULDER = 2.083;
    static final double COUNTS_PER_DEGRE_SHOULDER = (COUNTS_PER_REVOLUTION_SHOULDER * DRIVE_GEAR_REDUCTION_SHOULDER) / 360;
    public final void sleep(double milliseconds) {
        try {
            Thread.sleep((long) milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            this.setWheelsPower(drive, -strafe, -twist);
            telemetryAddData();
            if (gamepad1.b) {
                duck.setPower(-1);
            }
            if (gamepad1.b = false) {
                duck.setPower(0);
            }
            if (gamepad2.a) {
                i = 0;
                grappleServo.setPower(0);
                grappleServo1.setPower(0);
                rotateArm(-93);
            } else if (gamepad2.b) {
                i = 1;
                grappleServo.setPower(0);
                grappleServo1.setPower(0);
                rotateArm(-57);
            } else if (gamepad2.x) {
                i = 2;
                grappleServo.setPower(0);
                grappleServo1.setPower(0);
                rotateArm(-30);
            } else if (gamepad2.y) {
                i = 3;
                grappleServo.setPower(1);
                grappleServo1.setPower(-1);
                rotateArm(0);
            } else if (gamepad2.left_bumper) {
                grappleServo.setPower(0);
                grappleServo1.setPower(0);
                rotateArm(-41);
            }
            if (gamepad2.right_bumper) {
                grappleServo.setPower(-1);
                grappleServo1.setPower(1);
                sleep(2500);
            }
            double diffM1 = ArmMotor.getCurrentPosition() - armMotorDesiredPosition;
            if ((armMotorDesiredPosition > ArmMotor.getCurrentPosition()
                    ? diffM1 >= -1.0
                    : diffM1 <= 1.0)) {
                if (touch.isPressed()) {
                    if (gamepad2.right_bumper) {
                        grappleServo.setPower(-1);
                        grappleServo1.setPower(1);
                        sleep(2500);
                    }
                } else if (i == 0) {
                    grappleServo.setPower(0);
                    grappleServo1.setPower(0);
                    rotateArm(-93);
                } else if (i == 1) {
                    grappleServo.setPower(0);
                    grappleServo1.setPower(0);
                    rotateArm(-57);
                } else if (i == 2) {
                    grappleServo.setPower(0);
                    grappleServo1.setPower(0);
                    rotateArm(-30);
                } else if (i == 3) {
                    grappleServo.setPower(1);
                    grappleServo1.setPower(-1);
                    rotateArm(0);
                }
            }
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
        shoulder.setPower(gamepad2.left_stick_x);
    }
    public void initialize() {
        leftFront = hardwareMap.get(DcMotor.class, "FLMotor");
        rightFront = hardwareMap.get(DcMotor.class, "FRMotor");
        leftBack = hardwareMap.get(DcMotor.class, "BLMotor");
        rightBack = hardwareMap.get(DcMotor.class, "BRMotor");
        duck = hardwareMap.get(DcMotor.class, "duck");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grappleServo = hardwareMap.get(CRServo.class, "grapple");
        grappleServo1 = hardwareMap.get(CRServo.class, "grapple1");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        ArmMotor = hardwareMap.get(DcMotor.class, "armMotor1");
        setArmMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setShoulderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setShoulderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setShoulderMode(DcMotor.RunMode runMode) {
        shoulder.setMode(runMode);
    }
    public void telemetryAddData() {
        telemetry.addData("Time in Mili", "%.2f", time);
        telemetry.addData("Desired position motor 1", "%.2f", armMotorDesiredPosition);
        telemetry.addData("Current position motor 1", "%d", ArmMotor.getCurrentPosition());
        telemetry.addData("Touch", touch.isPressed());
        telemetry.addData(">", "Press Stop to end test.");
        telemetry.addData("powerDuck", duck.getPower());
        telemetry.update();
    }
    private void setArmMotorsMode(DcMotor.RunMode runModeArm) {
        ArmMotor.setMode(runModeArm);
    }
    private void rotateArm(double degrees) {
        ArmMotor.setTargetPosition((int) (COUNTS_PER_DEGREE * degrees));
        setArmMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmMotorsPower(0.5);
    }
    private void setArmMotorsPower(double powerArm) {
        ArmMotor.setPower(powerArm);
    }
}