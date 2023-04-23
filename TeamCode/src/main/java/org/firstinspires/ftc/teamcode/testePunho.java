package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class testePunho extends LinearOpMode {
    static final double COUNTS_PER_REVOLUTION = 1680;
    static final double DRIVE_GEAR_REDUCTION_ULTRA = 2;
    static final double COUNTS_PER_DEGREE_ULTRA = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION_ULTRA) / 360;

    public static final double COUNTS_PER_MOTOR_CORE = 288;
    public static final double COUNTS_PER_MOTOR_REV = 1120;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.0;

    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public  static  final double COUNTS_PER_DEGREE_CORE = (COUNTS_PER_MOTOR_CORE) / 360;

    DcMotorEx claw = null;

    public double clawTarget = 0.0;

    public void rotateClaw(double degrees){
        claw.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        clawTarget = (COUNTS_PER_DEGREE_CORE * degrees);

        claw.setTargetPosition((int) clawTarget);

        claw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        claw.setPower(0.5);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(DcMotorEx.class, "claw");
        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.x){
                rotateClaw(30);
            } else if(gamepad1.y){
                rotateClaw(60);
            } else if (gamepad1.a){
                rotateClaw(90);
            } else  if (gamepad1.b){
                rotateClaw(0);
            }

        }

    }
}
