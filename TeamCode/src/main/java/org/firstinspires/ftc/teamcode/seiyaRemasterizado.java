package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class seiyaRemasterizado extends LinearOpMode {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor linear = null;

    int i = 0;

    public ElapsedTime runtime = null;
    private double linearDesiredPosition = 0;

    static final double REVTOTAL = 8.7 * 360;
    static final double REVMETADE = 8.7 * 180;

    @Override
    public void runOpMode() throws InterruptedException {


        initialize();
        waitForStart();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            this.setWheelsPower(drive, -strafe, twist);

            if (gamepad1.a) {
                i = 0;

            } else if (gamepad1.b) {
                i = 1;

            } else if (gamepad1.x) {
                i = 2;

            } else if (gamepad1.y) {
                i = 3;

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}



   /* private void rotateLinear (double rev){

        linearDesiredPosition = COUNTS_PER_REVOLUTION_LINEAR * rev;
        linear.setTargetPosition((int) (linearDesiredPosition));
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.25);


    }

}
    */