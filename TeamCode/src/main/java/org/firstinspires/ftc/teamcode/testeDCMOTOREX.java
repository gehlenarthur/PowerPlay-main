package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class testeDCMOTOREX extends LinearOpMode {


      public DcMotorEx leftFront;
      public DcMotorEx rightFront;
      public DcMotorEx leftBack;
      public DcMotorEx rightBack;




    @Override
    public void runOpMode() throws InterruptedException {

    }

private void setVelocity(double angularRate){

    leftFront.setVelocity(999);

    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "FLMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "FRMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "BLMotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "BRMotor");

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