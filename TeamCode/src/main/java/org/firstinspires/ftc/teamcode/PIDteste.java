package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE_CORE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PIDteste extends LinearOpMode {
    DcMotorEx lever;



    @Override
    public void runOpMode() throws InterruptedException {

        lever = hardwareMap.get(DcMotorEx.class, "lever");

        waitForStart();

        while (lever.getCurrentPosition()/ COUNTS_PER_DEGREE_CORE < 720 ){

            lever.setPower(0.3);
            PIDFCoefficients pid = new PIDFCoefficients(2.5,0.1,0.2,0.5);
            lever.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

        }

        lever.setPower(0);

        lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       // encoderPID(50, 5, 0.3, 288, 2, lever, 2.5, 0.1, 0.2, 0.5);



    }
    public void encoderPID(double degrees, double time, double power, double counts, double target, DcMotorEx motor, double P, double I, double D, double F) {



        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {
            java.lang.Thread.yield();
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runTime = null;
        }

    }

