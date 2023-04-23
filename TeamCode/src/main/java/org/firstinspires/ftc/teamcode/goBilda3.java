package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class goBilda3 extends LinearOpMode {

    double target = 0;
    ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_REVOLUTION_GOBILDa = 384.5;

    DcMotor motor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class,"motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            encoder(8.7);
            returnpower();
            telemetry();

        }
    }

    public void encoder(double rotacao){

        target = rotacao * COUNTS_PER_REVOLUTION_GOBILDa;

        motor.setTargetPosition((int)target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void returnpower() {


        int error = (int) (target - motor.getCurrentPosition());
        int powerMax = 1;

        int porcentagemPower = (int) ((error / target) * 100);

        motor.setPower(0.8);
    }


    public  void telemetry(){

        telemetry.addData("posição Atual : ",motor.getCurrentPosition());
        telemetry.addData("power : ",motor.getPower());
        telemetry.update();
    }

}
