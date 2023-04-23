
/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Aceleracao extends LinearOpMode {

    DcMotor motor;

    Servo servo;

    double targetPosition = 0;
    int  porcentagemPower = 0;

    ElapsedTime runtime;

    double power = 1;

    static final double COUNTS_PER_REVOLUTION = 288;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) / 360;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class,"motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoder(1440);

    }

    public void encoder(double degrees){

        targetPosition = (degrees * COUNTS_PER_DEGREE);
        motor.setTargetPosition((int)degrees);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!(motor.getCurrentPosition() > targetPosition -COUNTS_PER_DEGREE && motor.getCurrentPosition() < targetPosition +COUNTS_PER_DEGREE)) {


            returnPower();
        }

        runtime = new ElapsedTime();
        while ((runtime.seconds() < 10) && motor.isBusy()) {

            telemetry();
            Thread.yield();

        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public  void telemetry(){

        telemetry.addData("posição Atual : ",motor.getCurrentPosition()/ COUNTS_PER_DEGREE);
        telemetry.addData("power : ",motor.getPower());
        telemetry.addData("% : ",porcentagemPower);
        telemetry.update();
    }

    public void returnPower(){
        //  if(targetPosition > motor.getCurrentPosition()) {


        double error = (targetPosition - motor.getCurrentPosition());
        double powerMax = 1;

        porcentagemPower = (int) (100 * (error / targetPosition));

        power =  (powerMax * porcentagemPower) / 100;

        motor.setPower(power );

    }/*else {

        double error = (motor.getCurrentPosition() - targetPosition);
        double powerMax = -1;

        porcentagemPower = (int) (100 * (error / targetPosition));

        power = -((powerMax * porcentagemPower) / 100);
    }
}
}
*/
