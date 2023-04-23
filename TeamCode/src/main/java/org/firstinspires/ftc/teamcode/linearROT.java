package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class linearROT extends LinearOpMode {

    public DcMotor linear = null;
    private double linearDesiredPosition = 0;

    static final double REVTOTAL = 8.7 * 360;
    static final double REVMETADE = 8.7 * 180;

    static final double rotacaoTotal = 360;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
       initialize();
       waitForStart();





        rotateLinear(8.7);




    }



    private void rotateLinear (double rev){

        linearDesiredPosition = rotacaoTotal * rev;
        linear.setTargetPosition((int) (linearDesiredPosition));
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(0.25);




    }

    public void initialize(){

       linear = hardwareMap.get(DcMotor.class, "linear");
       linear.setDirection(DcMotor.Direction.FORWARD);
    }
}
