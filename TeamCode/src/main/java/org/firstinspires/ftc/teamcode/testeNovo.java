package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class testeNovo extends LinearOpMode {

    DcMotorEx motor1;
    DcMotorEx motor2;
    DcMotorEx motor3;
    DcMotorEx motor4;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.left_stick_y);
        motor3.setPower(gamepad1.left_stick_y);
        motor4.setPower(gamepad1.left_stick_y);



    }

    private void initizalie(){
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor1");

        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        motor3.setDirection(DcMotorEx.Direction.REVERSE);
        motor4.setDirection(DcMotorEx.Direction.FORWARD);





    }


}
