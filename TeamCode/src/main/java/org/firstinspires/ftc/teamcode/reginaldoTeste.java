package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class reginaldoTeste extends LinearOpMode {
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        waitForStart();

        motor.setPower(gamepad1.left_stick_y);
        motor.setPower(-gamepad1.left_stick_y);

    }

}
