package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class reginaldoAuto extends LinearOpMode {
    DcMotorEx motorXD;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        encoder(100, 0.2, 288, 0.0, motorXD);

    }

    public void initialize() {

        motorXD = hardwareMap.get(DcMotorEx.class, "motor");
        motorXD.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorXD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorXD.setDirection(DcMotorEx.Direction.FORWARD);


    }

    public void encoder(double degrees, double power, double counts, double target, DcMotorEx motor) {

        target = degrees * counts;
        motor.setTargetPosition((int) target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

    }


}
