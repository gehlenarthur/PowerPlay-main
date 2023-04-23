package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class Aceleracao extends LinearOpMode {

    public DcMotorEx leftFront;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        leftFront = hardwareMap.get(DcMotorEx.class, "leftfront");
        waitForStart();

        encoderAceleracao(90, 10, 288,
                robot.targetLeverClaw, 0.3, leftFront);
        calculateTargetVelocity(0, 0, leftFront);
        returnPowerAceleracao(0, 0, leftFront);


    }

    public void encoderAceleracao(double degrees, double time, double counts, double target, double adicional, DcMotorEx motor) {
        target = degrees * counts;
        motor.setTargetPosition((int) target);
        double errorInicial = motor.getCurrentPosition() - target;
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double power = 0.1;
        motor.setPower(power);

        ElapsedTime runTime = new ElapsedTime();
        while (runTime.seconds() < time && motor.isBusy()) {

            power += 0.2  ;
            power = Math.min(power, adicional);
            motor.setPower(power);
            sleep(100);
            double targetVelocity = returnPowerAceleracao(target, errorInicial, motor);
            motor.setVelocity(targetVelocity);
        }

        runTime = null;
    }

    private double returnPowerAceleracao(double target, double errorInicial, DcMotorEx motor) {
        double currentVelocity = motor.getVelocity();
        double targetVelocity = calculateTargetVelocity(target, errorInicial, motor);
        double error = targetVelocity - currentVelocity;
        double Kp = 1;
        double power = error * Kp;
        return power;
    }

    private double calculateTargetVelocity(double target, double errorInicial, DcMotorEx motor) {
        double targetVelocity = 0.0;

        ElapsedTime runTime = new ElapsedTime();
        double errorDistance = (motor.getCurrentPosition() - target);
        double time = runTime.seconds();
        if (time > 0) {
            targetVelocity = errorDistance / time;
        }
        telemetry.addData("leftFront", motor.getPower());
        telemetry.update();
        return targetVelocity;
    }

}
