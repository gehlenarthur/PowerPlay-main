package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.setupImu(hardwareMap);
        robot.initBase(hardwareMap);
        robot.initMotors(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            robot.setWheelsPowerSpeedControl(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 3*560);

            if(gamepad1.right_bumper || gamepad2.right_bumper) {

            }

            if (gamepad1.left_bumper || gamepad2.left_bumper){

            }

            if(gamepad1.a || gamepad2.a){
                robot.encoderDesaceleracao(20, 5, 1680, 10, 0.08, robot.lever);
                robot.encoderDesaceleracao(8.7 * 30, 5, 384,10, 0.2, robot.linear);
                Thread.yield();


            }

            if(gamepad1.b || gamepad2.b){

            }

            if(gamepad1.y || gamepad2.y){

            }

            if(gamepad1.x || gamepad2.x){

            }

            if(gamepad1.dpad_down || gamepad2.dpad_down){

            }

            if(gamepad1.dpad_up || gamepad2.dpad_up){

            }

            if(gamepad1.dpad_right || gamepad2.dpad_right){

            }

            if(gamepad1.dpad_left || gamepad2.dpad_left){

            }

        }
    }
}
