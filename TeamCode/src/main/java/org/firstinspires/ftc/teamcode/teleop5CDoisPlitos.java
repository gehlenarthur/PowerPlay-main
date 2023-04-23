package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREES_GOBILDA;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE_CORE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_REVOLUTION_CORE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class teleop5CDoisPlitos extends LinearOpMode {

    Robot robot = new Robot();

    double linearPosition = 1;
    int servoVar = 1;
    int lado = 1;
    int passarCone = 1;
    int precionar = 1;

    public Runnable leverParaBaixo = new Runnable() {

        @Override
        public void run() {

            robot.encoder(130, 10, 0.4, COUNTS_PER_DEGREE_CORE, robot.targetLeverClaw, robot.clawLever);
            sleep(550);
            //robot.encoderDesaceleracaoSpeedControl(60,5.0,COUNTS_PER_DEGREE,robot.targetLever,0.0, robot.lever,1.66 * COUNTS_PER_REVOLUTION);
            robot.encoderSpeedControl(90, 7.0, COUNTS_PER_DEGREE, robot.targetLever, robot.lever, 1 * COUNTS_PER_REVOLUTION);

        }
    };

    public Runnable servoPrecionar = new Runnable() {
        @Override
        public void run() {

            while (precionar == 1) {

                robot.servoLever.setPower(0.7);
                robot.servoLever2.setPower(-0.7);
                sleep(50);
            }
            robot.servoLever.setPower(0.0);
            robot.servoLever2.setPower(0.0);

        }
    };

    public Runnable locomocao = new Runnable() {
        @Override
        public void run() {

            while (opModeIsActive()) {

                robot.setWheelsPowerSpeedControl(gamepad2.left_stick_y,gamepad2.left_stick_x,gamepad2.right_stick_x,2.0 * 560);
            }


        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initBase(hardwareMap);
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);

        waitForStart();

        robot.servoLinear.setPower(0.0000000000000000001);
        robot.servoLinear2.setPower(0.0000000000000000001);
        sleep(0000000000000000000001);

        java.lang.Thread tl = new java.lang.Thread(locomocao);
        tl.start();

        while (opModeIsActive()) {

            if (lado == 1) {

                if (gamepad1.left_bumper) {

                    robot.servoLever.setPower(-0.6);
                    robot.servoLever2.setPower(0.6);
                    sleep(400);
                    robot.servoLever.setPower(0.0);
                    robot.servoLever2.setPower(0.0);
                } else if (gamepad1.right_bumper) {

                    robot.servoLever.setPower(0.6);
                    robot.servoLever2.setPower(-0.6);
                    sleep(500);
                    robot.servoLever.setPower(0.0);
                    robot.servoLever2.setPower(0.0);
                }

                if (gamepad1.dpad_up) {
                    robot.encoderTeleop((robot.clawLever.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) + 7, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLeverClaw, robot.clawLever);
                }

                if (gamepad1.dpad_down) {
                    robot.encoderTeleop((robot.clawLever.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) - 7, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLeverClaw, robot.clawLever);
                }

                if (gamepad1.dpad_left) {
                    // robot.encoderSpeedControl(130, 10,  COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever, 288 * 1);
                    robot.encoderTeleopSpeedControl(130,1.0,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever, 288 * 1);
                    robot.encoderDesaceleracaoTeleOPSpeedControl(70,COUNTS_PER_DEGREE,robot.targetLever,0.0,robot.lever,COUNTS_PER_REVOLUTION * 0.15);

                }

                if (gamepad1.dpad_right) {

                    //robot.encoderDesaceleracaoSpeedControl(40, 5.0, COUNTS_PER_DEGREE, robot.targetLever, 0.3, robot.lever, 1 * COUNTS_PER_REVOLUTION);
                    robot.encoderTeleopSpeedControl(40,1.0,COUNTS_PER_DEGREE,robot.targetLever, robot.lever, 1 * COUNTS_PER_REVOLUTION);
                    //robot.encoderSpeedControl(137, 7, COUNTS_PER_DEGREE_CORE, robot.targetLinearClaw, robot.clawLinear, 1.66 * COUNTS_PER_REVOLUTION_CORE);
                    robot.encoderTeleopSpeedControl(137,1.0,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,robot.clawLinear,1.66 * COUNTS_PER_DEGREE_CORE);
                }

                if(gamepad1.b){

                    if(servoVar == 1){
                        servoVar = 2;
                    }else {
                        servoVar = 1;
                    }
                }

                if (servoVar == 1){

                    robot.servoLever.setPower(gamepad1.right_trigger * 0.7);
                    sleep(50);
                    robot.servoLever.setPower(gamepad1.left_trigger * -0.7);
                    sleep(50);
                    robot.servoLever.setPower(0.0);
                }else {

                    robot.servoLever2.setPower(gamepad1.right_trigger * -0.7);
                    sleep(50);
                    robot.servoLever2.setPower(gamepad1.left_trigger * 0.7);
                    sleep(50);
                    robot.servoLever2.setPower(0.0);
                }

                if(gamepad1.right_trigger > 0.5){
                    robot.encoderTeleopSpeedControl((robot.lever.getCurrentPosition() / COUNTS_PER_DEGREE) - 2.5,1.0,COUNTS_PER_DEGREE,robot.targetLever,robot.lever,1 * COUNTS_PER_REVOLUTION);

                }
                if(gamepad1.left_trigger > 0.5){
                    robot.encoderTeleopSpeedControl((robot.lever.getCurrentPosition() / COUNTS_PER_DEGREE) + 2.5,1.0,COUNTS_PER_DEGREE,robot.targetLever,robot.lever,1 * COUNTS_PER_REVOLUTION);

                }

                if (gamepad1.a) {

                    precionar = 1;

                    java.lang.Thread t;
                    t = new java.lang.Thread(servoPrecionar);
                    t.start();

                   /* robot.encoderSpeedControl(
                            10, 10.0, COUNTS_PER_DEGREE, robot.targetLever,
                             robot.lever, 0.25 * COUNTS_PER_REVOLUTION);
                    sleep(100);*/


                    //robot.encoderSpeedControl(180,5,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw, robot.clawLinear, 0.5 * COUNTS_PER_REVOLUTION_CORE);


                    robot.encoderDesaceleracaoSpeedControl(30 ,10.0,COUNTS_PER_DEGREE,robot.targetLever,0.5, robot.lever,0.40 * COUNTS_PER_REVOLUTION );
                    sleep(100);
                    robot.encoderSpeedControl(
                            173,5,COUNTS_PER_DEGREE_CORE,
                            robot.targetLeverClaw,robot.clawLever,2.03 * 288);

                    while(passarCone == 1){

                        if(gamepad1.right_trigger > 0.5){
                            robot.encoderTeleopSpeedControl((robot.lever.getCurrentPosition() / COUNTS_PER_DEGREE) - 2.5,1.0,COUNTS_PER_DEGREE,robot.targetLever,robot.lever,1 * COUNTS_PER_REVOLUTION);

                        }
                        if(gamepad1.left_trigger > 0.5){
                            robot.encoderTeleopSpeedControl((robot.lever.getCurrentPosition() / COUNTS_PER_DEGREE) + 2.5,1.0,COUNTS_PER_DEGREE,robot.targetLever,robot.lever,1 * COUNTS_PER_REVOLUTION);

                        }

                        if (gamepad1.dpad_up) {
                            robot.encoderTeleopSpeedControl((robot.clawLever.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) + 7, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLeverClaw, robot.clawLever,2.03 * 288);
                        }

                        if (gamepad1.dpad_down) {
                            robot.encoderTeleopSpeedControl((robot.clawLever.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) - 7, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLeverClaw, robot.clawLever,2.03 * 288);
                        }

                        if (gamepad2.dpad_up) {
                            robot.encoderTeleopSpeedControl((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) + 10, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLinearClaw, robot.clawLinear,1 * 288);
                        }

                        if (gamepad2.dpad_down) {
                            robot.encoderTeleopSpeedControl((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) - 10, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLinearClaw, robot.clawLinear,1 * 288);
                        }




                        ;

                        if(gamepad1.left_bumper){
                            passarCone = 2;
                            precionar = 2;
                            t.interrupt();
                        }
                    }

                    passarCone = 1;

                    robot.servoLinear.setPower(0.7);
                    robot.servoLinear2.setPower(-0.7);
                    sleep(250);
                    robot.servoLinear.setPower(0.0);
                    robot.servoLinear2.setPower(0.0);

                    robot.servoLever.setPower(-0.7);
                    robot.servoLever2.setPower(0.7);
                    sleep(300);
                    robot.servoLever.setPower(0.0);
                    robot.servoLever2.setPower(0.0);


                    robot.encoderDesaceleracaoSpeedControl(
                            40, 5.0, COUNTS_PER_DEGREE, robot.targetLever,
                            0.0, robot.lever, 1.33 * COUNTS_PER_REVOLUTION);

                    lado = 2;
                    robot.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.leftBack.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.rightBack.setDirection(DcMotorEx.Direction.REVERSE);

                }

            } else {

                if (gamepad1.dpad_up) {
                    robot.encoderTeleopSpeedControl((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) + 12, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLinearClaw, robot.clawLinear, 2.083 * COUNTS_PER_REVOLUTION_CORE);
                }

                if (gamepad1.dpad_down) {
                    robot.encoderTeleopSpeedControl((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE)  -12, 0.5, COUNTS_PER_DEGREE_CORE, robot.targetLinearClaw, robot.clawLinear, 2.083 * COUNTS_PER_REVOLUTION_CORE);
                }

                if (gamepad1.x) {
                    linearPosition = 1;

                } else if (gamepad1.b) {
                    linearPosition = 2;

                } else if (gamepad1.a) {
                    linearPosition = 0;
                }

                if(gamepad1.dpad_left){

                    robot.encoderTeleopSpeedControl((robot.linear.getCurrentPosition() / COUNTS_PER_DEGREES_GOBILDA) + 25,1.0,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,robot.linear,6 * 384.5);
                }

                if(gamepad1.dpad_right){

                    robot.encoderTeleopSpeedControl((robot.linear.getCurrentPosition() / COUNTS_PER_DEGREES_GOBILDA) - 25,1.0,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,robot.linear,6 * 384.5);
                }

                if (linearPosition == 1) {
                    robot.encoderSpeedControl(-4 * 360, 10,  COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,robot.linear,6 * 384.5 );

                } else if (linearPosition == 2) {
                    robot.encoderSpeedControl(-2.0 * 360, 10,  COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,robot.linear,6 * 384.5 );


                } else if (linearPosition == 0) {
                    robot.encoderDesaceleracaoMetade(0, 10, COUNTS_PER_DEGREES_GOBILDA, robot.targetLinear, 0.2, robot.linear);

                }

                if (gamepad1.left_bumper) {

                    robot.servoLinear.setPower(-0.5);
                    robot.servoLinear2.setPower(0.5);
                    sleep(350);
                    robot.servoLinear.setPower(0);
                    robot.servoLinear2.setPower(0);
                    sleep(250);


                    robot.encoderSpeedControl(145, 7, COUNTS_PER_DEGREE_CORE, robot.targetLinearClaw, robot.clawLinear, 1 * COUNTS_PER_REVOLUTION_CORE);
                    robot.encoder(0, 5, 0.4, COUNTS_PER_DEGREES_GOBILDA, robot.targetLinear, robot.linear);
                    lado = 1;
                    robot.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.rightBack.setDirection(DcMotorEx.Direction.FORWARD);
                    // deixar linear pronto para pegar o cone jogado
                } else {

                    robot.servoLinear.setPower(0.7);
                    robot.servoLinear2.setPower(-0.55);
                    sleep(100);

                }
            }
        }
    }
}