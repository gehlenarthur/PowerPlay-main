package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREES_GOBILDA;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE_CORE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_REVOLUTION_GOBILDa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class teleop2 extends LinearOpMode {

    Robot robot = new Robot();

    static final double PositionOpenServoLever = 0.35;
    static final double PositionOpenServoLever2 = 0.45;
    static final double PositionCloseServoLever = 0.53;
    static final double PositionCloseServoLever2 = 0.35;

    static final double PositionAbleServoLinear = 0.4;
    static final double PositionAbleServoLinear2 = -0.4;
    static final double PositionCloseServoLinear = -0.1;
    static final double PositionCloseServoLinear2 = 0.0;

    double linearPosition = 2;


    int lado = 1;

    public  Runnable leverParaBaixo = new Runnable() {
        public void run() {

            robot.encoder( 120,10,0.4,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever);
            sleep(550);
            robot.encoderDesaceleracaoMetadePower(-60,0.115,10,COUNTS_PER_DEGREE,robot.targetLever,-0.1, robot.lever);
            robot.encoderDesaceleracaoMetadePower(-90,0.115,10,COUNTS_PER_DEGREE,robot.targetLever,0.0, robot.lever);

        }
    };

    public  Runnable servoPrecionar = new Runnable() {
        public void run() {
            ElapsedTime runTime = new ElapsedTime();

            while (runTime.seconds() < 2) {

                robot.servoLever.setPower(0.7);
                robot.servoLever2.setPower(-0.7);
            }
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {



        Variables variables = new Variables();
        robot.initBase(hardwareMap);
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);


      //  robot.encoder( +120,7,0.4,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever);


        robot.encoderDesaceleracaoMetadePower(-40,0.085,10,COUNTS_PER_DEGREE,robot.targetLever,-0.2, robot.lever);


       // robot.encoderDesaceleracaoMetadePower(-90,0.085,10,COUNTS_PER_DEGREE,robot.targetLever,0.0, robot.lever);
        robot.encoder( +45+5,7,0.4,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw, robot.clawLinear);
        robot.encoder(-1.24 * 360,5,0.4,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear, robot.linear);


        waitForStart();

        while (opModeIsActive()){

            if(lado == 1){

                robot.setWheelsPowerSpeedControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,3*560);

                if(gamepad1.left_bumper){

                    //robot.MoveServos(robot.servoLever, robot.servoLever2,PositionOpenServoLever,PositionOpenServoLever2 );
                    //robot.MoveServos(robot.servoLever, robot.servoLever2,robot.servoLever.getPosition() - 0.1,robot.servoLever.getPosition() +0.4);
                    robot.servoLever.setPower(-0.7);
                    robot.servoLever2.setPower(0.7);
                    sleep(400);
                    robot.servoLever.setPower(0.0);
                    robot.servoLever2.setPower(0.0);
                }

                if(gamepad1.right_bumper){

                    //robot.MoveServos(robot.servoLever, robot.servoLever2,robot.servoLever.getPosition()  + 0.1,robot.servoLever.getPosition() - 0.4);

                    //robot.MoveServos(robot.servoLever, robot.servoLever2,PositionCloseServoLever,PositionCloseServoLever2 );
                    robot.servoLever.setPower(0.7);
                    robot.servoLever2.setPower(-0.7);
                    sleep(500);
                    robot.servoLever.setPower(0.0);
                    robot.servoLever2.setPower(0.0);
                }

                if(gamepad1.dpad_up){
                    robot.encoderTeleop((robot.clawLever.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) +7,0.5,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw,robot.clawLever);
                }

                if(gamepad1.dpad_down){
                    robot.encoderTeleop((robot.clawLever.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) -7,0.5,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw,robot.clawLever);
                }

                if (gamepad1.dpad_right){
                    new java.lang.Thread(leverParaBaixo).start();

                }

                if(gamepad1.a){

                    new java.lang.Thread(servoPrecionar).start();

                    //robot.encoderDesaceleracaoLever(-20,5,COUNTS_PER_DEGREE,robot.targetLever,-0.075, robot.lever);
                    robot.encoder(-15,10,0.20,COUNTS_PER_DEGREE,robot.targetLever, robot.lever);
                    sleep(100);
                    robot.encoder(210,7,0.4,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever);

                    //19
                    sleep(2500);

                    robot.servoLinear.setPower(0.7);
                    robot.servoLinear2.setPower(-0.7);
                    sleep(200);
                    robot.servoLinear.setPower(0.0);
                    robot.servoLinear2.setPower(0.0);

                    robot.servoLever.setPower(-0.7);
                    robot.servoLever2.setPower(0.7);
                    sleep(300);
                    robot.servoLever.setPower(0.0);
                    robot.servoLever2.setPower(0.0);
                    //sleep(10);

                    robot.encoderDesaceleracaoMetadePower(-40,0.085,10,COUNTS_PER_DEGREE,robot.targetLever,-0.2, robot.lever);

                    sleep(1000);

                    //  robot.encoderDesaceleracao(230,8,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,0.0, robot.clawLinear);

                    //        robot.encoderDesaceleracao((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) + 165,10,0.1,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw, robot.clawLinear);
                    //robot.encoderDesaceleracao((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) + 165,10,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,0.3,robot.clawLinear);


                    //codico de move cone para outro lado
                    lado = 2;
                    robot.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.leftBack.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.rightBack.setDirection(DcMotorEx.Direction.REVERSE);

                }

            }else {

                robot.setWheelsPowerSpeedControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,3*560);
               /* if(gamepad1.dpad_up){
                    robot.encoderTeleop(4,0.2,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,robot.clawLinear);
                }


                if(gamepad1.dpad_down){
                    robot.encoderTeleop(-4,0.2,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,robot.clawLinear);
                }*/
                if(gamepad1.dpad_up){
                    robot.encoderTeleop((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) +7,0.5,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,robot.clawLinear);
                }

                if(gamepad1.dpad_down){
                    robot.encoderTeleop((robot.clawLinear.getCurrentPosition() / COUNTS_PER_DEGREE_CORE) -7,0.5,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw,robot.clawLinear);
                }

                if(gamepad1.x){
                    linearPosition = 1;

                }else if(gamepad1.b){
                   linearPosition = 2;

                }else if(gamepad1.a){
                    linearPosition = 0;
                }

                if(linearPosition == 1){
                    robot.encoder(-2.3 * 360,10,0.4,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,robot.linear);

                }else if(linearPosition == 2){
                    robot.encoder(-3.5 * 360,10,0.4,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,robot.linear);

                }else if(linearPosition == 0){
                    robot.encoderDesaceleracaoMetade(0,10,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear,0.2,robot.linear);

                }

                if(gamepad1.left_bumper){

                    //robot.MoveServos(robot.servoLinear, robot.servoLinear2,PositionAbleServoLinear,PositionAbleServoLLienar2 );


                    //robot.MoveServos(robot.servoLever, robot.servoLever2,robot.servoLever.getPosition() -0.4,robot.servoLever.getPosition() + 0.4);
                    robot.servoLinear.setPower(-0.7);
                    robot.servoLinear2.setPower(0.7);
                    sleep(150);
                    robot.servoLinear.setPower(0);
                    robot.servoLinear2.setPower(0);
                    sleep(250);


                    robot.encoder( +45+5,7,0.2,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw, robot.clawLinear);
                    robot.encoder(-1.24 * 360,5,0.4,COUNTS_PER_DEGREES_GOBILDA,robot.targetLinear, robot.linear);
                    lado = 1;
                    robot.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
                    robot.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
                    robot.rightBack.setDirection(DcMotorEx.Direction.FORWARD);
                    // deixar linear pronto para pegar o cone jogado
                }else {

                    robot.servoLinear.setPower(0.7);
                    robot.servoLinear2.setPower(-0.7);


                }
            }

        }
    }
}