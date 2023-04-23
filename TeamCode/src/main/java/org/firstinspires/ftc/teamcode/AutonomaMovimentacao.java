package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE_CORE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class AutonomaMovimentacao extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();

        robot.setupImu(hardwareMap);
        robot.initBase(hardwareMap);
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);
        robot.initWebCam(hardwareMap);

        while (!isStarted() && !isStopRequested()){

            robot.identicacaoWebCam();
        }

        //robot.setWheelsPowerSpeedControl(0.0.4,0.4,560*2);
        robot.driveUsingEncoderDesaceleracaoMetade(-2.5, 0.0, 0.0, 6.0);
        robot.rotateUsingImu(-90);
        sleep(100);
        //robot.driveUsingEncoderDesaceleracaoMetade(-12.0, 0.0, 0.0, 6.0);
        sleep(100);
        robot.encoder( 120,10,0.4,COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever);
        robot.encoderDesaceleracaoMetadePower(-60,0.085,10,COUNTS_PER_DEGREE,robot.targetLever,-0.2, robot.lever);
        robot.encoderDesaceleracaoMetadePower(-90,0.085,10,COUNTS_PER_DEGREE,robot.targetLever,0.0, robot.lever);

        sleep(300);

        robot.servoLever.setPower(-0.7);
        robot.servoLever2.setPower(0.7);
        sleep(500);
        robot.servoLever.setPower(0.0);
        robot.servoLever2.setPower(0.0);

        robot.encoder( 0,7,0.4,COUNTS_PER_DEGREE_CORE,robot.targetLinearClaw, robot.clawLinear);
        robot.encoderDesaceleracaoMetadePower(0,0.085,10,COUNTS_PER_DEGREE,robot.targetLever,-0.2, robot.lever);





        if(robot.tagOfInterest.id == robot.tag0){

            robot.driveUsingEncoderDesaceleracaoMetade(23.0, 0.0, 0.0, 6.0);
            robot.rotateUsingImu(-90);
            sleep(200);
            robot.driveUsingEncoderDesaceleracaoMetade(26.0, 0.0, 0.0, 6.0);

        }else if(robot.tagOfInterest.id == robot.tag1){


            robot.rotateUsingImu(-90);
            sleep(200);
            robot.driveUsingEncoderDesaceleracaoMetade(26.0, 0.0, 0.0, 6.0);

        }else{

            robot.driveUsingEncoderDesaceleracaoMetade(-10.0, 0.0, 0.0, 6.0);
            robot.rotateUsingImu(-90);
            sleep(200);
            robot.driveUsingEncoderDesaceleracaoMetade(26.0, 0.0, 0.0, 6.0);

        }
    }
}
