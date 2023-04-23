package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_DEGREE_CORE;
import static org.firstinspires.ftc.teamcode.Variables.COUNTS_PER_REVOLUTION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "autoLeftCone")
public class autoLeft extends LinearOpMode {

    Robot robot = new Robot();

    public Runnable servoPrecionar = new Runnable() {
        @Override
        public void run() {
            ElapsedTime runTime = new ElapsedTime();
            while (runTime.seconds() < 1) {

                robot.servoLever.setPower(0.7);
                robot.servoLever2.setPower(-0.7);
                sleep(50);
            }
            robot.servoLever.setPower(0.0);
            robot.servoLever2.setPower(0.0);

            runTime = null;
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        robot.setupImu(hardwareMap);
        robot.initBase(hardwareMap);
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);
        robot.initWebCam(hardwareMap);

        while (!isStarted() && !isStopRequested()){

            robot.identicacaoWebCam();
        }

        //robot.setWheelsPowerSpeedControl(0.0.4,0.4,560*2);

        robot.camera.stopRecordingPipeline();
        robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(-6.0, 0.0, 0.0, 6.0,2.5 * 560);
        robot.rotateUsingImu(90);
        sleep(100);
        //robot.driveUsingEncoderDesaceleracaoMetade(-12.0, 0.0, 0.0, 6.0);


        java.lang.Thread t = new java.lang.Thread(servoPrecionar);
        t.start();
        robot.encoderSpeedControl(93, 10,  COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever, 288 * 1.66);
        robot.encoderDesaceleracaoSpeedControl(70,7,COUNTS_PER_DEGREE,robot.targetLever,0.0,robot.lever,COUNTS_PER_REVOLUTION * 0.25);

        robot.servoLever.setPower(-0.7);
        robot.servoLever2.setPower(0.7);
        sleep(500);
        robot.servoLever.setPower(0.0);
        robot.servoLever2.setPower(0.0);

        //  robot.encoderSpeedControl(75, 7.0, COUNTS_PER_DEGREE, robot.targetLever, robot.lever, 0.5 * COUNTS_PER_REVOLUTION);
        robot.encoderDesaceleracaoSpeedControl(0,5,COUNTS_PER_DEGREE,robot.targetLever,0.0,robot.lever,COUNTS_PER_REVOLUTION * 0.5);
        robot.encoderSpeedControl(0, 5,  COUNTS_PER_DEGREE_CORE,robot.targetLeverClaw, robot.clawLever, 288 * 2.03);

        if(robot.tagOfInterest.id == robot.tag0){

            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(25.0, 0.0, 0.0, 4.0,2.5 *  560);
            robot.rotateUsingImu(-90);
            //sleep(100);
            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(-25.0, 0.0, 0.0, 4.0,2.5 *  560);

        }else if(robot.tagOfInterest.id == robot.tag1){

            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(5.0, 0.0, 0.0, 4.0,2.5  *  560);
            robot.rotateUsingImu(-90);
            //sleep(100);
            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(-25.0, 0.0, 0.0, 4.0,2.5  *  560);

        }else if(robot.tagOfInterest.id == robot.tag2){

            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(5.0, 0.0, 0.0, 4.0,2.5  *  560);
            robot.rotateUsingImu(-90);
            //sleep(100);
            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(-24.0, 0.0, 0.0, 4.0,2.5  *  560);
            //sleep(100);
            robot.rotateUsingImu(87);
            robot.driveUsingEncoderDesaceleracaoMetadeSpeedContorl(-19.5, 0.0, 0.0, 4.0,2.5  *  560);

        }else {

            robot.setWheelsPower(0,0,0);
        }
    }
}