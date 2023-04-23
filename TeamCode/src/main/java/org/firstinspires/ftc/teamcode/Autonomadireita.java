package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomadireita extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();

        robot.initWebCam(hardwareMap);
        robot.identicacaoWebCam();

        waitForStart();






        if(robot.tagOfInterest.id == robot.tag0){

        }else if(robot.tagOfInterest.id == robot.tag1) {

        }else if(robot.tagOfInterest.id == robot.tag2) {

        }
    }
}

