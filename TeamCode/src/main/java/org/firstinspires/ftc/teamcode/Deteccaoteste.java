package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Deteccaoteste extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();

        robot.initWebCam(hardwareMap);
        robot.identicacaoWebCam();

        if(robot.tagOfInterest.id == robot.tag0){
            telemetry.addLine("taag 6 identificada");
            telemetry.update();
        }

        if(robot.tagOfInterest.id == robot.tag1){
            telemetry.addLine("taag 9 identificada");
            telemetry.update();
        }

        if(robot.tagOfInterest.id == robot.tag2){
            telemetry.addLine("taag 12 identificada");
            telemetry.update();
        }


    }
}
