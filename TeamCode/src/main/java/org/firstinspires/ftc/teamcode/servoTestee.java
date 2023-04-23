package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous
public class servoTestee extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();



        robot.servoLever = hardwareMap.get(CRServo.class, "servoLever");
        robot.servoLever2 = hardwareMap.get(CRServo.class, "servoLever2");
        waitForStart();



        while (opModeIsActive()) {

            robot.servoLever.setPower(0.5);
            robot.servoLever2.setPower(0.5);
        }
    }
}
