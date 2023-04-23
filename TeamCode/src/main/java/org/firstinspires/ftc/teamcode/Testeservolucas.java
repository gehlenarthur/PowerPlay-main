package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class Testeservolucas extends LinearOpMode {

    Servo servo;
    Servo servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();

        servo = hardwareMap.get(Servo.class, "servo");
        //servo2 = hardwareMap.get(Servo.class, "servo2");

        //robot.MoveServos(servo, servo2,0.0,0.0);
        //robot.MoveServo(servo, 0.45);//servo fechado da direita, 0.35 aberto
         //um sera o inverso do outro

        waitForStart();
       /* robot.MoveServos(servo, servo2, 0.3, 0.3);
        sleep(1000);
        robot.MoveServos(servo, 0.5);
        sleep(1000);
        robot.MoveServos(servo, 1.0);
                    sleep(1000);*/

    }
}
