package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class servoGiro extends LinearOpMode {

    Servo servo = null;
    double servoPosition = 1.0;

    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    ElapsedTime runtime = new ElapsedTime();

    public void initialize() {
        servo = hardwareMap.servo.get("servo");
        servo.setDirection(Servo.Direction.FORWARD);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        servo.setPosition(servoPosition);


        waitForStart();

            servoPosition = 0.3;
            servo.setPosition(servoPosition);

    }
}



