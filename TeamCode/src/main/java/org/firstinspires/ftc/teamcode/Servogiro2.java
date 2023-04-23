package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous

public class Servogiro2 extends LinearOpMode {

    Servo servo;
    double targetPosition = 1.0;



    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");


        waitForStart();
        setServoPosition(0.5);
        
    }

    private void setServoPosition(double degrees){
        servo.setPosition(degrees);
        idle();
    }
    private void initialize(){
    servo = hardwareMap.get(Servo.class, "servo");
    }
}





