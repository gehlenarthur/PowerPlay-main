package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.goBildaMotor.COUNTS_PER_DEGREE_GOBILDA;
import static org.firstinspires.ftc.teamcode.testeMecanunsDoisEncoders.COUNTS_PER_INCH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class luni {

    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor goBildaMotor;

    ElapsedTime runTime;
    double powerLeftFront = 1;
    double powerLeftBack = 1;
    double powerRightFront = 1;
    double powerRightBack = 1;
    double powerGobilda = 1;
    double targetPositionGoBildaMotor = 0;

    public void setWheelsPower(Double drive, Double strafe, Double twist) {
        // Cria uma lista dos motores e calcula qual deve ser a velocidade de giro do motor(power)
        List<Double> doubleList = new ArrayList<>();
        doubleList.add((drive - strafe + twist)); //0
        doubleList.add((drive + strafe - twist)); //1
        doubleList.add((drive + strafe + twist)); //2
        doubleList.add((drive - strafe - twist)); //3
        // leftFront      doubleList.add((drive - + 1 + twist)); //0 -1
        //  rightFront    doubleList.add((drive + +1 - twist)); //1 -1
        // leftBack       doubleList.add((drive + +1 + twist)); //2 +1
        // rightBack      doubleList.add((drive - +1 - twist)); //3 +1
        Double max = Math.abs(doubleList.get(0));
        for (int i = 1; i < doubleList.size(); i++) {
            if (max < Math.abs(doubleList.get(i))) {
                max = Math.abs(doubleList.get(i));
            }
        }
        if (max > 1) {
            for (int i = 0; i < doubleList.size(); i++) {
                doubleList.set(i, doubleList.get(i) / max);
            }
        }
        // Pega os valores de power calculados da lista e faz com que os motores girem com o devido power
        leftFront.setPower(doubleList.get(0));
        rightFront.setPower(doubleList.get(1));
        rightBack.setPower(doubleList.get(2));
        leftBack.setPower(doubleList.get(3));

    }

    public void driveUsingEncoder(Double driveDist, Double strafeDist, Double twistDist, Double driveSpeed,
                                  Double strafeSpeed, Double twistSpeed, Double timeoutS) {

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(
                leftBack.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightBack.setTargetPosition(
                rightBack.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));
        leftFront.setTargetPosition(
                leftFront.getCurrentPosition() + (int) ((driveDist - strafeDist + twistDist)
                        * COUNTS_PER_INCH));
        rightFront.setTargetPosition(
                rightFront.getCurrentPosition() + (int) ((driveDist + strafeDist - twistDist)
                        * COUNTS_PER_INCH));

        setWheelsMode(DcMotor.RunMode.RUN_TO_POSITION);

        setWheelsPower(Math.abs(driveSpeed), Math.abs(strafeSpeed), Math.abs(twistSpeed));

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy()
                && rightBack.isBusy())) {

            Thread.yield();
        }

        setWheelsPower(0.00, 0.00, 0.00);

        setWheelsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    private void setWheelsMode(DcMotor.RunMode runMode) {

        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
    }

    public void gobildaMotorEncoderDesaceleracao(double deegres, double time){

        targetPositionGoBildaMotor = deegres * COUNTS_PER_DEGREE_GOBILDA;
        goBildaMotor.setTargetPosition((int)targetPositionGoBildaMotor);
        goBildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime = new ElapsedTime();
        while (runTime.seconds() < time && goBildaMotor.isBusy()) {

            powerGobilda = returnpower(targetPositionGoBildaMotor,goBildaMotor);
            goBildaMotor.setPower(powerGobilda);
            telemetry();
        }

        runTime = null;
    }

    public void gobildaMotorEncoderDesaceleracaoMetade(double deegres, double time){

        targetPositionGoBildaMotor = deegres * COUNTS_PER_DEGREE_GOBILDA;
        goBildaMotor.setTargetPosition((int)targetPositionGoBildaMotor);
        goBildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime = new ElapsedTime();
        while (runTime.seconds() < time && goBildaMotor.isBusy()) {

            powerGobilda = returnpower(targetPositionGoBildaMotor,goBildaMotor);
            goBildaMotor.setPower(powerGobilda);
            telemetry();
        }

        runTime = null;
    }

    public void gobildaMotorEncoder(double deegres, double time,double power){



        targetPositionGoBildaMotor = deegres * COUNTS_PER_DEGREE_GOBILDA;
        goBildaMotor.setTargetPosition((int)targetPositionGoBildaMotor);
        goBildaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTime = new ElapsedTime();
        while (runTime.seconds() < time && goBildaMotor.isBusy()) {

            power = returnpower(targetPositionGoBildaMotor,goBildaMotor);
            goBildaMotor.setPower(power);
            telemetry();
        }

        runTime.reset();
    }

    public double returnPowerBase(double targerPosition){

        powerLeftBack =  returnpower(targerPosition,leftBack);
        powerLeftFront = returnpower(targerPosition,leftFront);
        powerRightBack = returnpower(targerPosition,rightBack);
        powerRightFront = returnpower(targerPosition,rightFront);

        return powerRightFront + powerRightBack + powerLeftFront + powerLeftBack / 4;
    }

    public double returnPowerBaseMetade(double targerPosition){

        powerLeftBack =  returnpowerMetade(targerPosition,leftBack);
        powerLeftFront = returnpowerMetade(targerPosition,leftFront);
        powerRightBack = returnpowerMetade(targerPosition,rightBack);
        powerRightFront = returnpowerMetade(targerPosition,rightFront);

        return powerRightFront + powerRightBack + powerLeftFront + powerLeftBack / 4;
    }

    public double returnpower(double targetPosition, DcMotor motor){

        int error = (int) (targetPosition - motor.getCurrentPosition());
        return  (error / targetPosition);

    }

    public double returnpowerMetade(double targetPosition, DcMotor motor){

        if(motor.getCurrentPosition() > targetPosition/2){

            targetPosition /= 2;

            int error = (int) (targetPosition - (motor.getCurrentPosition() - targetPosition));
            return  ((error / targetPosition));
        }

        return  1.0;
    }

    public  void telemetry(){

        telemetry.addData("posição Atual : ",goBildaMotor.getCurrentPosition());
        telemetry.addData("power : ",goBildaMotor.getPower());
        telemetry.addData("target : ",goBildaMotor.getTargetPosition());
        telemetry.update();
    }
}