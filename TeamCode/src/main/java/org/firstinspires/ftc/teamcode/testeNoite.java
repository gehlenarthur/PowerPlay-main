package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.testeMecanunsDoisEncoders.COUNTS_PER_INCH;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class testeNoite extends LinearOpMode {

        public BNO055IMU imu;
        public float lastHeadingAngleImu = 0;
        public double globalAngleImu = 0;

        public DcMotorEx leftFront = null;
        public DcMotorEx leftBack = null;
        public DcMotorEx rightFront = null;
        public DcMotorEx rightBack = null;

        double errorInicialLeftBack = 0;
        double errorInicialLeftFront = 0;
        double errorInicialRightBack = 0;
        double errorInicialRightFront = 0;




        @Override
        public void runOpMode() throws InterruptedException {

           Robot robot = new Robot();

           robot.initBase(hardwareMap);
           setupImu(hardwareMap);
           waitForStart();


           rotateUsingImu(40);


        }

        public double getAngle() {

            float headingAngleImu = imu
                    .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double deltaAngle = headingAngleImu - lastHeadingAngleImu;

            if (deltaAngle < -180) {
                deltaAngle += 360;
            } else if (deltaAngle > 180) {
                deltaAngle -= 360;
            }

            globalAngleImu += deltaAngle;
            lastHeadingAngleImu = headingAngleImu;

            return globalAngleImu;
        }

        public void setWheelsPowerSpeedControl(double drive, double strafe, double twist, double speed) {
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
            leftFront.setVelocity(doubleList.get(0) * speed);
            rightFront.setVelocity(doubleList.get(1) * speed);
            rightBack.setVelocity(doubleList.get(2) * speed);
            leftBack.setVelocity(doubleList.get(3) * speed);

        }

    public double returnpowerImu(double degrees, double errorInicial) {

        if (degrees > getAngle()) {

            int error = (int) (degrees - getAngle());

            return (error / degrees);
        } else if (degrees < getAngle()) {

            int error = (int) (getAngle() - degrees);

            return (error / errorInicial);

        }
        return 0.0;
    }

    public void setupImu(HardwareMap hMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.imu = hMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }

    public void rotateUsingImu(int degrees) {

        resetAngle();

        double errorInicial = getAngle() - degrees;

        if (degrees < 0) {
            while ((getAngle() >= degrees) || (getAngle() == 0)) {
                setWheelsPowerFromDegrees(degrees, errorInicial);
            }
        } else {
            while (getAngle() <= degrees) {
                setWheelsPowerFromDegrees(degrees, errorInicial);
            }
        }

        setWheelsPower(0.5, 0.5, 0.5);
    }

    public void resetAngle() {


        lastHeadingAngleImu = imu
                .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        globalAngleImu = 0;
    }


    public double checkDirection() {

        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0) {
            correction = 0;
        } else {
            correction = -angle;
        }

        correction = correction * gain;

        return correction;
    }

    private void setWheelsPowerFromDegrees(int degrees, double errorInicial) {

        // double power = 1 - (Math.abs(getAngle()) / Math.abs(degrees));

        setWheelsPowerSpeedControl(0.00, 0.00, degrees > 0 ? -(returnpowerImu(degrees, errorInicial)) : returnpowerImu(degrees, errorInicial), 3 * 560);
    }

    public void setWheelsPower(double drive, double strafe, double twist) {
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



}
