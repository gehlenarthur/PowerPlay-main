package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "IMUrotate")
public class ImuRotate extends LinearOpMode {

    BNO055IMU imu;
    private float lastHeadingAngleImu;
    private double globalAngleImu;
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;


    @Override
    public void runOpMode() throws InterruptedException {

        setupImu();
        initialize();


        resetAngle();
        waitForStart();
        rotateUsingImu(180);


    }

    private void setupImu() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void rotateUsingImu(int degrees) {


        if (degrees < 0) {
            while ((getAngle() > degrees) || (getAngle() == 0)) {
                setWheelsPowerFromDegrees(degrees);
            }
        } else {
            while (getAngle() < degrees) {
                setWheelsPowerFromDegrees(degrees);
            }
        }

        setWheelsPower(0.00, 0.00, 0.00);
        //sleep(4000);

     /*   if (getAngle() < degrees + 1 && getAngle() > degrees - 1) {

            rotateUsingImu(degrees);

        }*/
    }

    public void resetAngle() {

        lastHeadingAngleImu = imu
                .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        globalAngleImu = 0;
    }

    private double getAngle() {

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

    private void setWheelsPowerFromDegrees(int degrees) {

        double power = 1 - (Math.abs(getAngle()) / Math.abs(degrees));

        setWheelsPower(0.00, 0.00, degrees > 0 ? 0.75 : -0.75 * 0);
        telemetry();

        velocidadeIMU(degrees,getAngle());
    }

    public void setWheelsPower(double drive, double strafe, double twist) {

        List<Double> doubleList = new ArrayList<>();
        doubleList.add((drive - strafe + twist)); //0
        doubleList.add((drive + strafe - twist)); //1
        doubleList.add((drive + strafe + twist)); //2
        doubleList.add((drive - strafe - twist));

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
        leftFront.setPower(doubleList.get(0));
        rightFront.setPower(doubleList.get(1));
        leftBack.setPower(doubleList.get(2));
        rightBack.setPower(doubleList.get(3));

    }

    public void initialize() {

        leftFront = hardwareMap.get(DcMotorEx.class, "FLMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "FRMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "BLMotor");
        rightBack = hardwareMap.get(DcMotorEx.class, "BRMotor");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void velocidadeIMU(int degrees, double anguloAtual) {

        if (anguloAtual == degrees/ 9 || anguloAtual == degrees / 1.1) {
            setWheelsPower(0,0,0.2);


        } else {

            setWheelsPower(0,0,0.7);
        }

    }

    public void telemetry() {

        telemetry.addData("angle", getAngle());
        telemetry.addData("power de um dos motores : ", leftBack.getPower());

        telemetry.update();
    }


    public double returnpower(double target,double errorInicial, DcMotor motor){

        if(target > motor.getCurrentPosition()){

            int error = (int) (target - motor.getCurrentPosition());

            return (error / target);
        }else if(target < motor.getCurrentPosition()){

            int error = (int) ( motor.getCurrentPosition() - target);

            return ( error / errorInicial);

        }
        return 1.0;
    }
}