package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public abstract class Robot extends LinearOpMode {
    protected static final int LED_CHANNEL = 5;
    protected DcMotor m1LDrive = null;
    protected DcMotor m2RDrive = null;
    protected DcMotor m3Hybrid = null;
    protected DcMotor m4Lift = null;
    protected Servo s1Rotate = null;
    protected Servo s2Zahlop = null;

    protected Double shininessCoefficient = 1.8;

    protected float hsvValues[] = {0F, 0F, 0F};
    private String log = "";



    protected double BatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }










    protected void lvl1(){
        m4Lift.setPower(1);
        sleep(400);
        m4Lift.setPower(0);
        s1Rotate.setPosition(1);
        sleep(1500);
        s1Rotate.setPosition(0);
        sleep(800);
        m4Lift.setPower(-0.8);
        sleep(400);
    }
    protected void lvl2(){
        m4Lift.setPower(1);
        sleep(750);
        m4Lift.setPower(0);
        s1Rotate.setPosition(1);
        sleep(1500);
        s1Rotate.setPosition(0);
        sleep(900);
        m4Lift.setPower(-1);
        if (touch.isPressed()) {
            m4Lift.setPower(0);
        }
    }
    protected void lvl3(){
        m4Lift.setPower(1);
        sleep(1300);
        m4Lift.setPower(0);
        s1Rotate.setPosition(0.9);
        sleep(700);
        s2Zahlop.setPosition(1);
        sleep(350);
        s1Rotate.setPosition(0);
        s2Zahlop.setPosition(0);
        m4Lift.setPower(-1);
        sleep(1270);
        m4Lift.setPower(0);
    }
    protected void carousel() {
        m3Hybrid.setPower(-0.3);
        sleep(680);
        m3Hybrid.setPower(-0.9);
        sleep(850);
        m3Hybrid.setPower(0);
    }
    protected void carousell() {
        m3Hybrid.setPower(0.3);
        sleep(680);
        m3Hybrid.setPower(0.9);
        sleep(850);
        m3Hybrid.setPower(0);
    }


    protected void setMotorsPower(double D1_power, double D2_power) {
        // Send power to wheels
        m1LDrive.setPower(D1_power);
        m2RDrive.setPower(D2_power);
    }



    protected void chassisStopMovement() {
        m1LDrive.setPower(0);
        m2RDrive.setPower(0);
    }

    DistanceSensor distance;
    TouchSensor touch;

    protected void initHW(HardwareMap hardwMap) throws RuntimeException {

        DcMotor m1LDrive = hardwareMap.get(DcMotor.class, "m1 left drive");

        DcMotor m2RDrive = hardwareMap.get(DcMotor.class, "m2 right drive");

        DcMotor m3Hybrid = hardwareMap.get(DcMotor.class, "m3 hybrid");

        DcMotor m4Lift = hardwareMap.get(DcMotor.class, "m4 lift");

        Servo s1Rotate = hardwareMap.get(Servo.class, "s1 rotate");

        Servo s2Zahlop = hardwareMap.get(Servo.class, "s2 Zahlop");

        touch = hardwareMap.get(TouchSensor.class, "Touch");

        distance = hardwareMap.get(DistanceSensor.class, "Distance");







        m1LDrive.setDirection(DcMotor.Direction.FORWARD);

        m2RDrive.setDirection(DcMotor.Direction.FORWARD);

        m3Hybrid.setDirection(DcMotor.Direction.FORWARD);

        m4Lift.setDirection(DcMotor.Direction.FORWARD);
        m4Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m1LDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m2RDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m3Hybrid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m4Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}