


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "VR 20891 TeleOp 2 sticks", group = "Linear Opmode")

//@Disabled
public class TeleOP2022_2sticks extends LinearOpMode {

    private static final int LED_CHANNEL = 5;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lifttime = new ElapsedTime(5);



    //private DcMotor m3Hybrid = null;





    double magic(double input) {
        if(Math.abs(input)<0.02){

            double nol=0;
            return nol;
        } else{
            return Math.signum(input) * (0.9* Math.pow(Math.abs(input), 2)+0.1);
        }
    }
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








      DistanceSensor distance;
      TouchSensor touch;


    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * End of functions declaration
     */
    final double COUNTS_PER_INCH = 17.1;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // HardWare map
        DcMotor m1LDrive = hardwareMap.get(DcMotor.class, "m1 left drive");
        DcMotor m2RDrive = hardwareMap.get(DcMotor.class, "m2 right drive");
        DcMotor m3Hybrid = hardwareMap.get(DcMotor.class, "m3 hybrid");
        DcMotor m4Lift = hardwareMap.get(DcMotor.class, "m4 lift");
        Servo s1Rotate = hardwareMap.get(Servo.class, "s1 rotate");
        Servo s2Zahlop = hardwareMap.get(Servo.class, "s2 Zahlop");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        m1LDrive.setDirection(DcMotor.Direction.FORWARD);

        m2RDrive.setDirection(DcMotor.Direction.REVERSE);

        m3Hybrid.setDirection(DcMotor.Direction.FORWARD);

        m4Lift.setDirection(DcMotor.Direction.FORWARD);
        m4Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m1LDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m2RDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m3Hybrid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m4Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart();
        runtime.reset();


        double m1DrivePower;
        double m2DrivePower;

        double m1DrivePowerfordrivetofoundation=0;
        double m2DrivePowerfordrivetofoundation=0;

        double m1DrivePowerfordrivetofoundation2=0;
        double m2DrivePowerfordrivetofoundation2=0;

        double m1DrivePowerfordrivetofoundation1=0;
        double m2DrivePowerfordrivetofoundation1=0;

        double m1DrivePowerfordrivetofoundation11=0;
        double m2DrivePowerfordrivetofoundation11=0;

        double seconds;

        String positionServo = "not ready";

        //s1Rotate.setPosition(0);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);






        //ТЕЛЕОП НАЧИНАЕТСЯ




        while (opModeIsActive()) {

            double zagrebalo = gamepad2.left_stick_y;

            double lift = gamepad2.right_stick_y;

            double vperednazad = gamepad1.left_stick_y;

            double povorot = gamepad1.right_stick_x;

            ////////////////////////////////////////////////////
            //начало кода передвижения

            povorot = magic(povorot);
            vperednazad = magic(vperednazad);

            m1DrivePower = vperednazad+m2DrivePowerfordrivetofoundation1+m2DrivePowerfordrivetofoundation11+m2DrivePowerfordrivetofoundation2+m2DrivePowerfordrivetofoundation-povorot;

            m2DrivePower = vperednazad+m1DrivePowerfordrivetofoundation1+m1DrivePowerfordrivetofoundation11+m1DrivePowerfordrivetofoundation2+m1DrivePowerfordrivetofoundation+povorot;

            double mochs=1;
            double max = Math.max(m1DrivePower, m2DrivePower);
            // Send calculated power to wheels
            if (max >= 1) {
                m1LDrive.setPower(mochs*m1DrivePower *1/ max);
                m2RDrive.setPower(mochs*m2DrivePower *1/ max);
            } else {
                m1LDrive.setPower(mochs*m1DrivePower*1);
                m2RDrive.setPower(mochs*m2DrivePower*1);
            }

//----------------------------------------



            //углы

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //медленное движение

            if(gamepad1.dpad_right){
                m1DrivePowerfordrivetofoundation11=0.3;
                m2DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
            }else{
                m1DrivePowerfordrivetofoundation11=0;
                m2DrivePowerfordrivetofoundation11=0;
            }
            if(gamepad1.dpad_left){
                m1DrivePowerfordrivetofoundation1=-0.3;
                m2DrivePowerfordrivetofoundation1=0.3;
            }else{
                m1DrivePowerfordrivetofoundation1=0;
                m2DrivePowerfordrivetofoundation1=0;
            }
            if(gamepad1.dpad_down){
                m1DrivePowerfordrivetofoundation=-0.33;
                m2DrivePowerfordrivetofoundation=-0.33;
            }else{
                m1DrivePowerfordrivetofoundation=0;
                m2DrivePowerfordrivetofoundation=0;
            }
            if(gamepad1.dpad_up){
                m1DrivePowerfordrivetofoundation2=0.33;
                m2DrivePowerfordrivetofoundation2=0.33;
            }else{
                m1DrivePowerfordrivetofoundation2=0;
                m2DrivePowerfordrivetofoundation2=0;
            }


            /////////////////////////////////////////
            //конец кода передвижения






            //скорость захвата

            if (zagrebalo > 0.05||zagrebalo <0.05) {
                    m3Hybrid.setPower(zagrebalo);
            } else {
                m3Hybrid.setPower(0);
            }




            //скорость лифта

            if (lift !=0){
                m4Lift.setPower(lift);
            } else {m4Lift.setPower(0);}


            //проверка по кнопке

            if (touch.isPressed() & lift > 0) {
                    m4Lift.setPower(0);
                    lift = 0;
            }


            //автоподъем на 3 этаж

            if (gamepad2.y) {
                //ElapsedTime lifttime = new ElapsedTime(Resolution.SECONDS);
                if (touch.isPressed()) {
                    lifttime.reset();
                } else {}

            }

            //время автоподъема

            if (lifttime.milliseconds() < 1300) {
                m4Lift.setPower(-1);
                s1Rotate.setPosition(0.4);
            }




            //переворот клешни

            if (gamepad2.right_bumper) {
                s1Rotate.setPosition(0.65);
            }

            //клешня в начальное положение

            if(gamepad2.left_bumper){
                s1Rotate.setPosition(0);
            }


            /*if (lift<0.05){
                s1Rotate.setPosition(0.34);
            }
            if (lift>0.05){
                s1Rotate.setPosition(0);
            }*/



            //автокручение карусели за синих

            if (gamepad1.x) {
                m1LDrive.setPower(0);
                m2RDrive.setPower(0);
                m3Hybrid.setPower(-0.3);
                sleep(1111);
                m3Hybrid.setPower(-1);
                sleep(489);
                m3Hybrid.setPower(0);
            }

            //автокручение карусели за красных

            if (gamepad1.b) {
                m1LDrive.setPower(0);
                m2RDrive.setPower(0);
                m3Hybrid.setPower(0.3);
                sleep(1111);
                m3Hybrid.setPower(1);
                sleep(489);
                m3Hybrid.setPower(0);
            }



            // Show the elapsed game time and wheel power.


            // Телеметрия

//            |
//            |
//            |
//            V
            {

                telemetry.addData("нажатие кнопки", touch.isPressed());


                telemetry.addData("Показания дальномера", distance.getDistance(DistanceUnit.CM));

                telemetry.addData("Status", "Lift Time: " + lifttime.toString());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("SErvo status", positionServo);
                telemetry.addData("SErvo position", s1Rotate.getPosition());
                telemetry.addData("Podiem position ", m4Lift.getCurrentPosition());

                telemetry.addData("angleofrotate", angles.firstAngle);

                telemetry.update();


            }
        }

    }

}



