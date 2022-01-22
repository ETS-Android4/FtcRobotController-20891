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



@TeleOp(name = "VR 20891 TeleOp", group = "Linear Opmode")

//@Disabled
public class TeleOP2022_2sticks extends LinearOpMode {

    private static final int LED_CHANNEL = 5;
    private final double POWER = 1;
    private final double COUNTS_PER_INCH = 17.1;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime liftTime = new ElapsedTime(5);

    double countValue(double value) {
        if (Math.abs(value) < 0.02) return 0;

        return Math.signum(value) * (0.9 * Math.pow(Math.abs(value), 2) + 0.1);
    }
    protected double getBatteryVoltage() {
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

    @Override
    public void runOpMode() {
        //region HARDWARE&TELEMETRY MAP INIT
        //hw map
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "m1 left drive");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "m2 right drive");
        DcMotor m3Hybrid = hardwareMap.get(DcMotor.class, "m3 hybrid");
        DcMotor m4Lift = hardwareMap.get(DcMotor.class, "m4 lift");
        Servo s1Rotate = hardwareMap.get(Servo.class, "s1 rotate");
        Servo s2Zahlop = hardwareMap.get(Servo.class, "s2 Zahlop");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        //telemetry
        Orientation telemetryAngles;
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetryAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //endregion

        //region DIRECTIONS INIT
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        m3Hybrid.setDirection(DcMotor.Direction.FORWARD);
        m4Lift.setDirection(DcMotor.Direction.FORWARD);
        m4Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Hybrid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion

        waitForStart();
        runtime.reset();

        double leftDriveLeftOffset = 0;
        double leftDriveRightOffset = 0;
        double leftDriveUpOffset = 0;
        double leftDriveDownOffset = 0;

        double rightDriveLeftOffset = 0;
        double rightDriveRightOffset = 0;
        double rightDriveUpOffset = 0;
        double rightDriveDownOffset = 0;

        /*String positionServo = "not ready";
        double voltage = getBatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);*/

        while (opModeIsActive()) {
            //region MOVEMENT
            double horizontalDrive = countValue(gamepad1.right_stick_y);
            double verticalDrive = countValue(gamepad1.left_stick_y);

            double m1DrivePower = horizontalDrive
                    + rightDriveDownOffset
                    + rightDriveUpOffset
                    + rightDriveLeftOffset
                    + rightDriveRightOffset;
            double m2DrivePower = verticalDrive
                    + leftDriveDownOffset
                    + leftDriveUpOffset
                    + leftDriveLeftOffset
                    + leftDriveRightOffset;

            double maxDrivePower = Math.max(m1DrivePower, m2DrivePower);
            if (maxDrivePower >= 1) {
                leftDrive.setPower(POWER * m1DrivePower / maxDrivePower);
                rightDrive.setPower(POWER * m2DrivePower / maxDrivePower);
            } else {
                leftDrive.setPower(POWER * m1DrivePower);
                rightDrive.setPower(POWER * m2DrivePower);
            }

            telemetryAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (gamepad1.dpad_up) {
                leftDriveUpOffset = 0.3;
                rightDriveUpOffset = -0.3;
            } else {
                leftDriveUpOffset = 0;
                rightDriveUpOffset = 0;
            }

            if (gamepad1.dpad_down) {
                leftDriveDownOffset = -0.3;
                rightDriveDownOffset = 0.3;
            } else {
                leftDriveDownOffset = 0;
                rightDriveDownOffset = 0;
            }

            if (gamepad1.dpad_left) {
                leftDriveLeftOffset = -0.33;
                rightDriveLeftOffset = -0.33;
            } else {
                leftDriveLeftOffset = 0;
                rightDriveLeftOffset = 0;
            }

            if (gamepad1.dpad_right) {
                leftDriveRightOffset = 0.33;
                rightDriveRightOffset = 0.33;
            } else {
                leftDriveRightOffset = 0;
                rightDriveRightOffset = 0;
            }
            //endregion

            //region AUTO_SPIN
            // blue alliance
            if (gamepad1.right_bumper) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                m3Hybrid.setPower(-0.3);
                sleep(1111);
                m3Hybrid.setPower(-1);
                sleep(489);
                m3Hybrid.setPower(0);
            }
            //red alliance
            if (gamepad1.left_bumper) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                m3Hybrid.setPower(0.3);
                sleep(1111);
                m3Hybrid.setPower(1);
                sleep(489);
                m3Hybrid.setPower(0);
            }
            //endregion

            //region GRAB
            if (gamepad2.left_stick_y != 0 && gamepad2.left_stick_y > 0) {
                m3Hybrid.setPower(gamepad2.left_stick_y);
            } else {
                m3Hybrid.setPower(0);
            }
            //endregion

            //region LIFTING
            // start lifting
            if (gamepad2.y && touch.isPressed()) liftTime.reset();
            // end lifting
            if (liftTime.milliseconds() < 1300) {
                m4Lift.setPower(-1);
                s1Rotate.setPosition(0.4);
            }
            // проверка по кнопке
            if (gamepad2.right_stick_y > 0 & touch.isPressed()) {
                m4Lift.setPower(0);
            }
            //endregion

            //region CLAW
            // set claw speed
            if (gamepad2.right_stick_y != 0) m4Lift.setPower(gamepad2.right_stick_y);
            else m4Lift.setPower(0);
            // rotate claw
            if (gamepad2.right_bumper) s1Rotate.setPosition(1);
            // reset claw rotation
            if (gamepad2.left_bumper) s1Rotate.setPosition(0);
            //endregion

            //#region TELEMETRY
            telemetry.addData("нажатие кнопки", touch.isPressed());
            telemetry.addData("Показания дальномера", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Status", "Lift Time: " + liftTime.toString());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SErvo status", "not ready");
            telemetry.addData("SErvo position", s1Rotate.getPosition());
            telemetry.addData("Podiem position ", m4Lift.getCurrentPosition());
            telemetry.addData("angleofrotate", telemetryAngles.firstAngle);
            telemetry.update();
            //endregion
        }
    }
}