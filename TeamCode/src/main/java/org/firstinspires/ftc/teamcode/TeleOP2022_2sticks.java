package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

//@Disable
public class TeleOP2022_2sticks extends LinearOpMode {
    private static final double SLOW_MULTIPLIER = .3;
    private static final double VERTICAL_MULTIPLIER = .3;
    private static final double HORIZONTAL_MULTIPLIER = .33;

    private static final int LED_CHANNEL = 5;
    private final double POWER = 1;
    private final double COUNTS_PER_INCH = 17.1;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime liftTime = new ElapsedTime(5);

    double countDriveValue(double value) {
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        //endregion

        //region DIRECTIONS INIT
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
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

        double speedMultiplier = 1;

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
            double horizontalDrive = countDriveValue(gamepad1.left_stick_y);
            double verticalDrive = countDriveValue(gamepad1.right_stick_x);

            double m1DrivePower = (verticalDrive
                    + rightDriveDownOffset
                    + rightDriveUpOffset
                    + rightDriveLeftOffset
                    + rightDriveRightOffset
                    + horizontalDrive)
                    * speedMultiplier;
            double m2DrivePower = (verticalDrive
                    + leftDriveDownOffset
                    + leftDriveUpOffset
                    + leftDriveLeftOffset
                    + leftDriveRightOffset
                    - horizontalDrive)
                    * speedMultiplier;

            double maxDrivePower = Math.max(m1DrivePower, m2DrivePower);
            if (maxDrivePower >= 1) {
                leftDrive.setPower(POWER * m1DrivePower / maxDrivePower);
                rightDrive.setPower(POWER * m2DrivePower / maxDrivePower);
            } else {
                leftDrive.setPower(POWER * m1DrivePower);
                rightDrive.setPower(POWER * m2DrivePower);
            }

            telemetryAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (gamepad1.right_bumper) speedMultiplier = SLOW_MULTIPLIER;
            else speedMultiplier = 1;

            if (gamepad1.dpad_up) {
                leftDriveUpOffset = VERTICAL_MULTIPLIER;
                rightDriveUpOffset = -VERTICAL_MULTIPLIER;
            } else {
                leftDriveUpOffset = 0;
                rightDriveUpOffset = 0;
            }

            if (gamepad1.dpad_down) {
                leftDriveDownOffset = -VERTICAL_MULTIPLIER;
                rightDriveDownOffset = VERTICAL_MULTIPLIER;
            } else {
                leftDriveDownOffset = 0;
                rightDriveDownOffset = 0;
            }

            if (gamepad1.dpad_left) {
                leftDriveLeftOffset = -HORIZONTAL_MULTIPLIER;
                rightDriveLeftOffset = -HORIZONTAL_MULTIPLIER;
            } else {
                leftDriveLeftOffset = 0;
                rightDriveLeftOffset = 0;
            }

            if (gamepad1.dpad_right) {
                leftDriveRightOffset = HORIZONTAL_MULTIPLIER;
                rightDriveRightOffset = HORIZONTAL_MULTIPLIER;
            } else {
                leftDriveRightOffset = 0;
                rightDriveRightOffset = 0;
            }
            //endregion

            //region AUTO_SPIN
            // blue alliance
            if (gamepad1.x) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                m3Hybrid.setPower(-0.3);
                sleep(1111);
                m3Hybrid.setPower(-1);
                sleep(489);
                m3Hybrid.setPower(0);
            }
            //red alliance
            if (gamepad1.b) {
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
            // opens full lift
            if (gamepad2.dpad_up) {
                s1Rotate.setPosition(0.38);
                m4Lift.setPower(-0.3);
            }
            // set's rotation to 0.55
            if (gamepad2.dpad_down) {
                s1Rotate.setPosition(0.55);
            }
            // проверка по кнопке
            if (gamepad2.right_stick_y > 0 & touch.isPressed()) {
                m4Lift.setPower(0);
            }
            // sets rotation to 0.6
            if (gamepad1.y) {
                s1Rotate.setPosition(0.6);
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