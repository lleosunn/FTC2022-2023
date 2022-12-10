package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.RobotHardware;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import java.util.List;

@Autonomous
public class rightcycle extends LinearOpMode {

    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    DcMotor verticalLeft, verticalRight, horizontal;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    public void imuinit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        telemetry.addData("Gyro Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro Mode", "ready");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    final double COUNTS_PER_INCH = 1860;

    @Override
    public void runOpMode() {

        imuinit();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        RobotHardware robot = new RobotHardware(fl, fr, bl, br, lift1, lift2);

        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        Servo larm = hardwareMap.get(Servo.class, "larm");
        Servo rarm = hardwareMap.get(Servo.class, "rarm");
        Servo lclaw = hardwareMap.get(Servo.class, "lclaw");
        Servo rclaw = hardwareMap.get(Servo.class, "rclaw");

        robot.innitHardwareMap();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("angle", getAngle());
        telemetry.update();

        sensorColor = hardwareMap.get(ColorSensor.class, "color1");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "color1");

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);
        resetRuntime();

        double color = 0;
        while (opModeIsActive()) {

            globalPositionUpdate.globalCoordinatePositionUpdate(getAngle());
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("imu (degrees)", getAngle());
            telemetry.addData("time", getRuntime());
            telemetry.addData("color", color);
            telemetry.update();

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (getRuntime() < 1.5){ // drive to pole
                moveTo(0, -36, 0,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);
            }
            while (getRuntime() > 1 && getRuntime() < 1.5){ // detect color
                if (sensorColor.red() > sensorColor.blue()) { // Detect Red
                    if (sensorColor.red() > sensorColor.green()) {
                        telemetry.addLine("RED (A)");
                        color = 1;
                    }
                } else if (sensorColor.blue() > sensorColor.red()) { // Detect Blue
                    if (sensorColor.blue() > sensorColor.green()) {
                        telemetry.addLine("BLUE (B)");
                        color = 2;

                    }
                } else if (sensorColor.green() > sensorColor.red()) { // Detect Green
                    if (sensorColor.green() > sensorColor.blue()) {
                        telemetry.addLine("GREEN (C)");
                        color = 3;

                    }
                }
            }
            if (getRuntime() > 1.5 && getRuntime() < 3) { // correction
                moveTo(1, -49, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 3 && getRuntime() < 3.25) { // drop lift
                moveTo(1, -49, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 3.25 && getRuntime() < 3.5) { // deposit 0
                moveTo(1, -49, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }
            if (getRuntime() > 3.5 && getRuntime() < 4) { // align with stack
                moveTo(0, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(400);
                lift2.setTargetPosition(400);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.98);
                rarm.setPosition(0);
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 4 && getRuntime() < 6) { // drive to stack
                moveTo(-27, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 6 && getRuntime() < 6.25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 6.25 && getRuntime() < 6.5) { // lift up
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 6.5 && getRuntime() < 7) { // drive to pole
                moveTo(-10, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);
            }
            if (getRuntime() > 7 && getRuntime() < 8) { // correction
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
            }
            if (getRuntime() > 8 && getRuntime() < 8.25) { // drop lift
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 8.25 && getRuntime() < 8.5) { // deposit 1
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }
            if (getRuntime() > 8.5 && getRuntime() < 9) { // align with stack
                moveTo(0, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(340);
                lift2.setTargetPosition(340);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.98);
                rarm.setPosition(0);
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 9 && getRuntime() < 11) { // drive to stack
                moveTo(-27, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 11 && getRuntime() < 11.25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 11.25 && getRuntime() < 11.5) { // lift up
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 11.5 && getRuntime() < 12) { // drive to pole
                moveTo(-10, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 12 && getRuntime() < 13) { // correction
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

            }
            if (getRuntime() > 13 && getRuntime() < 13.25) { // drop lift
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 13.25 && getRuntime() < 13.5) { // deposit 2
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }

            if (getRuntime() > 13.5 && getRuntime() < 14) { // align with stack
                moveTo(0, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(250);
                lift2.setTargetPosition(250);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.98);
                rarm.setPosition(0);
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 14 && getRuntime() < 16) { // drive to stack
                moveTo(-27, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 16 && getRuntime() < 16.25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 16.25 && getRuntime() < 16.5) { // lift up
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 16.5 && getRuntime() < 17) { // drive to pole
                moveTo(-10, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 17 && getRuntime() < 18) { // correction
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

            }
            if (getRuntime() > 18 && getRuntime() < 18.25) { // drop lift
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 18.25 && getRuntime() < 18.5) { // deposit 3
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }

            if (getRuntime() > 18.5 && getRuntime() < 19) { // align with stack
                moveTo(0, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(210);
                lift2.setTargetPosition(210);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.98);
                rarm.setPosition(0);
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 19 && getRuntime() < 21) { // drive to stack
                moveTo(-27, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);
            }
            if (getRuntime() > 21 && getRuntime() < 21.25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 21.25 && getRuntime() < 21.5) { // lift up
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 21.5 && getRuntime() < 22) { // drive to pole
                moveTo(-10, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 22 && getRuntime() < 23) { // correction
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

            }
            if (getRuntime() > 23 && getRuntime() < 23.25) { // drop lift
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 23.25 && getRuntime() < 23.5) { // deposit 4
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }

            if (getRuntime() > 23.5 && getRuntime() < 24) { // align with stack
                moveTo(0, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(100);
                lift2.setTargetPosition(100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.98);
                rarm.setPosition(0);
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 24 && getRuntime() < 26) { // drive to stack
                moveTo(-27, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 26 && getRuntime() < 26.25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 26.25 && getRuntime() < 26.5) { // lift up
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 26.5 && getRuntime() < 27) { // drive to pole
                moveTo(-10, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 27 && getRuntime() < 28) { // correction
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

            }
            if (getRuntime() > 28 && getRuntime() < 28.25) { // drop lift
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 28.25 && getRuntime() < 28.5) { // deposit 5
                moveTo(0, -48, 45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }
            if (getRuntime() > 28.5 && getRuntime() < 29) { // get ready to park
                moveTo(0, -48, 90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.96);
                rarm.setPosition(0.02);
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 29 && getRuntime() < 31) { // park
                if (color == 1){
                    moveTo(23, -48, 90,
                            globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                }
                else if (color == 2) {
                    moveTo(0, -48, 90,
                            globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                }
                else {
                    moveTo(-25, -48, 90,
                            globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                }
            }
            if (getRuntime() > 31) {
                stop();
            }
        }
    }

    public void moveTo(double targetX, double targetY, double targetOrientation, double currentX, double currentY, double currentOrientation) {
        double h = 0.08 * (targetX - currentX);
        double v = 0.08 * (targetY - currentY);
        double t = 0.04 * (currentOrientation - targetOrientation);
        double x;
        double y;
        double turn;
        double theta = Math.toRadians(getAngle());

        if (h > 0.6) {
            x = 0.6;
        }
        else if (h < -0.6) {
            x = -0.6;
        }
        else x = h;

        if (v > 0.6) {
            y = 0.6;
        }
        else if (v < -0.6) {
            y = -0.6;
        }
        else y = v;

        if (t > 0.3) {
            turn = 0.3;
        }
        else if (t < -0.3) {
            turn = -0.3;
        }
        else turn = t;

        double tlH = x * Math.sin(theta - (Math.PI/4));
        double trH = x * Math.cos(theta - (Math.PI/4));
        double blH = x * Math.cos(theta - (Math.PI/4));
        double brH = x * Math.sin(theta - (Math.PI/4));

        double tlV = y * Math.sin(theta + (Math.PI/4));
        double trV = y * Math.cos(theta + (Math.PI/4));
        double blV = y * Math.cos(theta + (Math.PI/4));
        double brV = y * Math.sin(theta + (Math.PI/4));

        fl.setPower(tlV - tlH + turn);
        fr.setPower(trV - trH - turn);
        bl.setPower(blV - blH + turn);
        br.setPower(brV - brH - turn);
    }
}