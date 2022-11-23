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
public class left extends LinearOpMode {
    //declaring motors, distance sensors, timer
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

    //declare gyro (direction, angle)
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    //initialize gyro
    public void imuinit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        //send data to controller (screen), see angle while coding
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
    //when changes direction resets angle to 0
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

    //run program
    @Override
    public void runOpMode() {

        imuinit();

        //quotations --> what put in calibration file, assigns port to motor (ex. poto 9 - "tl")
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        RobotHardware robot = new RobotHardware(fl, fr, bl, br);

        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        Servo larm = hardwareMap.get(Servo.class, "larm");
        Servo rarm = hardwareMap.get(Servo.class, "rarm");
        Servo bclaw = hardwareMap.get(Servo.class, "bclaw");
        Servo fclaw = hardwareMap.get(Servo.class, "fclaw");

        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.innitHardwareMap();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("angle", getAngle());
        telemetry.update();

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "color1");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color1");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bclaw.setPosition(0.5);
        fclaw.setPosition(0.5);
        resetRuntime();


        double color = 0;

        //when it starts
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

            if (getRuntime() < 3){ // drive to cone
                moveTo(0, -18, 0,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                larm.setPosition(0.25);
                rarm.setPosition(0.85);
            }
            while (getRuntime() > 3 && getRuntime() < 4){ // detect color
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
            if (getRuntime() > 4 && getRuntime() < 7) { // drive to pole
                moveTo(0, -56, 0,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 7 && getRuntime() < 10) { // drive to pole
                moveTo(0, -50, -45,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

            }
            if (getRuntime() > 10 && getRuntime() < 12) { // drive to pole
                moveTo(-5, -58, -48,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);

            }

            if (getRuntime() > 12 && getRuntime() < 13) { // deposit
                bclaw.setPosition(1);
                fclaw.setPosition(0);
            }
            if (getRuntime() > 13 && getRuntime() < 14) { // get ready to park
                moveTo(0, -48, -90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                larm.setPosition(0.95);
                rarm.setPosition(0.05);
                bclaw.setPosition(0.5);
                fclaw.setPosition(0.5);
            }
            if (getRuntime() > 14 && getRuntime() < 17) { // park
                if (color == 1){
                    moveTo(23, -48, -90,
                            globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                }
                else if (color == 2) {
                    moveTo(0, -48, -90,
                            globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                }
                else {
                    moveTo(-24, -48, -90,
                            globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
                }
            }
            if (getRuntime() > 17) {
                stop();
            }



        }

    }
    public void moveTo(double targetX, double targetY, double targetOrientation, double currentX, double currentY, double currentOrientation) {
        double h = 0.07 * (targetX - currentX);
        double v = 0.07 * (targetY - currentY);
        double t = 0.03 * (currentOrientation - targetOrientation);
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