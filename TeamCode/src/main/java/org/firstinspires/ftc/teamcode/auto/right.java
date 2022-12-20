package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry;
import org.firstinspires.ftc.teamcode.RobotHardware;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class right extends LinearOpMode {

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

        odometry update = new odometry(verticalLeft, verticalRight, horizontal, 75, imu);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);
        resetRuntime();
        double polex = 0;
        double poley = -46;
        double poleh = 45;

        double color = 0;
        while (opModeIsActive()) {

            double heading = getAngle();
            update.globalCoordinatePositionUpdate();
            telemetry.addData("X Position", update.x() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", update.y() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", update.h());
            telemetry.addData("imu (degrees)", heading);
            telemetry.addData("time", getRuntime());
            telemetry.addData("color", color);
            telemetry.update();

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (getRuntime() < 1.5){ // drive to pole
                moveTo(0, -36, 0, update.x(), update.y(), heading % 360);
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
                moveTo(1, -50, 45, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(1100);
                lift2.setTargetPosition(1100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 3 && getRuntime() < 3.25) { // drop lift
                moveTo(1, -50, 45, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 3.25 && getRuntime() < 3.5) { // deposit 0
                moveTo(1, -50, 45, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }
            if (getRuntime() > 3.5 && getRuntime() < 4) { // align with stack
                moveTo(0, -48, 90, update.x(), update.y(), heading % 360);
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
            if (getRuntime() > 4 && getRuntime() < 5.5) { // drive to stack
                moveTo(-27, -48, 90, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);
            }
            if (getRuntime() > 5.5 && getRuntime() < 5.75) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 6 && getRuntime() < 6.25) { // lift up
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
                lift1.setTargetPosition(1100);
                lift2.setTargetPosition(1100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 6.25 && getRuntime() < 6.75) { // drive to pole
                moveTo(-10, -48, 90, update.x(), update.y(), heading % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);
            }
            if (getRuntime() > 6.75 && getRuntime() < 7.75) { // correction
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
            }
            if (getRuntime() > 7.75 && getRuntime() < 8) { // drop lift
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 8 && getRuntime() < 8.25) { // deposit 1
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }
            if (getRuntime() > 8.25 && getRuntime() < 8.75) { // align with stack
                moveTo(0, -48, 90, update.x(), update.y(), heading % 360);
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
            if (getRuntime() > 8.75 && getRuntime() < 10.5) { // drive to stack
                moveTo(-27, -48, 90, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 10.5 && getRuntime() < 10.75) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 10.75 && getRuntime() < 11) { // lift up
                lift1.setTargetPosition(1100);
                lift2.setTargetPosition(1100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 11 && getRuntime() < 11.5) { // drive to pole
                moveTo(-10, -48, 90, update.x(), update.y(), heading % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);
            }
            if (getRuntime() > 11.5 && getRuntime() < 12.5) { // correction
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
            }
            if (getRuntime() > 12.5 && getRuntime() < 12.75) { // drop lift
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 12.75 && getRuntime() < 13) { // deposit 2
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }

            if (getRuntime() > 13 && getRuntime() < 13.5) { // align with stack
                moveTo(0, -48, 90, update.x(), update.y(), heading % 360);
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
            if (getRuntime() > 13.5 && getRuntime() < 15.25) { // drive to stack
                moveTo(-27, -48, 90, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 15.25 && getRuntime() < 15.5) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 15.5 && getRuntime() < 15.75) { // lift up
                lift1.setTargetPosition(1100);
                lift2.setTargetPosition(1100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 15.75 && getRuntime() < 16.25) { // drive to pole
                moveTo(-10, -48, 90, update.x(), update.y(), heading % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 16.25 && getRuntime() < 17.25) { // correction
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
            }
            if (getRuntime() > 17.25 && getRuntime() < 17.5) { // drop lift
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 17.5 && getRuntime() < 17.75) { // deposit 3
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }

            if (getRuntime() > 17.75 && getRuntime() < 18.25) { // align with stack
                moveTo(0, -48, 90,
                        update.x(), update.y(), heading % 360);
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
            if (getRuntime() > 18.25 && getRuntime() < 20) { // drive to stack
                moveTo(-27, -48, 90, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);
            }
            if (getRuntime() > 20 && getRuntime() < 20.25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 20.25 && getRuntime() < 20.5) { // lift up
                lift1.setTargetPosition(1100);
                lift2.setTargetPosition(1100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 20.5 && getRuntime() < 21) { // drive to pole
                moveTo(-10, -48, 90, update.x(), update.y(), heading % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 21 && getRuntime() < 22) { // correction
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
            }
            if (getRuntime() > 22 && getRuntime() < 22.25) { // drop lift
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 22.25 && getRuntime() < 22.5) { // deposit 4
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }

            if (getRuntime() > 22.5 && getRuntime() < 23) { // align with stack
                moveTo(0, -48, 90, update.x(), update.y(), heading % 360);
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
            if (getRuntime() > 23 && getRuntime() < 24.75) { // drive to stack
                moveTo(-27, -48, 90, update.x(), update.y(), heading % 360);
                lclaw.setPosition(0.25);
                rclaw.setPosition(0.75);

            }
            if (getRuntime() > 24.75 && getRuntime() < 25) { // grab cone
                lclaw.setPosition(0.5);
                rclaw.setPosition(0.5);
            }
            if (getRuntime() > 25 && getRuntime() < 25.25) { // lift up
                lift1.setTargetPosition(1100);
                lift2.setTargetPosition(1100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (getRuntime() > 25.25 && getRuntime() < 25.75) { // drive to pole
                moveTo(-10, -48, 90, update.x(), update.y(), heading % 360);
                larm.setPosition(0.16);
                rarm.setPosition(0.83);

            }
            if (getRuntime() > 25.75 && getRuntime() < 26.75) { // correction
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
            }
            if (getRuntime() > 26.75 && getRuntime() < 27) { // drop lift
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (getRuntime() > 27 && getRuntime() < 27.25) { // deposit 5
                moveTo(polex, poley, poleh, update.x(), update.y(), heading % 360);

                lclaw.setPosition(0.35);
                rclaw.setPosition(0.65);
            }
            if (getRuntime() > 27.25 && getRuntime() < 27.75) { // get ready to park
                moveTo(0, -48, 90, update.x(), update.y(), heading % 360);
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
            if (getRuntime() > 27.75 && getRuntime() < 30) { // park
                if (color == 1){
                    moveTo(23, -48, 90, update.x(), update.y(), heading % 360);
                }
                else if (color == 2) {
                    moveTo(0, -48, 90, update.x(), update.y(), heading % 360);
                }
                else {
                    moveTo(-25, -48, 90, update.x(), update.y(), heading % 360);
                }
            }
            if (getRuntime() > 30) {
                stop();
            }
        }
    }

    public void moveTo(double targetX, double targetY, double targetOrientation, double currentX, double currentY, double currentOrientation) {
        double x = 0.075 * (targetX - currentX / COUNTS_PER_INCH);
        double y = 0.075 * (targetY - currentY / COUNTS_PER_INCH);
        double turn = 0.035 * (currentOrientation - targetOrientation);
        double theta = Math.toRadians(currentOrientation);

        if (x > 0.6) {
            x = 0.6;
        }
        else if (x < -0.6) {
            x = -0.6;
        }
        else x = x;
        if (y > 0.6) {
            y = 0.6;
        }
        else if (y < -0.6) {
            y = -0.6;
        }
        else y = y;
        if (turn > 0.3) {
            turn = 0.3;
        }
        else if (turn < -0.3) {
            turn = -0.3;
        }

        else turn = turn;
        double l = y * Math.sin(theta + (Math.PI/4)) - x * Math.sin(theta - (Math.PI/4));
        double r = y * Math.cos(theta + (Math.PI/4)) - x * Math.cos(theta - (Math.PI/4));

        fl.setPower(l + turn);
        fr.setPower(r - turn);
        bl.setPower(r + turn);
        br.setPower(l - turn);
    }
}