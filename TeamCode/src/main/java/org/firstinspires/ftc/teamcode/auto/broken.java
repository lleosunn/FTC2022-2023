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
import android.graphics.RenderNode;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class broken extends LinearOpMode {

    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor arm = null;
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

    odometry update;



    @Override
    public void runOpMode() {

        imuinit();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        arm = hardwareMap.get(DcMotor.class, "arm");
        RobotHardware robot = new RobotHardware(fl, fr, bl, br, lift1, lift2, arm);

        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        Servo larm = hardwareMap.get(Servo.class, "larm");
        Servo rarm = hardwareMap.get(Servo.class, "rarm");
        Servo lclaw = hardwareMap.get(Servo.class, "lclaw");
        Servo rclaw = hardwareMap.get(Servo.class, "rclaw");

        sensorColor = hardwareMap.get(ColorSensor.class, "color1");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "color1");

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.innitHardwareMap();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("angle", getAngle());
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            update.globalCoordinatePositionUpdate();
            telemetry.addData("X Position", update.x() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", update.y() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", update.h());
            telemetry.addData("imu (degrees)", getAngle());
            telemetry.addData("time", getRuntime());
            telemetry.update();

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

        }

        update = new odometry(verticalLeft, verticalRight, horizontal, 75, imu);
        Thread positionThread = new Thread(update);
        positionThread.start();
        double heading = getAngle();
        double color = 0;

        resetRuntime();

        //move to position 1 (signal cone) & lift arm| Segment 1
        moveTo(0, -36, 0, 2);
        //larm.setPosition(0.16);
        //rarm.setPosition(0.83);

        // Segment 2 (read cone)
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


        // Segment 3 move to pole & raises lift motors
        moveTo(1, -50, 45, 2);
        lift1.setTargetPosition(1100);
        lift2.setTargetPosition(1100);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(1);
        lift2.setPower(1);


        // Segment 4 lowers lift motors
        lift1.setTargetPosition(900);
        lift2.setTargetPosition(900);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(1);
        lift2.setPower(1);

        // Segment 5 claw release cone
        moveTo(1, -50, 45, 2);
        lclaw.setPosition(0.35);
        rclaw.setPosition(0.65);

        // Segment 6 aligns to cone stack
        alignwithconestack();
        //larm.setPosition(0.98);
        //rarm.setPosition(0);
        robot.setLift(400, 0.5);

        // Segment 7 drove to cone stack, open claw
        movetoconestack();
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);

        // Segment 8 grab first cone
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // Segment 9 lifts cone
        robot.setLift(1100, 1);

        // Segment 10 drive to pole
        movetopole();
        //larm.setPosition(0.16);
        //rarm.setPosition(0.83);

        // Segment 11 align with pole
        alignwithpole();

        // Segment 12 drop lift
        robot.setLift(900, 1);

        // Segment 13 first cone cycle deposit
        lclaw.setPosition(0.35);
        rclaw.setPosition(0.65);


        // Segment 14 align with cone
        alignwithconestack();
        robot.setLift(340, 0.5);
        //larm.setPosition(0.98);
        //rarm.setPosition(0);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // Segment 15 move to cone
        movetoconestack();
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);

        // Segment 16 grab cone
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // Segment 17
        robot.setLift(1100, 1);

        // segment 18
        movetopole();
        //larm.setPosition(0.16);
        //rarm.setPosition(0.83);

        // Segment 19
        alignwithpole();

        // segment 20
        robot.setLift(900, 0.5);

        // segment 21
        lclaw.setPosition(0.35);
        rclaw.setPosition(0.65);

        // segment 22
        alignwithconestack();
        robot.setLift(250, 0.5);
        //larm.setPosition(0.98);
        //rarm.setPosition(0);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // segment 23
        movetoconestack();
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);

        // segment 24
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // segment 25
        robot.setLift(1100, 1);

        // segment 26
        movetopole();
        //larm.setPosition(0.16);
        //rarm.setPosition(0.83);

        // segment 27
        alignwithpole();

        // segment 28
        robot.setLift(900, 0.5);

        // segment 29
        lclaw.setPosition(0.35);
        rclaw.setPosition(0.65);

        // segment 30
        alignwithconestack();
        robot.setLift(210, 0.5);
        //larm.setPosition(0.98);
        //rarm.setPosition(0);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // segment 31
        movetoconestack();
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);

        // segment 32
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // segment 33
        robot.setLift(1100, 1);

        // segment 34
        movetopole();
        //larm.setPosition(0.16);
        //rarm.setPosition(0.83);

        // segment 35
        alignwithpole();

        // segment 36
        robot.setLift(900, 0.5);

        // segmnent 37
        lclaw.setPosition(0.35);
        rclaw.setPosition(0.65);

        // segment 38
        alignwithconestack();
        robot.setLift(100, 0.5);
        //larm.setPosition(0.98);
        //rarm.setPosition(0);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // segment 39
        movetoconestack();
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);

        // Segment 40
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // Segment 41
        robot.setLift(1100, 1);

        // Segment 42
        movetopole();
        //larm.setPosition(0.16);
        //rarm.setPosition(0.83);

        // Segment 43
        alignwithpole();

        // Segment 44
        robot.setLift(900, 0.5);

        // Segment 45
        lclaw.setPosition(0.35);
        rclaw.setPosition(0.65);

        // Segment 46
        moveTo(0, -48, 90, 2);
        robot.setLift(0, 0.5);
        //larm.setPosition(0.96);
        //rarm.setPosition(0.02);
        lclaw.setPosition(0.5);
        rclaw.setPosition(0.5);

        // Segment 47
        if (color == 1){
            moveTo(23, -48, 90, 2);
        }
        else if (color == 2) {
            moveTo(0, -48, 90, 2);
        }
        else {
            moveTo(-25, -48, 90, 2);
        }

        // segment 48
        stop();


    }
    public void alignwithconestack() {
        moveTo(0, -48, 90, 2);
    }

    public void movetoconestack() {
        moveTo(-27, -48, 90, 2);
    }

    public void movetopole() {
        moveTo(-10, -48, 90, 2);
    }

    public void alignwithpole() {
        moveTo(0, -46, 45, 2);
    }

    public void moveTo(double targetX, double targetY, double targetOrientation, double error) {
        double distanceX = targetX - (update.x() / COUNTS_PER_INCH);
        double distanceY = targetY - (update.y() / COUNTS_PER_INCH);
        double distance = Math.hypot(distanceX, distanceY);

        while(opModeIsActive() && distance > error) {
            distance = Math.hypot(distanceX, distanceY);
            distanceX = targetX - (update.x() / COUNTS_PER_INCH);
            distanceY = targetY - (update.y() / COUNTS_PER_INCH);

            double x = 0.075 * distanceX;
            double y = 0.075 * distanceY;
            double turn = 0.035 * (update.h() - targetOrientation);
            double theta = Math.toRadians(update.h());

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


}