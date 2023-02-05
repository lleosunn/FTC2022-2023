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

import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class FINALRIGHT extends LinearOpMode {

    //motors
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor arm = null;

    //servos
    Servo lclaw;
    Servo rclaw;
    Servo odo1;
    Servo odo2;

    //sensors (color, odometers, imu)
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    DcMotor verticalLeft, verticalRight, horizontal;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private ElapsedTime runtime = new ElapsedTime();

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

        //motors
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        arm = hardwareMap.get(DcMotor.class, "arm");

        //servos
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        Servo odo1 = hardwareMap.get(Servo.class, "odo1");
        Servo odo2 = hardwareMap.get(Servo.class, "odo2");

        //odometers
        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        //color sensor
        sensorColor = hardwareMap.get(ColorSensor.class, "color1");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color1");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //robot hardware
        RobotHardware robot = new RobotHardware(fl, fr, bl, br, lift1, lift2, arm);
        robot.innitHardwareMap();

        imuinit();

        odo1.setPosition(1);
        odo2.setPosition(0);
        sleep(500);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("angle", getAngle());
        telemetry.update();

        waitForStart();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //start odometry thread
        update = new odometry(verticalLeft, verticalRight, horizontal, 10, imu);
        Thread positionThread = new Thread(update);
        positionThread.start();

        double color = 0;
        int[] armHeight = {320, 240, 160, 80, 0};
        resetRuntime();


        //grab cone
        clawClose();
        robot.setArm(660, 0.4);

        //drive to signal cone
        moveTo(-20, 0, -90, 4);
        runtime.reset();
        while (runtime.seconds() < 0.5) {
            stay(-22, 0, -90);
        }

        //detect color
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

        //drive to pole
        robot.setLift(850, 1);
        moveTo(-40, -3, -90, 3);

        //align with pole
        runtime.reset();
        while (runtime.seconds() < 1 && opModeIsActive()) {
            stay(-56, -6, -45);
        }

        //lower lift
        robot.setLift(680, 1);

        //drop cone
        runtime.reset();
        while (runtime.seconds() < 0.3 && opModeIsActive()) {
            clawOpen();
            robot.setLift(750, 1);
        }

        //start of 5 cycles
        for (int i = 0; i < 5; i++){
            alignwithconestack();

            runtime.reset();
            while (runtime.seconds() < 1.6 && opModeIsActive()) {
                movetoconestack();
                robot.setLift(armHeight[i], 0.5);
                robot.setArm(5, 0.3);
                clawOpen();
            }

            runtime.reset();
            while (runtime.seconds() < 0.2 && opModeIsActive()) {
                clawClose();
            }

            robot.setLift(850, 1);
            robot.setArm(500, 0.4);

            movetopole();

            runtime.reset();
            while (runtime.seconds() < 0.7 && opModeIsActive()) {
                alignwithpole();
                robot.setArm(660, 0.4);
            }

            runtime.reset();
            while (runtime.seconds() < 0.25 && opModeIsActive()) {
                alignwithpole();
                robot.setLift(680, 1);
            }

            runtime.reset();
            while (runtime.seconds() < 0.3 && opModeIsActive()) {
                alignwithpole();
                clawOpen();
                robot.setLift(750, 1);
            }

        }


        //parking
        moveTo(-50, -3, 0, 2);
        clawClose();
        robot.setLift(0, 0.5);
        robot.setArm(3, 0.3);

        if (color == 1){
            moveTo(-50, -24, 0, 2);
        }
        else if (color == 2) {
            moveTo(-50, -6, 0, 2);
        }
        else {
            moveTo(-50, 24, 0, 2);
        }
        update.stop();
        stop();


    }

    public void alignwithconestack() {
        moveTo(-50, 0, 0, 5);
    }

    public void movetoconestack() {
        stay(-50, 23, 0);
    }

    public void movetopole() {
        moveTo(-50, 7, 0, 3);
    }

    public void alignwithpole() {
        stay(-54, -4.5, -45);
    }
    public void clawOpen() {
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);
    }
    public void clawClose() {
        lclaw.setPosition(0.47);
        rclaw.setPosition(0.53);
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

            if(isStopRequested()) {
                update.stop();
            }
        }
    }
    public void stay(double targetX, double targetY, double targetOrientation) {
        double distanceX = targetX - (update.x() / COUNTS_PER_INCH);
        double distanceY = targetY - (update.y() / COUNTS_PER_INCH);
        double x = 0.1 * distanceX;
        double y = 0.1 * distanceY;
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

        if(isStopRequested()) {
            update.stop();
        }
    }


}