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
public class WOODright extends LinearOpMode {

    private static double maxpower = 0.7;

    //motors
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor arm = null;

    //servos
    Servo claw;
    Servo wrist;

    //sensors (color, odometers, imu)
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
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        //odometers
        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");

        //robot hardware
        RobotHardware robot = new RobotHardware(fl, fr, bl, br, lift1, lift2, arm);
        robot.innitHardwareMap();

        imuinit();
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

        int[] armHeight = {360, 270, 180, 90, 0};
        resetRuntime();

        //grab cone
        clawClose();
        sleep(300);
        robot.setArm(600, 0.8);
        sleep(200);
        wristTurn();
        moveTo(-16, 0, -90, 3);

        //drive to pole
        robot.setLift(850, 1);
        moveTo(-50, -3, -90, 3);

        //align with pole
        runtime.reset();
        while (runtime.seconds() < 1 && opModeIsActive()) {
            stay(-60, -8, -45);
        }
        runtime.reset();
        while (runtime.seconds() < 0.3 && opModeIsActive()) {
            clawOpen();
        }

        //start of 5 cycles
        for (int i = 0; i < 5; i++){
            alignwithconestack();

            runtime.reset();
            while (runtime.seconds() < 1.4 && opModeIsActive()) {
                movetoconestack();
                robot.setLift(armHeight[i], 0.5);
                robot.setArm(1, 0.5);
                wristReset();
                clawOpen();
            }

            runtime.reset();
            while (runtime.seconds() < 0.5 && opModeIsActive()) {
                clawClose();
            }

            robot.setLift(850, 1);
            robot.setArm(600, 0.8);

            movetopole();
            wristTurn();

            runtime.reset();
            while (runtime.seconds() < 1 && opModeIsActive()) {
                alignwithpole();
            }

            runtime.reset();
            while (runtime.seconds() < 0.2 && opModeIsActive()) {
                alignwithpole();
                clawOpen();
            }

        }


        //parking
        moveTo(-50, -3, 0, 1);
        clawClose();
        wristReset();
        robot.setLift(0, 0.5);
        robot.setArm(0, 0.3);

        update.stop();
        stop();



    }

    public void alignwithconestack() {
        moveTo(-50, 2, 0, 5);
    }

    public void movetoconestack() {
        stay(-50, 18, 0);
    }

    public void movetopole() {
        moveTo(-50, 0, 0, 3);
    }

    public void alignwithpole() {
        stay(-59.5, -10.5, -35);
    }
    public void clawOpen() {
        claw.setPosition(0.4);
    }
    public void clawClose() {
        claw.setPosition(0.515);
    }
    public void wristTurn() {
        wrist.setPosition(0.79);
    }
    public void wristReset() {
        wrist.setPosition(0.13);
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

        if (x > maxpower) {
            x = maxpower;
        }
        else if (x < -maxpower) {
            x = -maxpower;
        }
        else x = x;
        if (y > maxpower) {
            y = maxpower;
        }
        else if (y < -maxpower) {
            y = -maxpower;
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