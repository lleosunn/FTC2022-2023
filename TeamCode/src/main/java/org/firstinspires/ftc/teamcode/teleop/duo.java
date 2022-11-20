package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="duo", group="Linear Opmode")

public class duo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;

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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        Servo larm = hardwareMap.get(Servo.class, "larm");
        Servo rarm = hardwareMap.get(Servo.class, "rarm");
        Servo bclaw = hardwareMap.get(Servo.class, "bclaw");
        Servo fclaw = hardwareMap.get(Servo.class, "fclaw");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        imuinit();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Angle", getAngle());
//            telemetry.addData("y", -gamepad1.left_stick_y);
//            telemetry.addData("x", gamepad1.left_stick_x);
//            telemetry.addData("turn", gamepad1.right_stick_x);
            telemetry.addData("lift1", lift1.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.toRadians(getAngle());
            double modifier;

//            double flH = x * Math.sin(theta - (Math.PI/4));
//            double frH = x * Math.cos(theta - (Math.PI/4));
//            double blH = x * Math.cos(theta - (Math.PI/4));
//            double brH = x * Math.sin(theta - (Math.PI/4));
//
//            double flV = y * Math.sin(theta + (Math.PI/4));
//            double frV = y * Math.cos(theta + (Math.PI/4));
//            double blV = y * Math.cos(theta + (Math.PI/4));
//            double brV = y * Math.sin(theta + (Math.PI/4));

            if (gamepad1.right_trigger > 0) {
                modifier = 0.3;
            } else if (gamepad1.left_trigger > 0) {
                modifier = 0.75;
            } else modifier = 0.5;

            fl.setPower((modifier*1.15)*(y + x + turn));
            fr.setPower(modifier*(y - x - turn));
            bl.setPower(modifier*(y - x + turn));
            br.setPower(modifier*(y + x - turn));


//            fl.setPower(flV - flH - turn);
//            fr.setPower(frV - frH + turn);
//            bl.setPower(blV - blH - turn);
//            br.setPower(brV - brH + turn);

            if (gamepad2.a) { //deposit position
                larm.setPosition(0.25);
                rarm.setPosition(0.75);
            }
            if (gamepad2.b) { //intake position
                larm.setPosition(0.95);
                rarm.setPosition(0.05);
            }
            if (gamepad2.left_trigger > 0){ //back claw close
                bclaw.setPosition(0.5);
            }
            if (gamepad2.left_bumper){ //back claw open
                bclaw.setPosition(1);
            }
            if (gamepad2.right_trigger > 0){ //front claw close
                fclaw.setPosition(0.5);
            }
            if (gamepad2.right_bumper) { //front claw open
                fclaw.setPosition(0);
            }

            if(gamepad2.dpad_up) { // High
                lift1.setTargetPosition(1000);
                lift2.setTargetPosition(1000);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if(gamepad2.dpad_down) { // Set
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.3);
                lift2.setPower(0.3);
            }
            if(gamepad2.y) {
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if(gamepad2.dpad_right) { // Medium
                lift1.setTargetPosition(600);
                lift2.setTargetPosition(600);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }

            if(gamepad2.dpad_left) { // Low
                lift1.setTargetPosition(225);
                lift2.setTargetPosition(225);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }

            if(gamepad2.x) { // Flat
                larm.setPosition(0.9);
                rarm.setPosition(0.1);
            }

            if(gamepad2.left_stick_y > 0){
                lift1.setTargetPosition(lift1.getCurrentPosition()-50);
                lift2.setTargetPosition(lift2.getCurrentPosition()-50);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if(gamepad2.left_stick_y < 0){
                lift1.setTargetPosition(lift1.getCurrentPosition()+50);
                lift2.setTargetPosition(lift2.getCurrentPosition()+50);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }


        }
    }
}