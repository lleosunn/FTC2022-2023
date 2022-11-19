package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "odometry")
public class odometry extends LinearOpMode {
    //Drive motors
    DcMotor fl, fr, bl, br;
    //Odometry Wheels
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
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        RobotHardware robot = new RobotHardware(fl, fr, bl, br);

        verticalLeft = hardwareMap.dcMotor.get("fl");
        verticalRight = hardwareMap.dcMotor.get("br");
        horizontal = hardwareMap.dcMotor.get("fr");


        robot.innitHardwareMap();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        imuinit();
        waitForStart();

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal);

        while(opModeIsActive()){

            globalPositionUpdate.globalCoordinatePositionUpdate(getAngle());

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("imu (degrees)", getAngle());

            telemetry.addData("encoderLeft", verticalLeft.getCurrentPosition());
            telemetry.addData("encoderRight", verticalRight.getCurrentPosition());
            telemetry.addData("encoderAux", horizontal.getCurrentPosition());

            telemetry.update();

            double modifier = 0.6;
            double x = modifier * gamepad1.left_stick_x;
            double y = modifier * -gamepad1.left_stick_y;
            double turn = modifier * gamepad1.right_stick_x;
            double theta = Math.toRadians(getAngle());

            double tlH = x * Math.sin(theta - (Math.PI/4));
            double trH = x * Math.cos(theta - (Math.PI/4));
            double blH = x * Math.cos(theta - (Math.PI/4));
            double brH = x * Math.sin(theta - (Math.PI/4));

            double tlV = y * Math.sin(theta + (Math.PI/4));
            double trV = y * Math.cos(theta + (Math.PI/4));
            double blV = y * Math.cos(theta + (Math.PI/4));
            double brV = y * Math.sin(theta + (Math.PI/4));

            if (gamepad1.a) {
                moveTo(0, 0, 0,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
            }
            else if (gamepad1.dpad_up) {
                moveTo(20, 20, -90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
            }
            else if (gamepad1.dpad_left) {
                moveTo(-20, 20, -180,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
            }
            else if (gamepad1.dpad_down) {
                moveTo(-20, -20, -90,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
            }
            else if (gamepad1.dpad_right) {
                moveTo(20, -20, 135,
                        globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, getAngle() % 360);
            }
            else {
                fl.setPower(tlV - tlH + turn);
                fr.setPower(trV - trH - turn);
                bl.setPower(blV - blH + turn);
                br.setPower(brV - brH - turn);
            }

            if (gamepad1.right_stick_button) {
                resetAngle();
            }
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
    public void moveTo(double targetX, double targetY, double targetOrientation, double currentX, double currentY, double currentOrientation) {
        double h = 0.1 * (targetX - currentX);
        double v = 0.1 * (targetY - currentY);
        double t = 0.02 * (currentOrientation - targetOrientation);
        double x;
        double y;
        double turn;
        double theta = Math.toRadians(getAngle());

        if (h > 0.5) {
            x = 0.5;
        } else x = h;
        if (v > 0.5) {
            y = 0.5;
        } else y = v;
        if (t > 0.2) {
            turn = 0.2;
        } else turn = t;

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
