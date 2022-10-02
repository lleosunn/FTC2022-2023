package org.firstinspires.ftc.teamcode;

// The following imports are for OpenCV
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;
import java.util.Arrays;

// The following are for the core autonomous
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

// The OpenCV and FTC Dashboard part of the code was taken from PinkToTheFuture:
// https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
// FTC Dashboard comes from:
// https://acmerobotics.github.io/ftc-dashboard/gettingstarted.html
// And Opencv comes from:
// https://opencv.org/

// All of the movement and custom code for our robot comes from us, including the code for imu

// FTC Team Scorpio - 15171
// http://ftcscorpio.com/
/*
The tag
--A--
is a marker to figure out which parts of code to delete if you do not have the drivetrain. This is the first instance of the tag.
From instance 2-3, comment out if you are not using the motors and some sensors (REV Distance)
From instance 4-5, comment out if you are not using the motors and some sensors (REV Distance)
From instance 6-7, comment out if you are not using the motors and some sensors (REV Distance)


 */




@Config //Disable if not using FTC Dashboard
@Autonomous(name="Vanahiem", group = "RachitCode")

public class automain extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Yellow Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(150.0, 100.0, 10.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow Range
    //public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);



    // If you don't have the drivetrain configured, comment this out -----------\ (Search for "--A--" and you will find all of the places to comment out) --A--

    // Motor Configuration
    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange2;
    private DistanceSensor sensorRange3;


    DcMotor tl = hardwareMap.get(DcMotor.class, "tl");
    DcMotor tr = hardwareMap.get(DcMotor.class, "tr");
    DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
    DcMotor br = hardwareMap.get(DcMotor.class, "br");
    Servo claw1 = hardwareMap.get(Servo.class, "claw1");
    Servo claw2 = hardwareMap.get(Servo.class, "claw2");

    // Until Here ---------------------------------------------------------------/ --A--

    // Gyro Sensor Calibration
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    // Gyro Sensor Functions
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

        telemetry.addData("imu calibration status", imu.getCalibrationStatus().toString());

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


    // To comment out for not using motors, comment from here to the other comment thingy ---------------------------------------------------------\ --A--


    public void movefb(double power, double distance, double direction) {
        if (getAngle() < -1){ //turn left
            tl.setPower(-(power - (0.01 * getAngle())));
            tr.setPower((power + (0.01 * getAngle())));
            bl.setPower(-(power - (0.01 * getAngle())));
            br.setPower((power + (0.01 * getAngle())));
        } else if (getAngle() > 1){ //turn right
            tl.setPower(-(power - (0.01 * getAngle())));
            tr.setPower((power + (0.01 * getAngle())));
            bl.setPower(-(power - (0.01 * getAngle())));
            br.setPower((power + (0.01 * getAngle())));
        } else {
            if (direction > 0) {
                tl.setPower(power);
                tr.setPower(-power);
                bl.setPower(power);
                br.setPower(-power);
            } else if (direction < 0) {
                tl.setPower(-power);
                tr.setPower(power);
                bl.setPower(-power);
                br.setPower(power);
            } else {
                telemetry.addData("WARNING, THERE IS AN ERROR IN THE USAGE OF THE DIRECTION FUNCTION, DIRECTION IS", direction);
            }

        }
    }

    // deprecated
    public void turn(double power, double angle, double direction) {
        if (direction > 0) {
            tl.setPower(-(power - (0.01 * angle)));
            tr.setPower((power + (0.01 * angle)));
            bl.setPower(-(power - (0.01 * angle)));
            br.setPower((power + (0.01 * angle)));
        }
        if (direction < 0) {
            tl.setPower(-(-(power - (0.01 * angle))));
            tr.setPower(-((power + (0.01 * angle))));
            bl.setPower(-(-(power - (0.01 * angle))));
            br.setPower(-((power + (0.01 * angle))));
        }
        if (direction == 0) {
            telemetry.addData("THERE IS A PROBLEM WITH THE DIRECTION IN THE TURN FUNCTION", direction);
        }
    }

    public void strafe(double power, double distance, double direction) {
        if (direction > 0) {
            tl.setPower(power);
            tr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        } else if (direction < 0) {
            tl.setPower(-power);
            tr.setPower(-power);
            bl.setPower(-power);
            br.setPower(-power);
        } else {
            telemetry.addData("THERE IS A PROBLEM IN THE DIRECTION FOR THE strafe() FUNCTION", direction);
        }

    }

    // Comment end here for no motor ------------------------------------------------------------------------------------------------------------------/ --A--

    @Override
    public void runOpMode()
    {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamMain"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        // Continuation of the Autonomous Core Setup --A--
        
        tl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imuinit();
        
        // --A--
        
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            //testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());

            telemetry.addData("Locating the object in X", myPipeline.getRectMidpointX());
            telemetry.addData("Locating the object in Y", myPipeline.getRectMidpointY());

            webcam.openCameraDevice();

            // Debug
            telemetry.addData("Signal", myPipeline.signalDetect());
            telemetry.addData("If signal is not found, this should be true", myPipeline.findContoursRAW());

            // Green
            myPipeline.configureScalarLower(0.0, 0.0, 0.0);
            myPipeline.configureScalarUpper(255.0, 120.0, 120.0);
            sleep(2000);
            if (myPipeline.signalDetect() == false) {
                // Yellow
                myPipeline.configureScalarLower(0.0, 100.0, 0.0);
                myPipeline.configureScalarUpper(255.0, 170.0, 120.0);
                sleep(2000);
                if (myPipeline.signalDetect() == false) {
                    sleep(2000);
                    telemetry.addData("SIGNAL FOUND: C", !myPipeline.signalDetect());
                } else {
                    telemetry.addData("SIGNAL FOUND: B", !myPipeline.signalDetect());
                }
            } else {
                telemetry.addData("SIGNAL FOUND: A", !myPipeline.signalDetect());
            }
            telemetry.update();

        }
    }
    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }






    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
}


// congrats for reading the code
