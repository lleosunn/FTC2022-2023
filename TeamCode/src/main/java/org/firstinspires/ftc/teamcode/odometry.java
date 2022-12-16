package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class odometry {
    //Odometry wheels
    private DcMotor encoderLeft, encoderRight, encoderAux;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double currentRightPos = 0, currentLeftPos = 0, currentAuxPos = 0, currentIMU = 0,  orientationChange = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double oldRightPos = 0, oldLeftPos = 0, oldAuxPos = 0, oldIMU = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public odometry(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderAux){
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderAux = encoderAux;
    }

    public void globalCoordinatePositionUpdate(double orientation){
        oldLeftPos = currentLeftPos;
        oldRightPos = currentRightPos;
        oldAuxPos = currentAuxPos;
        oldIMU = currentIMU;

        currentLeftPos = encoderLeft.getCurrentPosition();
        currentRightPos = -encoderRight.getCurrentPosition();
        currentAuxPos = encoderAux.getCurrentPosition();
        currentIMU = orientation;

        double leftChange = currentLeftPos - oldLeftPos;
        double rightChange = currentRightPos - oldRightPos;
        double auxChange = currentAuxPos - oldAuxPos;
        double IMUChange = currentIMU - oldIMU;
//
//        orientationChange = (rightChange - leftChange) / 27703.414;
//        robotOrientationRadians = ((robotOrientationRadians + orientationChange)); //using odometry
        orientationChange = Math.toRadians(IMUChange);
        robotOrientationRadians = Math.toRadians(orientation); //using imu

        double horizontalChange = auxChange - (orientationChange * 50); //9250 40.59 35.575 10.3429 133.23 9.7259

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (n*Math.cos(robotOrientationRadians) - p*Math.sin(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (n*Math.sin(robotOrientationRadians) + p*Math.cos(robotOrientationRadians));
    }

    public double x(){ return robotGlobalXCoordinatePosition; }

    public double y(){ return robotGlobalYCoordinatePosition; }

    public double h(){ return Math.toDegrees(robotOrientationRadians); }

    public void stop(){ isRunning = false; }

}
