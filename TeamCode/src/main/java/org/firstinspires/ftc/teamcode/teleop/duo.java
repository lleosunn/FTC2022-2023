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
    private DcMotor arm = null;

    Servo lclaw;
    Servo rclaw;

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
        arm = hardwareMap.get(DcMotor.class, "arm");

        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        Servo odo1 = hardwareMap.get(Servo.class, "odo1");
        Servo odo2 = hardwareMap.get(Servo.class, "odo2");


        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("lift1", lift1.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double modifier;

            if (gamepad1.right_trigger > 0) {
                modifier = 0.3;
            } else if (gamepad1.left_trigger > 0) {
                modifier = 1;
            } else modifier = 0.5;

            fl.setPower((modifier*1.15)*(y + x + turn));
            fr.setPower(modifier*(y - x - turn));
            bl.setPower(modifier*(y - x + turn));
            br.setPower(modifier*(y + x - turn));

            if(gamepad2.y) { //reset lift
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if(gamepad2.x) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
            }

            if (gamepad2.a) { //deposit position
                arm.setTargetPosition(600);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.4);
                clawClose();
            }
            if (gamepad2.b) { //intake position
                arm.setTargetPosition(38);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.3);
                clawClose();
            }
            if (gamepad2.right_trigger > 0){ //claw close
                clawClose();
            }
            if (gamepad2.left_trigger > 0){ //claw open
                clawOpen();
            }

            if(gamepad2.dpad_up) { // high
                lift1.setTargetPosition(900);
                lift2.setTargetPosition(900);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if(gamepad2.dpad_right) { // Medium
                lift1.setTargetPosition(300);
                lift2.setTargetPosition(300);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if(gamepad2.dpad_left) { // Low
                arm.setTargetPosition(280);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.4);
            }
            if(gamepad2.dpad_down) { //lift down
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if(gamepad2.left_stick_y > 0.5){
                lift1.setTargetPosition(lift1.getCurrentPosition()-50);
                lift2.setTargetPosition(lift2.getCurrentPosition()-50);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if(gamepad2.left_stick_y < -0.5){
                lift1.setTargetPosition(lift1.getCurrentPosition()+100);
                lift2.setTargetPosition(lift2.getCurrentPosition()+100);
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            }
            if (gamepad2.right_stick_x < -0.5){
                arm.setTargetPosition(arm.getCurrentPosition()-25);
                arm.setPower(0.5);
            }
            if (gamepad2.right_stick_x > 0.5){
                arm.setTargetPosition(arm.getCurrentPosition()+25);
                arm.setPower(0.5);
            }
            if(gamepad1.a){
                odo1.setPosition(0.55);
                odo2.setPosition(0.45);
            }
            if(gamepad1.b){
                odo1.setPosition(1);
                odo2.setPosition(0);
            }


        }
    }
    public void clawOpen() {
        lclaw.setPosition(0.25);
        rclaw.setPosition(0.75);
    }
    public void clawClose() {
        lclaw.setPosition(0.47);
        rclaw.setPosition(0.53);
    }
}