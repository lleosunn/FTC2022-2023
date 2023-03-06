package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="WOODsolo", group="Linear Opmode")

public class WOODsolo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor arm = null;

    Servo claw;
    Servo wrist;
    Servo guider;


    private DistanceSensor clawDistance;

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

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        guider = hardwareMap.get(Servo.class, "guider");

        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");

        //robot hardware
        RobotHardware robot = new RobotHardware(fl, fr, bl, br, lift1, lift2, arm, claw, wrist, guider);
        robot.innitHardwareMap();

        waitForStart();
        runtime.reset();
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        arm.setTargetPosition(0);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(0);
        lift2.setPower(0);
        arm.setPower(0);

        double distance = 0;

        while (opModeIsActive()) {
            distance = clawDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("lift1", lift1.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("distance", distance);
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double modifier = 1;

            fl.setPower(modifier*(y + x + turn));
            fr.setPower(modifier*(y - x - turn));
            bl.setPower(modifier*(y - x + turn));
            br.setPower(modifier*(y + x - turn));

            if (gamepad1.right_bumper) {
                if (distance < 30) {
                    robot.clawClose();
                    sleep(200);
                    robot.setLift(900, 1);
                    robot.setArm(600, 0.8);
                    sleep(200);
                    robot.wristTurn();
                }
            }

            if (gamepad1.left_stick_button) {
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setPower(0);
                lift2.setPower(0);
            }
            if (gamepad1.dpad_up) {
                robot.setLift(900, 1);
            }
            if (gamepad1.dpad_right) {
                robot.setLift(380, 1);
            }
            if (gamepad1.dpad_left) {
                robot.setArm(730, 0.8);
            }
            if (gamepad1.dpad_down) {
                robot.setLift(0, 0.5);
            }
            if (gamepad1.right_stick_button) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
            }
            if (gamepad1.a) {
                robot.setArm(618, 0.8);
            }
            if (gamepad1.b) {
                robot.setLift(0, 0.5);
                robot.setArm(0, 0.5);
                robot.wristReset();
                robot.clawClose();
            }

            if (gamepad1.right_trigger > 0.5) {
                robot.clawClose();
            }
            if (gamepad1.left_trigger > 0.99) {
                robot.clawOpen();
            }
            if (gamepad1.left_trigger > 0.1) {
                robot.guiderSet();
            } else if (arm.getCurrentPosition() > 720) {
                robot.guiderFlat();
            } else robot.guiderBack();
            if (gamepad1.x) {
                robot.wristTurn();
            }
            if (gamepad1.y) {
                robot.wristReset();
            }

        }
    }
}

