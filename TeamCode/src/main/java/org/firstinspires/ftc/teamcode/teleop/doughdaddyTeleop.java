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

@TeleOp(name="kaiden", group="Linear Opmode")

public class doughdaddyTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo larm = hardwareMap.get(Servo.class, "larm");
        Servo rarm = hardwareMap.get(Servo.class, "rarm");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a) { //deposit position
                larm.setPosition(0.62);
                rarm.setPosition(0.38);
            }
            if (gamepad2.b) { //intake position
                larm.setPosition(0.82);
                rarm.setPosition(0.18);
            }



        }
    }
}