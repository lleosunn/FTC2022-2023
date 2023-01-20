package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo", group="Linear Opmode")

public class servo extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo claw = hardwareMap.get(Servo.class, "claw");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                claw.setPosition(1);
            }
            if (gamepad1.b){
                claw.setPosition(0);
            }
        }
    }
}

