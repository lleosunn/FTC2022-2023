package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

//simple functions
public class motorblock {
    DcMotor tl;
    DcMotor tr;
    DcMotor bl;
    DcMotor br;

    public motorblock(DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br) {
        this.tl = tl;
        this.tr = tr;
        this.bl = bl;
        this.br = br;
    }

    public void leftturn(double power) {
        tl.setPower(-power);
        tr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }

    public void rightturn(double power) {
        tl.setPower(power);
        tr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void forward(double power) {
        tl.setPower(power);
        tr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }

    public void backward(double power) {
        tl.setPower(-power);
        tr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }

    public void left(double power) {
        tl.setPower(-power);
        tr.setPower(-power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void right(double power) {
        tl.setPower(power);
        tr.setPower(power);
        bl.setPower(-power);
        br.setPower(-power);
    }

    public void stop() {
        tl.setPower(0);
        tr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        tl.setMode(mode);
        tr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }
    public void setDrivetrainTarget(int tlTarget, int trTarget, int blTarget, int brTarget) {
        tl.setTargetPosition(tlTarget);
        tr.setTargetPosition(trTarget);
        bl.setTargetPosition(blTarget);
        br.setTargetPosition(brTarget);
    }

    public void setDrivetrainPower(double tlPower, double trPower, double blPower, double brPower) {
        tl.setPower(tlPower);
        tr.setPower(trPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void rightDistance(double power, double differenceSide, double multiplier, double angle) {
        double tlSide = (multiplier*(-(power + (0.008 * angle))));
        double trSide = (multiplier*(-(power + (0.008 * angle))));
        double blSide = (multiplier*((power - (0.008 * angle))));
        double brSide = (multiplier*((power - (0.008 * angle))));
        tl.setPower(tlSide);
        tr.setPower(trSide);
        bl.setPower(blSide);
        br.setPower(brSide);
        if (-0.5 < differenceSide && differenceSide < 0.5) {
            leftturn(0.02*angle);
        }
    }

    public void frontDistance(double power, double differenceStraight, double multiplier, double angle) {
        double tlStraight = (multiplier*(-(power + (0.008 * angle))));
        double trStraight = (multiplier*((power - (0.008 * angle))));
        double blStraight = (multiplier*(-(power + (0.008 * angle))));
        double brStraight = (multiplier*((power - (0.008 * angle))));
        tl.setPower(tlStraight);
        tr.setPower(trStraight);
        bl.setPower(blStraight);
        br.setPower(brStraight);
        if (-0.5 < differenceStraight && differenceStraight < 0.5) {
            leftturn(0.02*angle);
        }
    }
    public void leftDistance(double power, double differenceSide, double multiplier, double angle) {
        double tlSide = (multiplier*((power - (0.008 * angle))));
        double trSide = (multiplier*((power - (0.008 * angle))));
        double blSide = (multiplier*(-(power + (0.008 * angle))));
        double brSide = (multiplier*(-(power + (0.008 * angle))));
        tl.setPower(tlSide);
        tr.setPower(trSide);
        bl.setPower(blSide);
        br.setPower(brSide);
        if (-0.5 < differenceSide && differenceSide < 0.5) {
            leftturn(0.02*angle);
        }
    }
}