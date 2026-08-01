package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Layers.Physical.hardwareMapFile;


import org.firstinspires.ftc.teamcode.Layers.Physical.advancedcrServo;


public class multiRollerIntake {
    hardwareMapFile hardwareMapFile0;

    CRServo frontRollerCRservo;
    CRServo backRollerCRservo;
    advancedcrServo frontRollerServo;
    advancedcrServo backRollerServo;


    public multiRollerIntake() { }

    public void initialize() {
        frontRollerServo = new advancedcrServo(hardwareMapFile0.initializeCRservo());
        backRollerServo = new advancedcrServo(hardwareMapFile0.initializeCRservo1());
        backRollerServo.setPower(0);
        frontRollerServo.setPower(0);
        backRollerServo.setReversed(true);
        frontRollerServo.setReversed(false);

    }

    public void intake() {
        frontRollerServo.setPower(1);
        backRollerServo.setPower(1);
    }

    public void outtake() {
        frontRollerServo.setPower(-1);
        backRollerServo.setPower(-1);
    }

    public void stopIntake() {
        frontRollerServo.stop();
        backRollerServo.stop();
    }
    public void getIntakePower(Telemetry telemetry) {
        telemetry.addData("front roller", frontRollerServo.getPower());
        telemetry.addData("back roller",backRollerServo.getPower());
    }

    public void frontIntake() {
        frontRollerServo.setPower(1);
        backRollerServo.setPower(0);
    }

    public void backIntake() {
        frontRollerServo.setPower(0);
        backRollerServo.setPower(1);
    }

    public void revFrontIntake() {
        frontRollerServo.setPower(-1);
        backRollerServo.setPower(0);
    }

    public void revBackIntake() {
        frontRollerServo.setPower(0);
        backRollerServo.setPower(-1);
    }

    public void halfBackFrontIntake() {
        backRollerServo.setPower(1);
        frontRollerServo.setPower(0.5);
    }

    public void halfFrontBackIntake() {
        backRollerServo.setPower(0.5);
        frontRollerServo.setPower(1);
    }

    public void revHalfFrontBackIntake() {
        backRollerServo.setPower(-0.5);
        frontRollerServo.setPower(-1);
    }
    public void revHalfBackFrontIntake() {
        backRollerServo.setPower(-1);
        frontRollerServo.setPower(-0.5);
    }










}
