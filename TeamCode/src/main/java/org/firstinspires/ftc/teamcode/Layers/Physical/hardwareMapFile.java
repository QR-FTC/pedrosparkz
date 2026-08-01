package org.firstinspires.ftc.teamcode.Layers.Physical;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp.servo;


public class hardwareMapFile {
    HardwareMap hardwareMap;

    public void  initialize(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public DcMotor initializemotor() {
        DcMotor motor =  hardwareMap.get(DcMotor.class, "motor");
        return motor;

    }
    public DcMotor initializemotor1() {
        DcMotor motor1 =  hardwareMap.get(DcMotor.class, "motor1");
        return motor1;

    }

    public DcMotor initializemotor2() {
        DcMotor motor2 =  hardwareMap.get(DcMotor.class, "motor2");
        return motor2;

    }

    public DcMotor initializemotor3() {
        DcMotor motor3 =  hardwareMap.get(DcMotor.class, "motor3");
        return motor3;

    }

    public Servo initializeservo() {
        Servo servo = hardwareMap.get(Servo.class, "servo");
        return servo;

    }
    public Servo initializeservo1() {
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        return servo1;

    }
    public Servo initializeservo2() {
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        return servo2;

    }
    public CRServo initializeCRservo() {
        CRServo CRservo = hardwareMap.get(CRServo.class, "CRservo");
        return CRservo;

    }

    public CRServo initializeCRservo1() {
        CRServo CRservo1 = hardwareMap.get(CRServo.class, "CRservo1");
        return CRservo1;

    }

    public CRServo initializeCRservo2() {
        CRServo CRservo2 = hardwareMap.get(CRServo.class, "CRservo2");
        return CRservo2;


    }

    public GoBildaPinpointDriver initializepinpoint() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        return pinpoint;
    }

}
