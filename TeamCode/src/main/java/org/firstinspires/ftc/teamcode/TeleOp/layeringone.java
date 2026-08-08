package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class layeringone {
    private final HardwareMap hardwareMap;
    private DcMotor practiceMotor;
    private Servo practiceServo;
    private TouchSensor digitalTouch;
    private double targetPower = 0.0;
    public layeringone(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void init() {
        mapHardware();
        configureMotor();
    }
    private void mapHardware() {
        practiceMotor = hardwareMap.get(DcMotor.class, "Practice");
        practiceServo = hardwareMap.get(Servo.class, "Servo");
        digitalTouch  = hardwareMap.get(TouchSensor.class, "digitalTouch");
    }
    private void configureMotor() {
        practiceMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        practiceMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        practiceMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setTargetPower(double power) {
        // keep the target within the motor's valid range
        targetPower = Math.max(0.0, Math.min(1.0, power));
    }
    public void increasePower() {
        setTargetPower(targetPower + 0.1);
    }
    public void decreasePower() {
        setTargetPower(targetPower - 0.1);
    }
    public void brake() {
        setTargetPower(0.0);
    }
    public void updateMotor() {
        practiceMotor.setPower(isSwitchPressed() ? targetPower : 0.0);
    }
    public boolean isSwitchPressed() {
        return digitalTouch.isPressed();
    }
    public double getTargetPower() {
        return targetPower;
    }
    public double getAppliedPower() {
        return practiceMotor.getPower();
    }
    public int getEncoderPosition() {
        return practiceMotor.getCurrentPosition();
    }
    public void openServo() {
        setServo(1.0);
    }
    public void closeServo() {
        setServo(0.0);
    }
    public void setServo(double position) {
        practiceServo.setPosition(position);
    }
    public double getServoPosition() {
        return practiceServo.getPosition();
    }
}