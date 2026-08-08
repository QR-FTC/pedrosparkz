package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "motor_practice")
public class layering2 extends OpMode {
    private layeringone bot;
    @Override
    public void init() {
        bot = new layeringone(hardwareMap);
        bot.init();
        telemetry.addLine("Initialized. ready to start");
        telemetry.update();
    }
    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            bot.increasePower();
        } else if (gamepad1.dpadDownWasPressed()) {
            bot.decreasePower();
        }
        bot.updateMotor();
        if (gamepad1.a) {
            bot.openServo();
        } else if (gamepad1.b) {
            bot.closeServo();
        }
        telemetry.addData("Switch", bot.isSwitchPressed() ? "pressed (run)" : "released (stop)");
        telemetry.addData("Target power", "%.2f", bot.getTargetPower());
        telemetry.addData("Motor power", "%.2f", bot.getAppliedPower());
        telemetry.addData("Encoder position", bot.getEncoderPosition());
        telemetry.addData("Servo position", "%.2f", bot.getServoPosition());
        telemetry.update();
    }
}