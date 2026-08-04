package org.firstinspires.ftc.teamcode.Layers.Physical;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class layermotor {
    DcMotorEx motor;
    Telemetry telemetry;

    final int TICKS = 28;
    double ratio;

    public layermotor(Telemetry telemetry, HardwareMap hardwareMap, String name, double ratio) {

       motor =  hardwareMap.get(DcMotorEx.class, name);
       motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       this.ratio = ratio;
       this.telemetry = telemetry;
    }

    public void run_using_position( double rotation) {
        motor.setTargetPosition((int) (rotation * ratio * TICKS));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);

    }
    public void telemetry() {
        telemetry.addData("Motor Position", motor.getCurrentPosition());
        double rotations = motor.getCurrentPosition() / (ratio * TICKS);
        telemetry.addData("Motor Rotations",rotations);
        telemetry.update();
    }

    public boolean isBusy(){
        return motor.isBusy();
    }

}