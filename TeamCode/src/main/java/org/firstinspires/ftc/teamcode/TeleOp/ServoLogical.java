package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Layers.Physical.ServoPhysical;
import dev.nextftc.core.subsystems.Subsystem;
public class ServoLogical implements Subsystem {
    // 1. Hardware definition
    private Servo servo;
    private ServoPhysical claw;
    private ServoPhysical leftclaw;
    private ServoPhysical rightclaw;
    private ServoPhysical wrist;
    private TouchSensor grabeLimit;
    // 2. States
    private final double open=0;
    private final double grabePosition = 0.5;
    private final double dropPosition = 0.0;
    // 3. Constructor and initialisation
    public ServoLogical(HardwareMap hmap){
        grabeLimit = hmap.get(TouchSensor.class, "limit");
        claw = new ServoPhysical(servo);
        leftclaw = new ServoPhysical(servo);
        rightclaw = new ServoPhysical(servo);
        wrist = new ServoPhysical(servo);
        leftclaw.setPosition(dropPosition);
        rightclaw.setPosition(dropPosition);
        claw.UnitValues(0,300,0,1);
    }
    public ServoLogical() {

    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    //4. List of APIs
    public void grab(){
        leftclaw.setPosition(grabePosition);
        rightclaw.setPosition(grabePosition);
    }
    public void drop(){
        leftclaw.setPosition(dropPosition);
        rightclaw.setPosition(dropPosition);
    }
    public void store () {
        leftclaw.setPosition(dropPosition);
        rightclaw.setPosition(dropPosition);
        claw.setMappedPosition(180);
        leftclaw.setPosition(grabePosition);
        rightclaw.setPosition(grabePosition);
    }
    public boolean isGrabeLimitPressed()
    {
        return grabeLimit.isPressed();
    }
    // 5. Update loop
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
