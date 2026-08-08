package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.TeleOp.ServoLogical;
public class grabeCommand extends Command {

    ServoLogical grade;
    ElapsedTime timmer = new ElapsedTime();
    public grabeCommand() {
        grade = new ServoLogical();
        requires();
        // setInterrptuptible(true); // this is the default, so you don't need to specify
    }

    @Override
    public void start() {
        grade.grab();
        timmer.reset();
        // executed when the command begins
    }

    @Override
    public boolean isDone() {
        return grade.isGrabeLimitPressed(); // whether or not the command is done
    }

    @Override
    public void update() {
        // if finished or timed out
        if (timmer.milliseconds() > 200) {

        }
        // executed on every update of the command
    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
    }
}
