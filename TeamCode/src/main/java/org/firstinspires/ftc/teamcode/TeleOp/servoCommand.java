package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.TeleOp.intakeSubsystem;



public class servoCommand extends Command {
    intakeSubsystem intake;
    ElapsedTime timer;

    boolean stop = false;
    double power= 0.1;




    public servoCommand() {
        intake = new intakeSubsystem();
        timer = new ElapsedTime();
        intake.initialize();
        requires(intake);
    }

    @Override
    public boolean isDone() {
        return intake.intakeTouchSensorCheck() || stop; // whether or not the command is done
    }

    @Override
    public void start() {
        intake.intake(power);
        timer.reset();
        // executed when the command begins
    }

    @Override
    public void update() {
        if(timer.milliseconds()>= 3000) {
           stop = true;

        }


        // executed on every update of the command
    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
    }
}


