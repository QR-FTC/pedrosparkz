package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.intakeSubsystem;
import dev.nextftc.core.commands.utility.LambdaCommand;






public class servoCommand extends Command {
    intakeSubsystem intake;
    ElapsedTime timer;
    DcMotor motor;
    DcMotor motor1;
    boolean stop = false;
    double power= 0.1;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    double time = 5;
    double time2= 3;

    double command = 0;

    boolean times = true;

    public servoCommand() {
        intake = new intakeSubsystem();
        timer = new ElapsedTime();
        intake.intakeSubsystem(hardwareMap, telemetry, motor, motor1);
        intake.initialize();
        requires(intake);
        new SequentialGroup(IntakeCommand(), IntakeCommand2());
    }


    public Command IntakeCommand() {
        return new LambdaCommand()
                .setUpdate(() -> intake.intake(power))
                .setIsDone(() -> intake.intakeTouchSensorCheck())
                .endAfter(time);
    }
    public Command IntakeCommand2() {
        return new LambdaCommand()
                .setUpdate(() -> intake.intake(power))
                .endAfter(time2);
    }

    public Command AutoCommand(){
        return new SequentialGroup(IntakeCommand(), IntakeCommand2());
    }




    @Override
    public boolean isDone() {
        return stop; // whether or not the command is done
    }

    @Override
    public void start() {
        AutoCommand().start();
        timer.reset();

        // executed when the command begins
    }

    @Override
    public void update() {
        if(timer.milliseconds()>= 10000) {
            stop=true;
        }

    }

    public void stop(boolean interrupted) {
            intake.stop();
        }
        // executed when the command ends
}


