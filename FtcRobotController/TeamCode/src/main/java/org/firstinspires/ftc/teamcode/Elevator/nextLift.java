package org.firstinspires.ftc.teamcode.Elevator;


import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.PID;

public class nextLift extends Subsystem {

    public static final nextLift INSTANCE = new nextLift();
    public nextLift() { }
    public MotorEx motor;
    String name = "EH";
    public PIDFController controller;

//    public String EH = "EH";
    public Command setTolerance(int tolerance){
        return new LambdaCommand().setStart(()->{
            controller.setSetPointTolerance(tolerance);
        });
    }

    public Command toHeight(double height, double sec) {
        return new ParallelRaceGroup(
                new RunToPosition(motor,height,controller,this),
        new Delay(sec)
        );

    }

//    @Override
//    public Command getDefaultCommand() {
//        return new HoldPosition(EH, controller, this);
//    }

    @Override
    public void initialize() {
        motor = new MotorEx(name);
        motor.resetEncoder();
        motor.reverse();
        controller = new PIDFController(0.001, 0.02, 0.0, new StaticFeedforward(0.5));
        setTolerance(150);

    }
}
