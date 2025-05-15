package org.firstinspires.ftc.teamcode.Elevator;


import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class nextLift extends Subsystem {

    public static final nextLift INSTANCE = new nextLift();
    private nextLift() { }
    public MotorEx motor;
    public double wantedHeight = 0;

    public PIDFController controller = new PIDFController(0.01, 0.01, 0, new StaticFeedforward(0.1));
    public String name = "EH";




    public Command toLow() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHigh() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                2900, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public NullCommand setWantedHeight(double x){
        this.wantedHeight = x;
        return null;
    }

    public Command toPosTicks() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                wantedHeight, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }



    @Override
    public void initialize() {
        motor = new MotorEx(name);
        motor.reverse();
    }


}
