package org.firstinspires.ftc.teamcode.Elevator;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class ElevatorAngleNext  extends Subsystem {
    // BOILERPLATE
    public static final ElevatorAngleNext INSTANCE = new ElevatorAngleNext();
    private ElevatorAngleNext() { }
    public MotorEx EA;

    public PIDFController PID_EA = new PIDFController(0.0025, 0, 0, new StaticFeedforward(0));

    public Command setTolerance(int tolerance){
        return new LambdaCommand().setStart(()->{
            PID_EA.setSetPointTolerance(tolerance);
        });
    }
    public Command toAngle(double angle) {

        return new SequentialGroup(
                new RunToPosition(EA,angle,PID_EA,this)
//                new HoldPosition(EA,PID_EA,this).endAfter(sec)
        );

    }
    @Override
    public void initialize() {

        EA = new MotorEx("EA");
        EA.resetEncoder();
        EA.reverse();
        setTolerance(150);
    }

}
