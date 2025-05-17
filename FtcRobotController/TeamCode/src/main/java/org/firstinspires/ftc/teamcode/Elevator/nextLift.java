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

public class nextLift extends Subsystem {

    public static final nextLift INSTANCE = new nextLift();
    public nextLift() { }
    public MotorEx EH, EA;
    public double wantedHeight = 0;
    public double sec;

    public PIDFController controller = new PIDFController(0.01, 0.01, 0, new StaticFeedforward(0.1));
    public PIDFController PID_EA = new PIDFController(0.01, 0.01, 0, new StaticFeedforward(0.1));
//    public String EH = "EH";


    public Command toHeight(double sec,double height) {
        return new SequentialGroup(
                new RunToPosition(EH,height,controller,this).perpetually().endAfter(sec),
                // why doesn't this work without the delay?
                new Delay(sec)
        );
    }


    public Command toAngle(double angle){
        return new RunToPosition(EA,angle,PID_EA,this);
    }


    @Override
    public void initialize() {
        EH = new MotorEx("EH");
        EH.resetEncoder();
        EH.reverse();

        EA = new MotorEx("EA");
        EA.resetEncoder();
        EA.reverse();
    }


}
