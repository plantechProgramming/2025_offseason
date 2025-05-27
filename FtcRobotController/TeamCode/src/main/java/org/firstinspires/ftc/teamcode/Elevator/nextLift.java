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
    public MotorEx EH, EA;
    public double wantedHeight = 0;
    public double sec;
    public static double PID_EA_KP = 0.0001;
    public static double PID_EA_KI = 0;
    public static double PID_EA_KD = 0.001;
    public static double PID_EA_KF = 0;
    public static double PID_EH_KP = 0.01;
    public static double PID_EH_KI = 0.01;
    public static double PID_EH_KD = 0;
    public static double PID_EH_KF = 0.1;


    public PIDFController controller = new PIDFController(0.002, 0.02, 0, new StaticFeedforward(0.5));



//    public String EH = "EH";

    public Command setTolerance(int tolerance){
        return new LambdaCommand().setStart(()->{
            controller.setSetPointTolerance(tolerance);
        });
    }

    public Command toHeight(double height) {
//        if (height == 0.0){
            return new RunToPosition(EH,height,controller,this);
//        }
//        else {
//            return new SequentialGroup(
//                    new RunToPosition(EH, height, controller, this),
//
//                    new HoldPosition(EH,controller,this).endAfter(0));


//                 new ParallelRaceGroup(

//                        new Delay(0),
//                        new HoldPosition(EH,controller,this)
//                ));
//            return new RunToPosition(EH,height,controller,this);
//            new HoldPosition(EH,controller,this)

//        }
    }






//    @Override
//    public Command getDefaultCommand() {
//        return new HoldPosition(EH, controller, this);
//    }


    @Override
    public void initialize() {
        EH = new MotorEx("EH");
        EH.resetEncoder();
        EH.reverse();

        EA = new MotorEx("EA");
        EA.resetEncoder();
        EA.reverse();
        setTolerance(150);
    }


}
