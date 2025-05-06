package org.firstinspires.ftc.teamcode.Elevator;

import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.units.TimeSpan;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class nextLift extends Subsystem {

    public static final nextLift INSTANCE = new nextLift();
    private nextLift() { }
    public static double kP_next = 0.15;
    public static double kI_next = 0.05;
    public static double kD_next = 0.05;
    public PIDFController controller = new PIDFController(kP_next, kI_next, kD_next, new StaticFeedforward(0));
    public MotorEx motor;




    public Command toLow() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHigh() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                2950, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
//    public  Command  holdPos(long time){
//
////            double count = 0;
////            if(count <= 1) {
////                count += 1;
////
////            }count-=1;
////
////        return null;
//    }




//    public Command holdForTime(long x) {
//        return new SequentialGroup(
//                motor.setPower(controller.calculate(motor.getCurrentPosition())),
//                new Delay(x),
//              //  wait(x),
//                motor.setPower(0)
//        );
//    }

    @Override
    public void initialize() {
        motor = new MotorEx("EH");
        motor.reverse();
    }


}
