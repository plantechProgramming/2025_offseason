package org.firstinspires.ftc.teamcode.Elevator.intake;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class nextIntakeClaw extends Subsystem {
    // BOILERPLATE
    public static final nextIntakeClaw INSTANCE = new nextIntakeClaw();
    private nextIntakeClaw() { }

    // same as TeleOp roni2_intake
    public Servo intake;
    public String clawL = "intake";

    public PIDFController PID_intA = new PIDFController(0.005, 0, 0, new StaticFeedforward(0));
//intA = intake Angle
    public Command close(){
        return new ServoToPosition(intake, // SERVO TO MOVE
                0.64,// POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command open(){
        return new ServoToPosition(intake, // SERVO TO MOVE
                0,// POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    @Override
    public void initialize() {
        intake = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,clawL);
        intake.setPosition(0.7);
    }

}