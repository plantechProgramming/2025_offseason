package org.firstinspires.ftc.teamcode.Elevator;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.qualcomm.robotcore.hardware.CRServo;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class nextIntakeClaw extends Subsystem {
    // BOILERPLATE
    public static final nextIntakeClaw INSTANCE = new nextIntakeClaw();
    private nextIntakeClaw() { }

    // same as TeleOp roni2_intake
    public Servo clawAngle;
    public String claw = "intake";
    public PIDFController PID_intA = new PIDFController(0.005, 0, 0, new StaticFeedforward(0));

    public Command close(){
        return new ServoToPosition(clawAngle, // SERVO TO MOVE
                1,// POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command open(){
        return new ServoToPosition(clawAngle, // SERVO TO MOVE
                0,// POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    @Override
    public void initialize() {

        clawAngle = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,claw);
    }

}