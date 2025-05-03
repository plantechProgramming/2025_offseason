//package org.firstinspires.ftc.teamcode.Elevator;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
//import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
//import com.rowanmcalpin.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.ftc.OpModeData;
//import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
//
//
//public class nextIntakeAngle extends Subsystem {
//    // BOILERPLATE
//    public static final nextIntakeAngle INSTANCE = new nextIntakeAngle();
//    private nextIntakeAngle() { }
//    public CRServo servo;
//    public String intake_center_angle = "intA";
//
//
//    public Command Up() {
//        return new ServoToPosition(servo, // SERVO TO MOVE
//                1, // POSITION TO MOVE TO
//                this); // IMPLEMENTED SUBSYSTEM
//    }
//
//    public Command Down() {
//        return new ServoToPosition(servo, // SERVO TO MOVE
//                0, // POSITION TO MOVE TO
//                this); // IMPLEMENTED SUBSYSTEM
//    }
//
//
//    @Override
//    public void initialize() {
//        servo = OpModeData.INSTANCE.getHardwareMap().get(CRServo.class, intake_center_angle);
//    }
//
//}


