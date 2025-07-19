package org.firstinspires.ftc.teamcode.Elevator.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.Controller;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.sun.tools.javac.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Objects;

public class nextIntakeClaw extends Subsystem {
    // BOILERPLATE
    public static final nextIntakeClaw INSTANCE = new nextIntakeClaw();
    private nextIntakeClaw() { }

    // same as TeleOp roni2_intake
    public CRServo intakeR, intakeL;
    public String clawL = "IntakeR";
    public String clawR = "IntakeL";

    private List<CRServo> intake;
    ElapsedTime runtime = new ElapsedTime();
    public PIDFController PID_intA = new PIDFController(0.005, 0, 0, new StaticFeedforward(0));
//intA = intake Angle
    public Command runCRservoForTime(List<CRServo> crServos, double sec, double power) {
        return new LambdaCommand()
                .setStart(() -> {
                    runtime.reset();
                })
                .setUpdate(() -> {
                    for (CRServo crservo : crServos){
                        crservo.setPower(power);
                    }
                })
                .setStop(interrupted -> {
                    for (CRServo crservo : crServos){
                        crservo.setPower(0);
                    }
                })
                .setIsDone(() -> runtime.seconds() >= sec); // Returns if the command has finished


    }


    public Command in(double sec){
        return runCRservoForTime(intake, sec, 1);
    }

    public Command out(double sec){
        return runCRservoForTime(intake, sec, -1);
    }
    @Override
    public void initialize() {
        intakeL = OpModeData.INSTANCE.getHardwareMap().get(CRServo.class,clawL);
        intakeR = OpModeData.INSTANCE.getHardwareMap().get(CRServo.class,clawR);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = List.of(intakeR, intakeL);
    }

}