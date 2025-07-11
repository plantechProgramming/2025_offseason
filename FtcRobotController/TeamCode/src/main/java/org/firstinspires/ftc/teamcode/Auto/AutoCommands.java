package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
//import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;
import org.firstinspires.ftc.teamcode.Elevator.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeClaw;

public class AutoCommands extends NextFTCOpMode{
//    Elevator elevator = new Elevator(DcMotor EA, );

    public AutoCommands() {
        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE,nextIntakeClaw.INSTANCE);
    }
    nextLift lift = nextLift.INSTANCE;
    nextIntakeAngle intakeAngle = nextIntakeAngle.INSTANCE;
    ElevatorAngleNext elevatorAngle = ElevatorAngleNext.INSTANCE;
    nextIntakeClaw intakeClaw = nextIntakeClaw.INSTANCE;


    public Command takeSample(){
        return new SequentialGroup(
                new ParallelGroup(
                        lift.toHeight(2400,0.5),
                        intakeAngle.Down(),
                        intakeClaw.open()
                ),
                intakeClaw.close()
        );
    }
    public Command sampleToBasket(){
        return new SequentialGroup(


                elevatorAngle.toAngle(1700),
                lift.toHeight(2300,2),
            intakeAngle.Up(),
            intakeClaw.open(),



                lift.toHeight(0,0.5),
                elevatorAngle.toAngle(0)

             );
    }

    // not very useful



    @Override
    public void onStartButtonPressed() {

    }
}


