package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;

import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeClaw;
import org.firstinspires.ftc.teamcode.Elevator.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;

@Autonomous(name = "emily")
public class nextAuto extends NextFTCOpMode {

    public nextAuto() {
        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE, nextIntakeClaw.INSTANCE);
    }
    public Command firstRoutine()  {

        return new SequentialGroup(
                ElevatorAngleNext.INSTANCE.toAngle(1600, 1),
                new Delay(1),
                nextLift.INSTANCE.toHeight(2300,1),
                new Delay(1),
                nextIntakeAngle.INSTANCE.Up(),
                new Delay(0.5),
                nextIntakeClaw.INSTANCE.out(1.5)
                );

    }

    @Override
    public void onStartButtonPressed() {

        firstRoutine().invoke();
    }
}













