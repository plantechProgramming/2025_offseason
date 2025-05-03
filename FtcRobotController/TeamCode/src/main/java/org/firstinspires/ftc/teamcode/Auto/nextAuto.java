package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
//import org.firstinspires.ftc.teamcode.Elevator.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;

@Autonomous(name = "nextRoni")
public class nextAuto extends NextFTCOpMode {
    public nextAuto() {
        super( nextLift.INSTANCE);
    }
    public Command firstRoutine() {
        return new SequentialGroup(
                nextLift.INSTANCE.toHigh(),
                nextLift.INSTANCE.holdForTime(500)
        );
    }

    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}













