package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;

import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.controllers.Controller;
import com.rowanmcalpin.nextftc.core.units.TimeSpan;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
//import org.firstinspires.ftc.teamcode.Elevator.nextIntakeAngle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;

@Autonomous(name = "nextRoni")
public class nextAuto extends NextFTCOpMode {
    public nextAuto() {
        super( nextLift.INSTANCE);
    }
    public Command firstRoutine()  {
//        return new ParallelGroup(
//                new SequentialGroup(
//                        nextLift.INSTANCE.setWantedHeight(2800),
//                        new Delay(5),
//                        nextLift.INSTANCE.setWantedHeight(0)
//                ),
//                nextLift.INSTANCE.toPosTicks().perpetually()
//        );
        return new SequentialGroup(
                new ParallelGroup(
                        nextLift.INSTANCE.toHigh().perpetually().endAfter(5)
                ),
            new Delay(5),
            nextLift.INSTANCE.toLow()
        );
    }



    @Override
    public void onStartButtonPressed() {

        firstRoutine().invoke();
    }
}













