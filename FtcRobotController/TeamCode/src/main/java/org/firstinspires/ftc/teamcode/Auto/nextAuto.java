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
    public static final nextLift lift = new nextLift();

    public nextAuto() {
        super(lift);
    }
    public Command firstRoutine()  {
        return new ParallelGroup(
            lift.toHeight(5,2900),
            lift.toAngle(1000)
        );
    }



    @Override
    public void onStartButtonPressed() {

        firstRoutine().invoke();
    }
}













