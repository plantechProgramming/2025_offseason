package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
//import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;

@Autonomous(name = "nextRoni")
public class nextAuto extends NextFTCOpMode {

    //TODO: should be in nextlift class?
//    public static final nextLift lift = new nextLift();
//    public static final nextIntakeAngle intakeAngle = new nextIntakeAngle();

    public nextAuto() {
        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE);
    }
    public Command firstRoutine()  {

        return new ParallelGroup(
                ElevatorAngleNext.INSTANCE.toAngle(1400,2),
                nextLift.INSTANCE.toHeight(1400)

//                nextLift.INSTANCE.toHeight(2900),




//                nextLift.INSTANCE.toHeight(3,1000),
//                nextLift.INSTANCE.toAngle(3, 700)


                );

    }



    @Override
    public void onStartButtonPressed() {

        firstRoutine().invoke();
    }
}













