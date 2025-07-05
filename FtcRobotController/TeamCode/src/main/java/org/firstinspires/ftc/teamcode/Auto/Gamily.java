package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;

import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.controllers.Controller;
import com.rowanmcalpin.nextftc.core.units.TimeSpan;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
//import org.firstinspires.ftc.teamcode.Elevator.nextIntakeAngle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Elevator.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.Elevator.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;
import org.firstinspires.ftc.teamcode.Auto.AutoCommands;

@Autonomous(name = "auto :DDD")
public class Gamily extends PedroOpMode {

    //TODO: should be in nextlift class?
//    public static final nextLift lift = new nextLift();
//    public static final nextIntakeAngle intakeAngle = new nextIntakeAngle();

    public Gamily() {
        super(nextLift.INSTANCE, nextIntakeAngle.INSTANCE, ElevatorAngleNext.INSTANCE);
    }
    AutoCommands commands = new AutoCommands();

    Follower follower;
    Pose startPose = new Pose(8.67,86,0);
    Pose scorePose = new Pose(14.11,126.53,-28);
    Pose sample1 = new Pose();

    private PathChain preload, take1;

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        take1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),-17)
                .build();

    }


    public Command preload()  {
        return new SequentialGroup(
                new FollowPath(preload),
                commands.sampleToBasket()
        );
    }

    public Command take1(){
        return new SequentialGroup(
            new FollowPath(take1),
                commands.sampleToBasket()
        );
    }

    @Override
    public void onStartButtonPressed() {
        preload().invoke();
    }
}


