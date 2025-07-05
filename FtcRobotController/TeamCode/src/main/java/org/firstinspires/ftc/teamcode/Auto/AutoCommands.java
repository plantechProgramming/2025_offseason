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
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
//import org.firstinspires.ftc.teamcode.Elevator.nextIntakeAngle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Elevator.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.Elevator.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;
import org.firstinspires.ftc.teamcode.Elevator.nextIntakeClaw;

public class AutoCommands extends NextFTCOpMode{

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
                        lift.toHeight(2400),
                        intakeAngle.Down(),
                        intakeClaw.open()
                ),
                intakeClaw.close()
        );
    }
    public Command sampleToBasket(){
        return new SequentialGroup(
            new ParallelGroup(
                elevatorAngle.toAngle(90,2),
                lift.toHeight(2400)
            ),
            intakeClaw.open(),
            intakeAngle.Up(),
            lift.toHeight(0),
            elevatorAngle.toAngle(0,2)
            );
    }

    // not very useful
    public Command score(){
        return new SequentialGroup(
                takeSample(),
                sampleToBasket()
        );
    }


    @Override
    public void onStartButtonPressed() {

    }
}


