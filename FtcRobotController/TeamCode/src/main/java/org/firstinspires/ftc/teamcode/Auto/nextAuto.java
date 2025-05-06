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
//    Controller controller = new Controller() {
//        @Override
//        public double getTarget() {
//            return 0;
//        }
//
//        @Override
//        public void setTarget(double v) {
//
//        }
//
//        @Override
//        public double getSetPointTolerance() {
//            return 0;
//        }
//
//        @Override
//        public void setSetPointTolerance(double v) {
//
//        }
//
//        @Override
//        public double calculate(double v) {
//            return 0;
//        }
//
//        @Override
//        public void reset() {
//
//        }
//
//        @Override
//        public boolean atTarget(double v) {
//            return false;
//        }
//    };

    public nextAuto() {
        super( nextLift.INSTANCE);
    }
    public Command firstRoutine()  {
        return new SequentialGroup(
                nextLift.INSTANCE.toHigh()

        );
    }

    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}













