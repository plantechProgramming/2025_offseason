package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.OpMode;
import java.lang.Thread;

import org.firstinspires.ftc.teamcode.Elevator.ElevatorAuto;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.TeamCode.teamcode.MecanumDrive;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="roadrunner test", group="Robot")
@Config
public class roadrunnerAuto extends LinearOpMode{
//        public class Lift {
//            private DcMotorEx lift;
//
//            public Lift(HardwareMap hardwareMap) {
//                lift = hardwareMap.get(DcMotorEx.class, "EH");
//                lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//                lift.setDirection(DcMotorEx.Direction.REVERSE);
//                lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }
//            double count = 0;
//
//
//            public class LiftUp implements Action {
//                private boolean initialized = false;
//                double power;
//                double thresh = 40;
//                PID pid_EH = new PID(0.15,0.05,0.05,0,0);
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
////                    if (!initialized) {
////                        lift.setPower(0.8);
////                        initialized = true;
////                    }
//                    pid_EH.setWanted(2000);
//                    double pos = lift.getCurrentPosition();
//                    if (Math.abs(2000 - lift.getCurrentPosition()) < thresh){
//                        while (count < num){
//                            lift.setPower(0.5);
//                            count++;
//                        }
//                        packet.put("count", count);
//                        power = 0;
//                        lift.setPower(power);
//                        return false;
//
//                    }
//                    else{
//                        power = pid_EH.update(lift.getCurrentPosition());
//                        packet.put("liftPos", lift.getCurrentPosition());
//                        lift.setPower(power);
//                        return true;
//                    }
//                }
//            }
//
//            public Action liftUp() {
//                return new LiftUp();
//            }
//            public class LiftDown implements Action {
//                private boolean initialized = false;
//                double power;
//                double thresh = 40;
//                PID pid_EH = new PID(0.15,0.05,0.05,0,0);
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
////                    if (!initialized) {
////                        lift.setPower(0.8);
////                        initialized = true;
////                    }
//                    pid_EH.setWanted(0);
//                    double pos = lift.getCurrentPosition();
//                    if (Math.abs(0 - lift.getCurrentPosition()) < thresh){
//                        power = 0;
//                        lift.setPower(power);
//                        return false;
//                    }
//                    else{
//                        power = pid_EH.update(lift.getCurrentPosition());
//                        packet.put("liftPos", lift.getCurrentPosition());
//                        lift.setPower(power);
//                        return true;
//                    }
//                }
//            }
//            public Action liftDown() {
//                return new LiftDown();
//            }
//
////            public class LiftDown implements Action {
////                private boolean initialized = false;
////
////                @Override
////                public boolean run(@NonNull TelemetryPacket packet) {
////                    if (!initialized) {
////                        lift.setPower(-0.8);
////                        initialized = true;
////                    }
////
////                    double pos = lift.getCurrentPosition();
////                    packet.put("liftPos", pos);
////                    if (pos > 100.0) {
////                        return true;
////                    } else {
////                        lift.setPower(0);
////                        return false;
////                    }
////                }
////            }
////            public Action liftDown(){
////                return new LiftDown();
////            }
//        }


    public static double inPerTile = 24.3125;
    public static double num = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: find if same as normal competition tiles


        double pie = Math.PI;
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//        ElevatorAuto elevatorAuto =new ElevatorAuto();
         double inPerTileY = inPerTile*1.42;
//        Elevator lift = new Elevator(EA, EH, intake_center_angle, intake_left, intake_right, intake_AR, intAR, telemetry);

        double inPerTileX = inPerTile;
         VelConstraint velConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 0;
            }
            VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
                if (Math.abs(robotPose.position.x.value()) > 5.8*inPerTileY || Math.abs(robotPose.position.y.value()) > 5.8*inPerTileX) {
                    return 10.0;
                }
                return 60;
            };
        };

        waitForStart();
        if(isStopRequested()){
            return;
        }
        Thread lift = new Thread();
        lift.start();
//        elevatorAuto.Change_Height();
//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                         .lineToXLinearHeading(inPerTileX*2, pie/2)
//                        .lineToX(inPerTileX)
//                        .lineToY(inPerTileY)
//                        .lineToYLinearHeading(inPerTileY, pie/2)
//                        .splineTo(new Vector2d(0, inPerTileY), - Math.PI / 4)


//                        .strafeTo(new Vector2d(0.2*inPerTileX,-1.9*inPerTileY))
//                        .turn(Math.toRadians(-22))
//                        .waitSeconds(1)
//                        .turn(Math.toRadians(22))
//                        .turn(Math.toRadians(-30))
//                        .waitSeconds(1)

//                .build());


    }
    protected void end() {

    }
}