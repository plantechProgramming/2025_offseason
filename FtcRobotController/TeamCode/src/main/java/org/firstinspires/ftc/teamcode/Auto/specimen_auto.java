package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Elevator.ElevatorAngleNext;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeAngle;
import org.firstinspires.ftc.teamcode.Elevator.intake.nextIntakeClaw;
import org.firstinspires.ftc.teamcode.Elevator.nextLift;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "roni ;)")
public class specimen_auto extends PedroOpMode {

    //TODO: should be in nextlift class?
      public static final nextLift lift = new nextLift();
      public static final nextIntakeAngle intakeAngle = new nextIntakeAngle();

    public specimen_auto() {
          super(nextLift.INSTANCE,  nextIntakeAngle.INSTANCE,ElevatorAngleNext.INSTANCE, nextIntakeClaw.INSTANCE);
      }
    AutoCommands commands = new AutoCommands();



    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose startPose = new Pose(8.7,80.75 ,Math.toRadians(0));
    private final Pose scorePose = new Pose(10,128,Math.toRadians(0));
    private final Pose sample1 = new Pose(24,120,0);
    private final Pose parkPose = new Pose(10, 15.5, Math.toRadians(0));    // Parking position

//    private final Pose startPose = new Pose(134.926487747958, 55.953325554259045, 0);
//    private final Pose scorePose = new Pose(128.89, 16.47, -45);
//    private final Pose sample1 = new Pose(119, 23, 0);
//    private final Pose parkPose = new Pose(83, 45, Math.toRadians(90));    // Parking position

    private Path park;
    private PathChain  scorePreload, grabPickup1, take2, take3, scorePickup1, score2, score3;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();
//
//
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(sample1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading())
                .build();
        park = new Path(new BezierLine(new Point(startPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());

    }


    public Command preload()  {
        return new SequentialGroup(
                new FollowPath(scorePreload)
        );
    }

//    public Command take1(){
//        return new SequentialGroup(
//            new FollowPath(take1),
//                commands.sampleToBasket()
//        );
//    }
@Override
public void onInit() {
    follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
    follower.setStartingPose(startPose);
    buildPaths();
}

    @Override
    public void onStartButtonPressed() {
        preload().invoke();
    }
}


//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0: // Move from start to scoring position
//                follower.followPath(scorePreload,true);
//                setPathState(1);
//                break;
////
////            case 1: // Wait until the robot is near the scoring position
////                if (!follower.isBusy()) {
////                    follower.followPath(grabPickup1, true);
////                    setPathState(2);
////                }
////                break;
////
////            case 2: // Wait until the robot is near the first sample pickup position
////                if (!follower.isBusy()) {
////                    follower.followPath(scorePickup1, true);
////                    setPathState(3);
////                }
////                break;
////
////
////
////            case 7: // Wait until the robot returns to the scoring position
////                if (!follower.isBusy()) {
////                    follower.followPath(park, true);
////                    setPathState(8);
////                }
////                break;
//
//            case 1: // Wait until the robot is near the parking position
//                if (!follower.isBusy()) {
//                    setPathState(-1); // End the autonomous routine
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
////    @Override
////    public void loop() {
////
////        // These loop the movements of the robot
////        follower.update();
////        autonomousPathUpdate();
////
////        // Feedback to Driver Hub
////        telemetry.addData("path state", pathState);
////        telemetry.addData("x", follower.getPose().getX());
////        telemetry.addData("y", follower.getPose().getY());
////        telemetry.addData("heading", follower.getPose().getHeading());
////        telemetry.update();
////    }
////
////    @Override
////    public void init() {
////        pathTimer = new Timer();
////        opmodeTimer = new Timer();
////        opmodeTimer.resetTimer();
////
////        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
////        follower.setStartingPose(startPose);
////        buildPaths();
////    }
////
////    /**
////     * This method is called continuously after Init while waiting for "play".
////     **/
////    @Override
////    public void init_loop() {
////    }
////
////    /**
////     * This method is called once at the start of the OpMode.
////     * It runs all the setup actions, including building paths and starting the path system
////     **/
////    @Override
////    public void start() {
////        opmodeTimer.resetTimer();
////        setPathState(0);
////    }
////
////    /**
////     * We do not use this because everything should automatically disable
////     **/
////    @Override
////    public void stop() {
////    }
//}