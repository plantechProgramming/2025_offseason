package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeamCode.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeamCode.teamcode.TankDrive;

import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.OpMode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="roadrunner test", group="Robot")

public class roadrunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double inPerTile = 24.25;
        double pie = Math.PI;
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VelConstraint velConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 0;
            }
            VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
                if (Math.abs(robotPose.position.x.value()) > 5.8*inPerTile || Math.abs(robotPose.position.y.value()) > 5.8*inPerTile) {
                    return 10.0;
                }
                return 60;
            };
        };

        waitForStart();
        if(isStopRequested()){
            return;
        }
        Actions.runBlocking(

                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(inPerTile, inPerTile),Math.toRadians(90))
//                        .strafeTo(new Vector2d(-inPerTile, -inPerTile))
////                        .turn(Math.toRadians(90))
                        .build());

    }
    protected void end() {

    }
}