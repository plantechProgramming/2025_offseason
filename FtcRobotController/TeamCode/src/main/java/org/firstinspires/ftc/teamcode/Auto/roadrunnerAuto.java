package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
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
@Config
public class roadrunnerAuto extends LinearOpMode{
        public static double inPerTile = 24.3125;


    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: find if same as normal competition tiles



        double pie = Math.PI;
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
         double inPerTileY = inPerTile*1.42;
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
        Actions.runBlocking(

                drive.actionBuilder(beginPose)
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

                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(30, 30), Math.toRadians(90))
                        .build());

    }
    protected void end() {

    }
}