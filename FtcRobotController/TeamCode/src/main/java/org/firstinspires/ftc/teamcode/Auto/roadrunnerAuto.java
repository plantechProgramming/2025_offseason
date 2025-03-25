package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeamCode.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeamCode.teamcode.TankDrive;

import androidx.activity.result.contract.ActivityResultContracts;

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
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.setTangent(0)
                        .splineTo(new Vector2d(48, 48), Math.PI / 2)
                        .build());

    }
    protected void end() {

    }
}