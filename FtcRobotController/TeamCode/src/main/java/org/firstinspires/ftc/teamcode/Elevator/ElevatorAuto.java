package org.firstinspires.ftc.teamcode.Elevator;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ElevatorAuto{
    private DcMotorEx EA,EH;
    public void Elevator(HardwareMap hardwareMap){
        EA = hardwareMap.get(DcMotorEx.class,"EA");
        EA.setDirection(DcMotorEx.Direction.REVERSE);
        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        EA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        EH = hardwareMap.get(DcMotorEx.class, "EH");
        EH.setDirection(DcMotorEx.Direction.FORWARD);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EH.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        EH.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public Action up(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }
}
