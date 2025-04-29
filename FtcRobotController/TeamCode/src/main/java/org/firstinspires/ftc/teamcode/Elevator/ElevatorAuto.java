package org.firstinspires.ftc.teamcode.Elevator;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

public class ElevatorAuto extends LinearOpMode{
    DcMotorEx EH, EA;
    CRServo intake_center_angle;
    Telemetry telemetry;


    public ElevatorAuto(DcMotorEx EA, DcMotorEx EH, CRServo intake_center_angle, Telemetry telemetry){
        this.EH = EH;
        this.EA = EA;
        this.intake_center_angle = intake_center_angle;
        this.telemetry = telemetry;
        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    PID pid_EH = new PID(0.15, 0.05, 0.05, 0, 0);

    public void Change_Height(double wanted){
        //int count = 0;

//        EH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double thresh = 80;
        double power = 1;
        pid_EH.setWanted(wanted);
        while (Math.abs(wanted - EH.getCurrentPosition()) > thresh){
            power = pid_EH.update(EH.getCurrentPosition());
            EH.setPower(power);
        }
        EH.setPower(0);
        return;
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
