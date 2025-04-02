package org.firstinspires.ftc.teamcode.Elevator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

public class Elevator extends LinearOpMode{

    //Thread thread = Thread.currentThread();
    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx EH, EA;
    Servo intake_center, intake_AR, intAR;
    CRServo intake_left, intake_right;
    Telemetry telemetry;

    public Elevator(DcMotorEx EA, DcMotorEx EH, Servo intake_center, CRServo intake_left, CRServo intake_right, Servo intake_AR, Servo intAR, Telemetry telemetry){
        this.EH = EH;
        this.EA = EA;
        this.intake_center = intake_center;
        this.intake_left = intake_left;
        this.intake_right = intake_right;
        this.intake_AR = intake_AR;
        this.intAR = intAR;
        this.telemetry = telemetry;
    }
    public void Move_Elevator(double x){

        EH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EH.setDirection(DcMotorSimple.Direction.REVERSE);

        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        EH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setWanted(x);

        while(Math.abs(x) + 320 > Math.abs(EH.getCurrentPosition()) && Math.abs(EH.getCurrentPosition()) < Math.abs(x) - 320){
            EH.setPower(pid.update(EH.getCurrentPosition()));
            telemetry.addLine("in");
            telemetry.addData("pos: ", EH.getCurrentPosition());
            telemetry.addData("x: ", x);
            telemetry.update();
        }

        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        EH.setPower(0);
    }

    public void Change_Angle(double x){

        EA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PID pid = new PID(0.5, 0.2, 0.1, 0, 0);

        EA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setWanted(x);

        while(Math.abs(x) + 320 > Math.abs(EA.getCurrentPosition()) && Math.abs(EA.getCurrentPosition()) < Math.abs(x) - 320){
            EA.setPower(-pid.update(EA.getCurrentPosition()));

        }

        EA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        EA.setPower(0);
    }
    public void Change_Height(double x){
        int count = 0;
        EH.setDirection(DcMotorSimple.Direction.REVERSE);
        EH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        EH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PID pid = new PID(0.15, 0.05, 0.5, 0, 0);

        double thresh = 100;
        pid.setWanted(x);
        double power = 1;
        while (count < 3){
            if (Math.abs(Math.abs(x)-Math.abs(EH.getCurrentPosition())) < thresh){
                count++;
            }
            power = pid.update(EH.getCurrentPosition());


            EH.setPower(power);
            telemetry.addData("EH",EH.getCurrentPosition());
            telemetry.update();

        }
        EH.setPower(0);
//        EH.setPower(0);
        EH.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }



    public void extend(double pow, double sec){
        runtime.reset();
        while(sec > runtime.seconds()){
            intake_right.setPower(pow);
            intake_left.setPower(-pow);
        }
        intake_left.setPower(0);
        intake_right.setPower(0);
    }


    public void move_intake_AG(double pos){
        intAR.setPosition(pos);
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
