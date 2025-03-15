package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Side;
import org.openftc.easyopencv.OpenCvCamera;

public class DriveTrain {
    private DcMotorEx BR, BL, FR, FL;
    private BNO055IMU imu;
    private IMU Imu;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private OpenCvCamera camera;
    ElapsedTime runtime = new ElapsedTime();
    public static double Kp = 0.5, Ki = 0.2, Kd = 0.01;
    static final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double COUNTS_PER_CM = 537.6 / WHEEL_DIAMETER_CM * Math.PI;//(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * PI);

    public DriveTrain(DcMotorEx BR, DcMotorEx BL, DcMotorEx FR, DcMotorEx FL, Telemetry telemetry, IMU imu) {
        this.BL = BL;
        this.BR = BR;
        this.FL = FL;
        this.FR = FR;

        this.Imu = imu;
        this.telemetry = telemetry;
    }


    public void side_drive(double power){
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FL.setPower(1 * power);
        BL.setPower(-1 * power);

        FR.setPower(-1 * power);
        BR.setPower(1 * power);
    }

    public void drive(double y, double x, double rx, double botHeading){

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY + rotX + rx) / denominator;

        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);

        FR.setPower(frontRightPower);
        BR.setPower(backRightPower);

    }



    public void turnToGyro_minus(double degrees) {
        double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        PID pid = new PID(2, 3, 0, 0, 0);

        pid.setWanted(degrees);

        if(degrees < 0){
            while (botAngle >= degrees) {
                botAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                FL.setPower(-Math.abs(pid.update(botAngle)));
                FR.setPower(Math.abs(pid.update(botAngle)));

                BR.setPower(Math.abs(pid.update(botAngle)));
                BL.setPower(-Math.abs(pid.update(botAngle)));

                telemetry.addData("IMU", botAngle);
                telemetry.update();

            }stop();
        }
    }

    public void turnToGyro_plus(double degrees) {
        double botAngle = Math.abs(Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        PID pid = new PID(5, 0, 0, 0, 0);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        degrees = degrees * (1500.0 / 40.0);

        double pos = BR.getCurrentPosition();
        pid.setWanted(degrees);

        while (Math.abs(pos) <= Math.abs(degrees)) {
                pos = BR.getCurrentPosition();

                FL.setPower(Math.abs(pid.update(pos)));
                BL.setPower(Math.abs(pid.update(pos)));

                FR.setPower(-Math.abs(pid.update(pos)));
                FR.setPower(-Math.abs(pid.update(pos)));

                telemetry.addData("IMU", botAngle);
                telemetry.update();

            }stop();
    }


        public void stop(){
            FL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
}
