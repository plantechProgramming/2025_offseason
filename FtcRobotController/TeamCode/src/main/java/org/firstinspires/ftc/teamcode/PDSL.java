package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDSL {
    public double sigmoid(double error) {
        error = error /120;
        return 2/(1+Math.exp(-error)) - 1;
    }

    public double linear(double error) {
        error = error / 150;
        return Math.min(Math.max(error,-1),1);
    }
    private static final ElapsedTime timer = new ElapsedTime();
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double power = 0;

    public double wanted = 0;

    private double integral = 0;

    private double prevError = 0;
    private double prevTime = 0;

    public PDSL(final double kP, final double kI, final double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setWanted2(final double wanted) {
        this.wanted = wanted;
    }

    public double update2(final double current) {
        final double currentError = wanted - current;
        final double currentTime = timer.milliseconds();
        final double deltaTime = currentTime - prevTime;

        integral += currentError * deltaTime;
        final double derivative = deltaTime == 0 ? 0 : (currentError - prevError) / deltaTime;

        prevError = currentError;
        prevTime = currentTime;

        power =  kP * currentError + kI * integral + kD * derivative;

        // 40 deg in ticks = 1492, thresh for change from sig to lin
        if (Math.abs(currentError) < 1492) {
            return linear(power);
        }
        return sigmoid(power);
    }
}

