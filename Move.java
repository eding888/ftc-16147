package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Move extends Components {
    public static boolean slowMode = false;

    public static void move(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double XPowerLS, double YPowerLS, double XPowerRS, double YPowerRSes, boolean slowModes ){
        if ((leftStickX <= -0.3) && (leftStickY >= 0.3))
            diagonalDownLeft((-XPowerLS + YPowerLS) / 2);
        else if ((leftStickX <= -0.3) && (leftStickY <= -0.3))
            diagonalUpLeft((-XPowerLS - YPowerLS) / 2);
        else if ((leftStickX >= 0.3) && (leftStickY >= 0.3))
            diagonalDownRight((XPowerLS + YPowerLS) / 2);
        else if ((leftStickX >= 0.3) && (leftStickY <= -0.3))
            diagonalUpRight((XPowerLS - YPowerLS) / 2);
        else if((leftStickX <= 0.3)&&(leftStickX >= -0.3)&&(leftStickY <= 0.3)&&(leftStickY >= -0.3))
            driveEnd();
        else if ((leftStickX <= 0.3) && (leftStickX >= -0.3))
            moveForward(-YPowerLS);
        else if((leftStickY <= 0.3) &&  (leftStickY >= -0.3))
            moveLeft(-XPowerLS);
        if(rightStickX < -0.05)
            rotateLeft(-XPowerRS);
        else if(rightStickX > -0.05)
            rotateRight(XPowerRS);
        else
            driveEnd();

        slowMode = slowModes;
    }



    private static void moveForward(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    private static void moveBackward(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(-power);
    }

    private static void moveLeft(double power) {


        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(-power);
    }

    private static void moveRight(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(power);
    }

    private static void rotateLeft(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    private static void rotateRight(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(-power);
    }

    private static void diagonalUpRight(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(power * 1.5);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(power * 1.5);
    }

    private static void diagonalUpLeft(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(0);
        leftDriveB.setPower(power * 1.5);
        rightDriveA.setPower(power * 1.5);
        rightDriveB.setPower(0);
    }

    private static void diagonalDownRight(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(0);
        leftDriveB.setPower(-power * 1.5);
        rightDriveA.setPower(-power * 1.5);
        rightDriveB.setPower(0);
    }

    private static void diagonalDownLeft(double power) {

        if(slowMode)
            power = power / 2;

        leftDriveA.setPower(-power * 1.5);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(-power * 1.5);
    }
    private static void driveEnd(){
        leftDriveA.setPower(0);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(0);
    }






}
