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

    public static void moveForwardENCODER(double power, int inches){

        int ticks = inches * 31;


        leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveA.setTargetPosition(ticks);
        leftDriveB.setTargetPosition(ticks);
        rightDriveA.setTargetPosition(ticks);
        rightDriveB.setTargetPosition(ticks);

        leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        moveForward(-power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveBackwardENCODER(double power, int inches){
        int ticks = inches * 31;

        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);


        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        moveForward(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveLeftENCODER(double power, int inches){
        int ticks = inches * 31;

        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        moveRight(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveRightENCODER(double power, int inches){
        int ticks = inches * 31;

        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);






        moveLeft(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void rotateLeftENCODER(double power, int inches){

        int ticks = inches * 31;
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);






        rotateRight(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public static void rotateRightENCODER(double power, int inches){
        int ticks = inches * 31;
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);



       rotateLeft(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static void moveForward(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    private static void moveBackward(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(-power);
    }

    private static void moveLeft(double power) {


        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(-power);
    }

    private static void moveRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(power);
    }

    private static void rotateLeft(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    private static void rotateRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(-power);
    }

    private static void diagonalUpRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power * 1.5);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(power * 1.5);
    }

    private static void diagonalUpLeft(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(0);
        leftDriveB.setPower(power * 1.5);
        rightDriveA.setPower(power * 1.5);
        rightDriveB.setPower(0);
    }

    private static void diagonalDownRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(0);
        leftDriveB.setPower(-power * 1.5);
        rightDriveA.setPower(-power * 1.5);
        rightDriveB.setPower(0);
    }

    private static void diagonalDownLeft(double power) {

        if(slowMode)
            power = power / 2.5;

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
