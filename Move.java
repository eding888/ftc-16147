package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Move extends Components {
    public boolean slowMode = false;

    public void move(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double XPowerLS, double YPowerLS, double XPowerRS, double YPowerRSes ){
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
    }

    public void moveForwardENCODER(double power, int inches){
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = inches * 31;

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);

        moveForward(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveBackwardENCODER(double power, int inches){
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = inches * 31;

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);

        moveForward(-power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveLeftENCODER(double power, int inches){
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = inches * 31;

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);


        moveLeft(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRightENCODER(double power, int inches){
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = inches * 31;

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);


        moveRight(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateLeftENCODER(double power, int inches){
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = inches * 31;

        Components.leftDriveA.setTargetPosition(ticks);
        Components.leftDriveB.setTargetPosition(ticks);
        Components.rightDriveA.setTargetPosition(ticks);
        Components.rightDriveB.setTargetPosition(ticks);


        rotateLeft(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateRightENCODER(double power, int position){
        Components.leftDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Components.leftDriveA.setTargetPosition(position);
        Components.leftDriveB.setTargetPosition(position);
        Components.rightDriveA.setTargetPosition(position);
        Components.rightDriveB.setTargetPosition(position);

       rotateRight(power);

        while(Components.leftDriveA.isBusy() && Components.leftDriveB.isBusy() && Components.rightDriveA.isBusy() && Components.rightDriveB.isBusy()){

        }
        driveEnd();
        Components.leftDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.leftDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Components.rightDriveB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveForward(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    private void moveBackward(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(-power);
    }

    private void moveLeft(double power) {


        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(-power);
    }

    private void moveRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(power);
    }

    private void rotateLeft(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power);
        leftDriveB.setPower(-power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    private void rotateRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(-power);
        rightDriveB.setPower(-power);
    }

    private void diagonalUpRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(power * 1.5);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(power * 1.5);
    }

    private void diagonalUpLeft(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(0);
        leftDriveB.setPower(power * 1.5);
        rightDriveA.setPower(power * 1.5);
        rightDriveB.setPower(0);
    }

    private void diagonalDownRight(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(0);
        leftDriveB.setPower(-power * 1.5);
        rightDriveA.setPower(-power * 1.5);
        rightDriveB.setPower(0);
    }

    private void diagonalDownLeft(double power) {

        if(slowMode)
            power = power / 2.5;

        leftDriveA.setPower(-power * 1.5);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(-power * 1.5);
    }
    private void driveEnd(){
        leftDriveA.setPower(0);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(0);
    }
}
