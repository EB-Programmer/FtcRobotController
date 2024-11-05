package org.firstinspires.ftc.teamcode;

public class ClawArmPresetPosition {
    private double armPosition;
    private double slideHeight;

    private ClawArmPresetPosition(double armPosition, double slideHeight) {
        this.armPosition = armPosition;
        this.slideHeight = slideHeight;
    }

    public static ClawArmPresetPosition WallGrabPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_WALL_SPECIMEN_POSITION,
                VerticalSlideConstants.VERT_WALL_HEIGHT);
    }

    public static ClawArmPresetPosition HighBasketPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_BASKET_POUNCE_POSITION,
                VerticalSlideConstants.VERT_HIGH_BASKET_HEIGHT);
    }

    public static ClawArmPresetPosition LowBasketPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_BASKET_POUNCE_POSITION,
                VerticalSlideConstants.VERT_LOW_BASKET_HEIGHT);
    }

    public static ClawArmPresetPosition HighChamberPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_CHAMBER_HANG_POSITION,
                VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT);
    }

    public static ClawArmPresetPosition LowChamberPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_CHAMBER_HANG_POSITION,
                VerticalSlideConstants.VERT_LOW_CHAMBER_HEIGHT);
    }

    public static ClawArmPresetPosition PreExchangePreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_EXCHANGE_POSITION,
                VerticalSlideConstants.VERT_PRE_EXCHANGE_HEIGHT);
    }

    public static ClawArmPresetPosition MakeExchangePreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_EXCHANGE_POSITION,
                VerticalSlideConstants.VERT_MAKE_EXCHANGE_HEIGHT);
    }

    public double getArmPosition() {
        return armPosition;
    }

    public double getSlideHeight() {
        return slideHeight;
    }
}
