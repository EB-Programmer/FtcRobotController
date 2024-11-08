package org.firstinspires.ftc.teamcode;

public class ClawArmPresetPosition {
    private final double armPosition;
    private final double slideHeight;
    private final String name;

    private ClawArmPresetPosition(double armPosition, double slideHeight, String name) {
        this.armPosition = armPosition;
        this.slideHeight = slideHeight;
        this.name = name;
    }

    public static ClawArmPresetPosition WallGrabPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_WALL_SPECIMEN_POSITION,
                VerticalSlideConstants.VERT_WALL_HEIGHT, "Wall Grab");
    }

    public static ClawArmPresetPosition HighBasketPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_BASKET_DROP_POSITION,
                VerticalSlideConstants.VERT_HIGH_BASKET_HEIGHT, "High Basket");
    }

    public static ClawArmPresetPosition LowBasketPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_BASKET_DROP_POSITION,
                VerticalSlideConstants.VERT_LOW_BASKET_HEIGHT, "Low Basket");
    }

    public static ClawArmPresetPosition HighBasketBackDropPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_BACK_DROP_POSITION,
                VerticalSlideConstants.VERT_HIGH_BASKET_HEIGHT, "Reverse High Basket");
    }

    public static ClawArmPresetPosition LowBasketBackDropPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_BACK_DROP_POSITION,
                VerticalSlideConstants.VERT_LOW_BASKET_HEIGHT, "Reverse Low Basket");
    }

    public static ClawArmPresetPosition HighChamberBackHangPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_BACK_CHAMBER_HANG_POSITION,
                VerticalSlideConstants.VERT_BACK_HIGH_CHAMBER_HEIGHT, "Reverse High Chamber");
    }
    public static ClawArmPresetPosition HighChamberPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_CHAMBER_HANG_POSITION,
                VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT, "High Chamber");
    }

    public static ClawArmPresetPosition LowChamberPreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_CHAMBER_HANG_POSITION,
                VerticalSlideConstants.VERT_LOW_CHAMBER_HEIGHT, "Low Chamber");
    }

    public static ClawArmPresetPosition PreExchangePreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_EXCHANGE_POSITION,
                VerticalSlideConstants.VERT_PRE_EXCHANGE_HEIGHT, "Pre-Exchange");
    }

    public static ClawArmPresetPosition MakeExchangePreset() {
        return new ClawArmPresetPosition(ClawConstants.CLAW_ARM_EXCHANGE_POSITION,
                VerticalSlideConstants.VERT_MAKE_EXCHANGE_HEIGHT, "Make Exchange");
    }

    public double getArmPosition() {
        return this.armPosition;
    }

    public double getSlideHeight() {
        return this.slideHeight;
    }

    public String getName() {
        return this.name;
    }
}
