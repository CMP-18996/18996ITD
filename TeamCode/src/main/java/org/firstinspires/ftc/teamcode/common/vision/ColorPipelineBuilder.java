package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.teamcode.common.robot.Color;

public class ColorPipelineBuilder {
    private final ColorPipeline built;
    public ColorPipeline build() {
        return built;
    }

    public static ColorPipelineBuilder createBuilder(Color color) {
        return new ColorPipelineBuilder(new ColorPipeline(color));
    }

    public ColorPipelineBuilder setMinSize(double size) {
        built.setMinSize(size);
        return this;
    }

    public ColorPipelineBuilder setMaxSize(double size) {
        built.setMaxSize(size);
        return this;
    }

    private ColorPipelineBuilder(ColorPipeline colorPipeline) {
        built = colorPipeline;
    };
}
