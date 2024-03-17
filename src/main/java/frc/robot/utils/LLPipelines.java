// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
/** Add your docs here. */
public class LLPipelines {

    public enum pipelines {
        APRILTAGALL(0, pipelinetype.fiducialmarkers),//all tage
        APRILTAGSTART(1, pipelinetype.fiducialmarkers), // tags 3,4 and 7,8
        APRILTAGSAMP(2, pipelinetype.fiducialmarkers), // tags 5 and 6
        APRILTAGSOURCE(3, pipelinetype.fiducialmarkers),//tags 1,2 1nd 9,10
        APRILTAGSTAGE(4, pipelinetype.fiducialmarkers),//tags 11,12,13 and 14,15,16
        APRILTAGALIGN(5, pipelinetype.fiducialmarkers),//4 or 7 only
        APRILTAGSPARE(6, pipelinetype.fiducialmarkers),//
        PYTHON_7(7, pipelinetype.python),
        NOTE_DETECT(8, pipelinetype.detector),
        NOTE_DETECT1(9, pipelinetype.detector);

        public static final pipelines values[] = values();

        private pipelinetype type;

        public String pipelineTypeName;

        private int number;

        private pipelines(int number, pipelinetype type) {
            this.number = number;
            this.type = type;
        }

    }

    public enum pipelinetype {
        color_retroreflective,
        grip,
        python,
        fiducialmarkers,
        classifier,
        detector;
    
        public static final pipelinetype values[] = values();
      }


}
