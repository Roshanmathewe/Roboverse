# RoboVerse: AI-Powered Digital Twin Platform for Autonomous Robot Simulation
## Foundation of Computer Vision — Project Report

**Department:** Artificial Intelligence and Machine Learning  
**Institution:** Karunya Institute of Technology and Sciences  
**Academic Year:** 2026–2027  
**Project Team:** Roshan Mathew E (URK24CS7046), Suraj Nair (URK24CS7082)

> **Submission note:** Replace the screenshot placeholders with genuine screenshots from your running project. The report distinguishes implemented integration from performance that still requires quantitative evaluation. Do not report unmeasured accuracy, precision, recall, mAP, or sustained FPS as verified results.

## Abstract

RoboVerse is a software-based digital-twin and computer-vision demonstration built around a simulated warehouse robot. The project integrates a warehouse environment, a robot model, a camera stream, a YOLO-based package detector, ROS 2 communication, RViz visualization, and a dashboard. The simulated camera provides images to the perception module, which runs object detection and publishes an annotated image for visualization. This setup demonstrates how camera-based perception can be integrated into a robotics simulation without requiring physical hardware.

The project emphasizes system integration and visualization rather than claiming a newly trained detection model. A sample 640 × 480 camera snapshot was inspected during development; the highest observed package-class confidence in that snapshot was approximately 0.0618. This is a confidence score for one observed frame, not a measure of model accuracy. Reliable performance claims require a labeled evaluation set and repeated measurements. Future work includes improving the detector, gathering representative images, tuning inference settings, and reporting standard detection metrics.

**Keywords:** Computer vision, digital twin, ROS 2, Gazebo, YOLO, object detection, warehouse simulation, RViz.

# Chapter 1: Introduction

## 1.1 Overview

Computer vision enables machines to extract useful information from images and video. In warehouse applications, vision systems can help identify packages, locate objects, and provide visual feedback to autonomous systems. Testing such systems on physical robots can be expensive and may involve safety and setup constraints. Simulation offers a repeatable environment in which perception components and robot software can be integrated and observed.

RoboVerse provides a simulated warehouse setting with a robot-mounted camera and a computer-vision pipeline. The camera stream is processed by a YOLO-based detector, and the resulting annotated images can be visualized alongside the simulation and dashboard.

## 1.2 Background

A computer-vision system commonly includes image acquisition, preprocessing, inference, postprocessing, and visualization or downstream use. Object detectors such as the YOLO family predict object classes and bounding boxes in images. ROS 2 provides a communication framework through topics and nodes, while Gazebo provides physics-based simulation and virtual sensors. RViz supports visualization of ROS data.

Combining these components makes it possible to study the interface between a simulated sensor and a vision model. The quality of the final detection still depends on the model, training data, camera view, lighting, object appearance, and inference settings.

## 1.3 Problem Statement

A warehouse robot needs visual information about objects in its environment. Developing and testing a complete vision pipeline on physical equipment can require additional cost and time. A software simulation can provide a controlled environment for integrating camera input, object detection, and visualization. The project addresses the implementation of such an integrated demonstration and the need to evaluate its detection performance honestly.

## 1.4 Objectives

- Create a simulated warehouse environment and robot setup.
- Obtain a camera image stream from the simulated robot.
- Integrate a YOLO-based package-detection component.
- Publish and visualize annotated camera images through ROS 2.
- Display the simulation and system status through visualization tools and a dashboard.
- Identify limitations and define measurable evaluation steps for future improvement.

## 1.5 Scope

The scope is a software-only proof of concept. It covers simulation, camera image flow, detector integration, ROS 2 communication, and visualization. It does not establish production-ready warehouse autonomy, reliable package recognition in all conditions, or verified real-world performance. Those claims require additional tests and, for real deployment, validation on physical hardware.

# Chapter 2: Literature Review

## 2.1 Overview of Existing Work

Computer-vision research has developed from hand-engineered features and traditional classifiers to deep convolutional neural networks and modern object detectors. Two-stage detectors often prioritize proposal-based localization, while one-stage detectors such as YOLO perform detection in a single integrated inference pipeline and are widely used where speed matters. Performance depends on dataset coverage, model size, image resolution, hardware, and confidence and non-maximum-suppression settings.

Simulation and robotics middleware are also important parts of perception research. A virtual camera can provide repeatable images, while ROS 2 topics allow separate sensor, perception, and visualization nodes to communicate. Gazebo supports simulation of environments and sensors, and RViz helps inspect published data.

## 2.2 Selected Research and Technical References

The following references provide background for the methods used in this project. Confirm the final bibliographic details and the exact version or edition accessed before academic submission.

1. Redmon, J., Divvala, S., Girshick, R., and Farhadi, A. “You Only Look Once: Unified, Real-Time Object Detection.” *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, 2016.
2. Redmon, J., and Farhadi, A. “YOLO9000: Better, Faster, Stronger.” *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, 2017.
3. Redmon, J., and Farhadi, A. “YOLOv3: An Incremental Improvement.” arXiv:1804.02767, 2018.
4. Bochkovskiy, A., Wang, C.-Y., and Liao, H.-Y. M. “YOLOv4: Optimal Speed and Accuracy of Object Detection.” arXiv:2004.10934, 2020.
5. Jocher, G., Chaurasia, A., and Qiu, J. Ultralytics YOLO documentation and software repository. Consult the official documentation for the exact version used.
6. Macenski, S., Foote, T., Gerkey, B., Lalancette, C., and Woodall, W. “Robot Operating System 2: Design, Architecture, and Uses in the Wild.” *Science Robotics*, 7(66), 2022.
7. Open Robotics. *Gazebo Sim Documentation*. Official documentation for the installed Gazebo version.
8. Open Robotics. *ROS 2 Documentation*. Official documentation for the installed ROS 2 distribution.
9. Open Robotics. *RViz User Guide*. Official documentation for ROS visualization tools.
10. Open Robotics. *ros_gz Documentation*. ROS–Gazebo integration documentation relevant to the installed versions.

## 2.3 Comparison of Approaches

| Approach | Strengths | Limitations |
|---|---|---|
| Traditional image processing | Simple and interpretable for constrained scenes | Sensitive to changes in lighting, shape, and background |
| Two-stage deep detectors | Can offer strong localization performance | May have higher inference cost |
| One-stage detectors such as YOLO | Convenient integrated detection pipeline; useful for real-time experiments | Accuracy depends on model and training data; speed varies by hardware and settings |
| Simulated camera and ROS 2 pipeline | Repeatable tests and modular communication | Simulation images may not represent real-world variation |
| Manual visual inspection | Useful for debugging individual frames | Not a substitute for quantitative evaluation |

## 2.4 Research Gap and Project Motivation

A useful demonstration needs more than a detector running in isolation: it must connect camera acquisition, inference, message publication, and visualization into a coherent pipeline. RoboVerse focuses on this integration in a simulated warehouse. The current implementation is a proof of concept; it does not claim a novel detection algorithm or superior benchmark performance. A further research contribution could compare baseline and improved detectors on a labeled dataset using a fixed evaluation protocol.

# Chapter 3: System Analysis and Design

## 3.1 Existing System

In a basic workflow, camera images may be inspected manually or an object detector may be run separately on saved images. This can make it difficult to observe how sensor output, perception, and robot visualization work together during a simulation.

## 3.2 Proposed System

RoboVerse connects a simulated warehouse camera to a YOLO-based detector through ROS 2. The annotated output is published for visualization, while the simulator and dashboard provide context about the running system. The components are modular so that the detector or visualization configuration can be updated independently.

## 3.3 Functional Requirements

- Launch the simulated warehouse environment and robot.
- Produce camera images from the simulated sensor.
- Subscribe to the camera image topic from the perception component.
- Run object detection and generate an annotated image.
- Publish the processed image to a ROS 2 topic.
- Display camera output and system status using available visualization tools.

## 3.4 Non-functional Requirements

- Components should be modular and maintainable.
- The pipeline should provide useful visual feedback during debugging.
- Performance measurements should be repeatable and documented.
- Software versions and launch steps should be recorded.
- The system should fail safely when a topic or model is unavailable.

## 3.5 Architecture

```text
+----------------------------+
| Gazebo Warehouse Simulation|
| Robot + Virtual Camera     |
+-------------+--------------+
              |
              v
       /camera/image
              |
              v
+----------------------------+
| ROS 2 Perception Node       |
| Image handling + YOLO model |
+-------------+--------------+
              |
              v
 /camera/detections/image
              |
              v
+----------------------------+
| RViz / Dashboard / Review   |
| Annotated image + status    |
+----------------------------+
```

## 3.6 Workflow

```text
Start simulation
      |
      v
Initialize robot and camera
      |
      v
Receive camera image
      |
      v
Run object detector
      |
      v
Draw predicted boxes and labels
      |
      v
Publish annotated image
      |
      v
Visualize output and record test results
```

## 3.7 Use Cases

- **Operator/developer:** Starts the simulation, checks camera output, observes detections, and records issues.
- **Simulation system:** Generates the warehouse scene and virtual camera images.
- **Perception node:** Processes images and publishes annotated results.
- **Visualization tools:** Display the simulation, camera data, and system state.

# Chapter 4: Methodology

## 4.1 Development Approach

The project was developed as an integration workflow: configure the simulation, verify camera output, connect the perception node, inspect the annotated image topic, and validate visualization. Debugging was performed by checking topic names and camera output before interpreting detector results.

## 4.2 Image Acquisition

The virtual camera generates image frames from the simulated environment. The ROS 2 image topic used by the perception pipeline is `/camera/image`. Image dimensions and stream availability should be checked before inference testing.

## 4.3 Object Detection

The perception component uses a YOLO-family model for package detection. At a high level, the model processes an input image and predicts candidate bounding boxes, class labels, and confidence scores. Postprocessing filters predictions according to configured thresholds and produces an annotated image. The model file and exact runtime settings should be recorded in the final experiment log.

A detector confidence score represents the model's confidence in an individual prediction; it is not the same as accuracy. Accuracy-related claims require comparison with ground-truth labels across a defined evaluation set.

## 4.4 ROS 2 Communication

ROS 2 nodes exchange data through topics. The perception node consumes `/camera/image` and publishes annotated output to `/camera/detections/image`. Correct topic names, message types, and compatible image encodings are required for the pipeline to work.

## 4.5 Tools and Environment

Record the actual versions from the machine used for the final demonstration:

- Operating system: Windows 11 with Ubuntu 24.04 under WSL 2
- Robotics middleware: ROS 2 Jazzy
- Simulator: Gazebo Harmonic
- Visualization: RViz
- Detector: YOLO-family model; record the exact package and version
- Programming language: Python for ROS 2 and perception integration
- Hardware: MSI Thin A15 laptop; Ryzen 7 7735HS CPU, 16 GB RAM, NVIDIA RTX 4060 Laptop GPU with 8 GB VRAM

Versions can change across installations; verify them from the working environment before final submission.

## 4.6 Evaluation Plan

For a defensible evaluation, use a labeled set of representative images and record:
- Number of images and number of ground-truth objects
- Class definitions and labeling procedure
- Precision, recall, and F1-score at a stated confidence threshold
- mAP@0.5 and, if supported by the evaluation tool, mAP@0.5:0.95
- Average inference latency and sustained FPS measured over a defined duration
- Hardware, image size, model version, confidence threshold, and warm-up procedure

Do not calculate these metrics from a single frame or report assumed values.

# Chapter 5: Implementation

## 5.1 Main Modules

1. **Simulation module:** Loads the warehouse environment and robot model in Gazebo.
2. **Camera module:** Provides image frames from the virtual camera.
3. **Perception module:** Subscribes to camera images and runs YOLO-based package detection.
4. **ROS 2 communication module:** Transmits images and related data through topics.
5. **Visualization module:** Displays the simulator, camera output, and annotated detections through RViz and the project dashboard.

## 5.2 Image Pipeline

The processing sequence is:

1. Receive an image message from `/camera/image`.
2. Convert the message to the image representation expected by the detector.
3. Run the configured model.
4. Apply the model's configured postprocessing and confidence threshold.
5. Draw the predicted boxes and labels.
6. Publish the annotated image on `/camera/detections/image`.
7. View the output and record observed behavior.

The exact code and launch-file paths should be taken from the final repository version rather than copied from a generic example.

## 5.3 Implementation Notes

During integration, topic consistency was important: the camera publisher and perception subscriber must use the same topic and compatible message types. The visualization configuration must also point to the actual output topic. If the image is visible but detections are weak, the problem may be model suitability or scene appearance rather than topic connectivity.

## 5.4 Screenshots to Insert

Replace each placeholder with an actual screenshot from the project:

- **Figure 5.1:** Gazebo warehouse environment and robot.
- **Figure 5.2:** Raw camera image from the simulated robot.
- **Figure 5.3:** Annotated image published by the detector.
- **Figure 5.4:** RViz view showing the relevant camera display.
- **Figure 5.5:** RoboVerse dashboard with simulation status.

Do not create or label illustrative images as real test evidence.

# Chapter 6: Results and Discussion

## 6.1 Integration Results

The project integrates a simulated warehouse, robot camera stream, ROS 2 perception pipeline, YOLO-based detector, annotated image output, and visualization tools. The expected output topic is `/camera/detections/image`, based on the implemented configuration. Include screenshots from the final run to demonstrate each component.

## 6.2 Observed Detector Output

During development, one inspected camera snapshot was 640 × 480 pixels. In that snapshot, the highest observed package-class confidence was approximately **0.0618**. This indicates a weak prediction in that particular frame and should not be presented as detection accuracy or as a successful package-recognition rate. It suggests that the model, scene, thresholds, or training data require further investigation.

## 6.3 Testing Table

| Test | Expected result | Evidence/status |
|---|---|---|
| Launch simulation | Warehouse and robot start | Insert screenshot from final run |
| Camera stream | Camera images are available | Verify with a genuine camera screenshot |
| Perception subscription | Detector receives images | Confirm from logs/topic inspection |
| Annotated output | Processed image is published | Insert genuine output screenshot |
| Visualization | Image is visible in RViz/dashboard | Insert genuine screenshot |
| Recognition quality | Correct package detections on labeled data | Quantitative evaluation still required |
| Throughput/latency | Sustained FPS and latency measured over a fixed interval | Do not claim a value until measured |

## 6.4 Discussion

The implementation demonstrates how a vision model can be integrated into a simulated robotic system. Integration success and detector quality are separate concerns: a functioning image pipeline does not guarantee accurate predictions. The low confidence observed in one frame is a limitation and should be reported transparently. A stronger evaluation requires labeled ground truth, multiple scenes, and repeatable measurements.

## 6.5 Limitations

- The detector's package-recognition performance has not been established by a complete labeled benchmark.
- A single-frame confidence score cannot establish precision, recall, or mAP.
- Simulated lighting, textures, object scale, and camera angles may differ from real warehouse conditions.
- Sustained throughput and end-to-end latency must be measured under a defined test procedure.
- The system is a simulation proof of concept and is not validated for production deployment.

# Chapter 7: Conclusion and Future Scope

## 7.1 Conclusion

RoboVerse demonstrates an integrated computer-vision workflow in a simulated warehouse environment. The system brings together a simulated robot camera, a YOLO-based package detector, ROS 2 image communication, and visualization through RViz and a dashboard. The project provides a useful foundation for testing perception modules in a repeatable environment.

The current results support a claim of software integration, not a claim of high detection accuracy. The observed low package confidence in one inspected frame highlights the need for additional model evaluation and improvement. Genuine screenshots and measured test results should be included in the final submission.

## 7.2 Future Scope

- Build or collect a labeled dataset of simulated packages under varied lighting, distance, orientation, and occlusion.
- Evaluate the baseline model using precision, recall, F1-score, and mAP.
- Fine-tune or replace the model only after establishing a baseline and preserving existing artifacts.
- Measure inference latency and sustained FPS using a repeatable protocol.
- Add tracking across video frames and evaluate ID consistency.
- Test robustness to occlusion, clutter, and changes in camera viewpoint.
- Validate any proposed improvements against the same held-out test set.

# References

[1] J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, “You Only Look Once: Unified, Real-Time Object Detection,” *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, 2016.

[2] J. Redmon and A. Farhadi, “YOLO9000: Better, Faster, Stronger,” *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, 2017.

[3] J. Redmon and A. Farhadi, “YOLOv3: An Incremental Improvement,” arXiv:1804.02767, 2018.

[4] A. Bochkovskiy, C.-Y. Wang, and H.-Y. M. Liao, “YOLOv4: Optimal Speed and Accuracy of Object Detection,” arXiv:2004.10934, 2020.

[5] Ultralytics, *Ultralytics YOLO Documentation and Software Repository*. Use the official documentation for the installed model/package version.

[6] S. Macenski, T. Foote, T. Gerkey, C. Lalancette, and W. Woodall, “Robot Operating System 2: Design, Architecture, and Uses in the Wild,” *Science Robotics*, vol. 7, no. 66, 2022.

[7] Open Robotics, *Gazebo Sim Documentation*. Use the documentation corresponding to the installed Gazebo release.

[8] Open Robotics, *ROS 2 Documentation*. Use the documentation corresponding to ROS 2 Jazzy.

[9] Open Robotics, *RViz User Guide*. Official ROS visualization documentation.

[10] Open Robotics, *ros_gz Documentation*. ROS–Gazebo integration documentation.

# Appendix A: Final Evidence Checklist

- [ ] Screenshot of the running Gazebo warehouse and robot
- [ ] Screenshot of the raw camera feed
- [ ] Screenshot of the annotated detector output
- [ ] Screenshot of RViz camera visualization
- [ ] Screenshot of the dashboard
- [ ] Terminal or log evidence that the expected nodes and topics are running
- [ ] Record of software versions and model file
- [ ] Evaluation dataset description and labeling method
- [ ] Measured metrics, with method and sample size
- [ ] Confirm all references and bibliographic details before submission

## Appendix B: Experiment Record Template

| Item | Value to record |
|---|---|
| Date/time | |
| Model name and version | |
| Dataset and number of images | |
| Image resolution | |
| Confidence threshold | |
| Precision | |
| Recall | |
| F1-score | |
| mAP@0.5 | |
| Average inference latency | |
| Sustained FPS and test duration | |
| Hardware/software versions | |
| Notes and limitations | |
