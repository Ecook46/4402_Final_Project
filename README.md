# 4402_Final_Project
This repository holds the final code submission for the CEE 4402 final project. Below is a description of the submitted files:

## Follow_Car
- **control_drive.py**: Permits simple line tracking while driving.
- **control.py**: Provides main control functionality in mainRun.py consisting of all aspects of sub-process 2.
- **driveOnly.py**: Similar functionality to driveOnly.py but runs independent of other processes for testing.
- **mainRun.py**: Main control script used in final testing, creates and manages the two subprocesses. 
- **processImage.py**: Provides functionality to sub-process 1 by running the YOLO model on new frames received from other processes.
- **targetTrack.pt**: Trained YOLO model file.
- **trackTarget.py**: Combined camera/servo functionality with model to test static target tracking.

## Training
- **trainTargetModel.ipynb**: Simple training pipeline for a YOLOv11n model run on Google Colab.


## Visualization
- **loggedData.csv**: Raw CSV data logged from car during a trial run.
- **visualize.ipynb**: Processes the CSV to create simple visualizations.