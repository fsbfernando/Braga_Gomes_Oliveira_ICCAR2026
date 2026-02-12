**PosesCalibration.py:** Positions for UR16e move to take the pictures for intrinsic and extrinsic camera calibration.

**Tabela_45_Valores_ISO.csv:** Repeatibility and accuracy per block of the experiment described on paper.

**Calibration.m/.asv:** Algorithm to execute the camera calibration with previous obtained data.

**EstimateCameraRobotTransform/Park_martin/XC2.m:** Algorithm of 'EstimateCameraRobotTransform' in matlab, adapted to use different methods of hand-eye calibration.

**Hand_eye_complete:** Algorithm that provides as output the hand-eye transformation (with previous data as input).

**jnt_configrad.mat:** Joint positions of ur16e in each saved point in the world.

**Analise_Rep_Acc_MAD_Images.r:** R Algorithm used no analyze the experiment data (2 ANOVA's application with MAD to remove outliers)
