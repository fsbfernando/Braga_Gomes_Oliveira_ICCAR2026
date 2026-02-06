clc, clear, close all

%% Usa o Python externo instalado no Sistema
pythonExe = "/usr/bin/python3";
script = "PosesCalibration.py";

cmd = pythonExe + " " + script;
status = system(cmd)

%% Calibraçao Intrinsics

% Carrega as imagens de calibração de uma pasta específica
imageFolder = './Intrinsics_v2/*.png';
images = imageDatastore(imageFolder);

% Detecta os pontos das bordas do tabuleiro de xadrez nas imagens
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Gera as coordenadas do mundo, baseadas no tamanho dos quadrados do tabuleiro
squareSize = 15; % mm
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Define as combinações de parâmetros para testar
numRadialDistortionCoefficients = [2, 3];
estimateSkewOptions = [true, false];
estimateTangentialDistortionOptions = [true, false];

% Inicializa uma estrutura para armazenar os resultados
results = struct();
counter = 0;

% Testa todas as combinações de parâmetros definidos acima
for radialCoeffs = numRadialDistortionCoefficients
    for skewOption = estimateSkewOptions
        for tangOption = estimateTangentialDistortionOptions
            counter = counter + 1;
            
            % Exibe a combinação de parâmetros que está sendo testada
            fprintf('\nTesting combination %d:\n', counter);
            fprintf('RadialCoeffs: %d, Skew: %d, Tangential: %d\n', ...
                    radialCoeffs, skewOption, tangOption);
            
            % Calibração inicial usando todas as imagens
            goodImages = true(1, size(imagePoints, 3));
            params = estimateCameraParameters(imagePoints(:,:,goodImages), worldPoints, ...
                'ImageSize', [size(readimage(images,1), 1), size(readimage(images,1), 2)], ...
                'NumRadialDistortionCoefficients', radialCoeffs, ...
                'EstimateSkew', skewOption, ...
                'EstimateTangentialDistortion', tangOption);
            
            % Erro inicial de reprojeção (quanto a calibração está "desajustada")
            initialError = params.MeanReprojectionError;
            
            % Iterações de remoção de outliers (remover as 7 imagens com piores erros de reprojeção)
            numIterations = 3;
            for iter = 1:numIterations
                reprojErrors = params.ReprojectionErrors;
                meanErrors = calculateMeanReprojectionErrors(reprojErrors);
                
                % Encontra a imagem com o maior erro médio e a marca como "ruim"
                [~, worstIdx] = max(meanErrors);
                validImageIdx = find(goodImages);
                worstIdxValid = validImageIdx(worstIdx);
                goodImages(worstIdxValid) = false;
                
                % Recalibra os parâmetros sem a imagem "ruim"
                params = estimateCameraParameters(imagePoints(:,:,goodImages), worldPoints, ...
                    'ImageSize', params.ImageSize, ...
                    'NumRadialDistortionCoefficients', radialCoeffs, ...
                    'EstimateSkew', skewOption, ...
                    'EstimateTangentialDistortion', tangOption);
            end
            
            % Armazena os resultados da calibração para essa combinação de parâmetros
            results(counter).RadialCoeffs = radialCoeffs;
            results(counter).EstimateSkew = skewOption;
            results(counter).EstimateTangentialDistortion = tangOption;
            results(counter).InitialError = initialError;
            results(counter).FinalError = params.MeanReprojectionError;
            results(counter).Params = params;
            results(counter).ImagesUsed = find(goodImages);
            
            % Exibe os erros iniciais e finais após a remoção de outliers
            fprintf('Initial error: %.4f pixels\n', initialError);
            fprintf('Final error after outlier removal: %.4f pixels\n', params.MeanReprojectionError);
            fprintf('Images kept: %d/%d\n', sum(goodImages), numel(images.Files));
        end
    end
end

% Encontra a melhor combinação de parâmetros com o menor erro final
[bestError, bestIdx] = min([results.FinalError]);
disp([results.FinalError])
bestParams = results(bestIdx).Params;

% Exibe os resultados da melhor combinação
fprintf('\n=== BEST COMBINATION ===\n');
fprintf('RadialCoeffs: %d, Skew: %d, Tangential: %d\n', ...
        results(bestIdx).RadialCoeffs, ...
        results(bestIdx).EstimateSkew, ...
        results(bestIdx).EstimateTangentialDistortion);
fprintf('Initial error: %.4f pixels\n', results(bestIdx).InitialError);
fprintf('Final error: %.4f pixels\n', bestError);
fprintf('Images used: %s\n', mat2str(results(bestIdx).ImagesUsed));

% Salva os melhores parâmetros encontrados e todos os resultados das calibrações
save('./data/bestCameraParams_v2.mat', 'bestParams');

% Exibe os erros de reprojeção para a melhor combinação de parâmetros
figure;
showReprojectionErrors(bestParams);
title(sprintf('Best Combination (Error: %.4f pixels)', bestError));

% Função auxiliar para calcular os erros de reprojeção médios, ignorando NaN e zeros
function meanErrors = calculateMeanReprojectionErrors(reprojErrors)
    % Função para calcular a média dos erros de reprojeção,
    % ignorando valores NaN e zeros, para cada imagem.

    % Inicializando o vetor para armazenar os erros médios
    meanErrors = zeros(1, size(reprojErrors, 3));  
    
    % Loop sobre cada imagem
    for imgIdx = 1:size(reprojErrors, 3)
        % Extrair os erros de reprojeção para a imagem imgIdx
        imageErrors = reprojErrors(:, :, imgIdx);
        
        % Identificar onde há NaN ou zero (não válidos)
        validMask = ~(isnan(imageErrors(:, 1)) | isnan(imageErrors(:, 2)) | imageErrors(:, 1) == 0 | imageErrors(:, 2) == 0);
        
        % Calcular a soma dos erros válidos
        validErrors = sqrt(sum(imageErrors(validMask, :).^2, 2)); 
        
        % Calcular a média dividindo pela quantidade de valores válidos
        meanErrors(imgIdx) = sum(validErrors) / sum(validMask);  
    end
end



%% Extrinsics

ld = load("data/bestCameraParams_v2.mat").bestParams;
intrinsics = ld.Intrinsics;

numPoses = 30;
imds = imageDatastore("./Extrinsics_v2/");
montage(imds)

squareSize = 0.015; % Measured in meters
camExtrinsics(numPoses,1) = rigidtform3d;

% Estimate transform from the board to the camera for each pose.
for i = 1:numPoses

    % Undistort the image using the estimated camera intrinsics.
    calibrationImage = readimage(imds,i);
    undistortedImage = undistortImage(calibrationImage,intrinsics);

    % Estimate the extrinsics while disabling the partial checkerboard
    % detections to ensure consistent checkerboard origin across poses. 
    [imagePoints, patternDims] = detectCheckerboardPoints(undistortedImage,PartialDetections=false);
    worldPoints = patternWorldPoints("checkerboard",patternDims,squareSize);
    camExtrinsics(i) = estimateExtrinsics(imagePoints,worldPoints,intrinsics);
end

jointPositionsDeg = load("jnt_config_rad.mat").jnt_config

robotModel = loadrobot("universalUR16e");
robotModel.DataFormat = "column";

endEffectorToBaseTform(numPoses,1) = rigidtform3d; 
flangeToBaseTform(numPoses,1) = rigidtform3d;
for i = 1:numPoses   
    jointPositionsRad = jointPositionsDeg(i,:)'; % Convert the pose angles from degrees to radians.
    endEffectorToBaseTform(i) = getTransform(robotModel,jointPositionsRad,"tool0");
    flangeToBaseTform(i) = getTransform(robotModel,jointPositionsRad,"flange");
end

config = "moving-camera";
% cameraToEndEffectorTform = estimateCameraRobotTransform(camExtrinsics,endEffectorToBaseTform,config)
% cameraToFalngeTform = estimateCameraRobotTransform(camExtrinsics, flangeToBaseTform, config)

% park
% cameraToEndEffectorTform_park = estimateCameraRobotTransform_park(camExtrinsics,endEffectorToBaseTform,config)

%XC2
cameraToEndEffectorTform_xc2 = estimateCameraRobotTransform_xc2(camExtrinsics,endEffectorToBaseTform,config)

%% teste
% 
testImage = imread("apriltag1.png");
figure;
imshow(testImage)
figure;
undistortedTestImage = undistortImage(testImage,ld);
imshow(undistortedTestImage)
figure;
% 
% testPose = [73.1 -65.60 59.09 -83.41 -89.67 -16.86]'
% % testPose = [74.64 -72.22 75.51 -93.21 -89.66 -15.29]'
% % testPose = [77.56 -70.37 40.89 -60.44 -89.71 -12.49]'
% % testPose = [67.50 -87.91 41.83 -42.99 -89.54 -22.57]'
testPose = [90.61 -54.65 61.84 -97.14 -89.77 -2.92]'
% 
testPoseRad = deg2rad(testPose);
endEffectorToBaseTformTest = getTransform(robotModel,testPoseRad,"tool0");
% 
% Specify the tag family and tag size of the AprilTag.
tagFamily = 'tag36h11';
tagSize = .049; % AprilTag size in meters
% 
% Detect AprilTag in test image.
[~,~,aprilTagToCameraTform] = readAprilTag(undistortedTestImage,tagFamily,intrinsics,tagSize);
aprilTagToCameraTform   
% 
% Find the transformation from the camera to the robot base.
cameraToBaseTestTform = rigidtform3d(endEffectorToBaseTformTest * cameraToEndEffectorTform.A);
% 
% Find the transformation from the April Tag to the robot base.
tagToBaseTestTform = cameraToBaseTestTform.A * aprilTagToCameraTform.A;
cubePosition = tagToBaseTestTform(1:3,4) * 1000;
cubePosition(1) = cubePosition(1) * (-1);
cubePosition(2) = cubePosition(2) * (-1);
cubePosition
% 
show(robotModel,testPoseRad);
hold on
% 
% % Show the estimated positions and orientations of the camera and cube.
plotCamera(AbsolutePose = cameraToBaseTestTform, Opacity=0, size=0.02)
scatter3(cubePosition(1), cubePosition(2), cubePosition(3), 300, 'square', 'filled')
cubeRotationQuaternion = rotm2quat(tagToBaseTestTform(1:3,1:3));
plotTransforms(cubePosition', cubeRotationQuaternion)
title("Robot Arm and Estimated Position of AprilTag Cube")
xlim([-.3,.9])
ylim([-.5, .5])
zlim([-.2,.9])

%
% centroid = [538.66 320.33]; % pixel detectado pela YOLO (1x2)
% 
% % Corrigindo o pixel
% undCentroid = undistortPoints(centroid, intrinsics);
% 
% u_corr = undCentroid(1);
% v_corr = undCentroid(2);
% pixel = [u_corr; v_corr; 1];
% XYZ_Cam = (614.2-137)/1000 * (intrinsics.K \ pixel);
% pose_foto = [71.43 -70.08 54.03 -73.90 -89.77 -17.27]';
% testPoseRad = deg2rad(pose_foto);
% endEffectorToBaseTformTest = getTransform(robotModel,testPoseRad,"tool0");
% 
% cameraToBaseTestTform = rigidtform3d(endEffectorToBaseTformTest * cameraToEndEffectorTform.A);
% 
% % Convert to robot base frame
% XYZ_Base = transformPointsForward(cameraToBaseTestTform, XYZ_Cam')*1000
% % XYZ_Base(1) = XYZ_Base(1)+1.33;
% % XYZ_Base(2) = XYZ_Base(2)-2.45;
% XYZ_Base(1) = XYZ_Base(1)+1.7;
% XYZ_Base(2) = XYZ_Base(2);
% XYZ_Base