clear, close all
pyenv
pose_home = deg2rad([90, -90, 90, -90, -90, 0]);
pose_april1 = deg2rad([90.61, -54.65, 61.84, -97.14, -89.76, -2.92]);
pose_april2 = deg2rad([58.67, -51.91, 56.9, -94.95, -89.77, -34.87]);
pose_april3 = deg2rad([89.65, -95.83, 117.11, -111.23, -89.73, -3.86]);
pose_april4 = deg2rad([36.82, -90.91, 112.61, -111.65, -89.73, -56.68]);
pose_april5 = deg2rad([-86.13, -111.10, 127.4, -106.25, -89.72, -67.19]);
pose_home_py = py.list(pose_home);

pose_april1_py = py.list(pose_april1);
pose_april2_py = py.list(pose_april2);
pose_april3_py = py.list(pose_april3);
pose_april4_py = py.list(pose_april4);
pose_april5_py = py.list(pose_april5);

coord_real_apriltag1 = [223.19 -812.21 11.94];
coord_real_apriltag2 = [-219.0 -819.97 15.21];
coord_real_apriltag3 = [222.78 -458.57 9.56];
coord_real_apriltag4 = [-212.74 -455.9 11.12];
coord_real_apriltag5 = [-250.86 256.34 -6.05];

blocos = "B" + (1:9);
apriltags = (1:5);

% B1 = Metodo: Tsai   | Iluminacao: Low
% B2 = Metodo: Tsai   | Iluminacao: Medium
% B3 = Metodo: Tsai   | Iluminacao: High
% B4 = Metodo: Parkin | Iluminacao: Low
% B5 = Metodo: Parkin | Iluminacao: Medium
% B6 = Metodo: Parkin | Iluminacao: High
% B7 = Metodo: XC2    | Iluminacao: Low
% B8 = Metodo: XC2    | Iluminacao: Medium
% B9 = Metodo: XC2    | Iluminacao: High

randList = randperm(length(blocos));
shuffledBlocos = blocos(randList);

disp(shuffledBlocos)

ld = load("data/bestCameraParams_v2.mat").bestParams;
intrinsics = ld.Intrinsics;

cameraToEndEffectorTform_tsai = load("data/cameraToEndEffectorTform_tsai.mat").cameraToEndEffectorTform;
cameraToEndEffectorTform_parkin = load("data/cameraToEndEffectorTform_parkin.mat").cameraToEndEffectorTform_park;
cameraToEndEffectorTform_xc2 = load("data/cameraToEndEffectorTform_xc2.mat").cameraToEndEffectorTform_xc2;

robotModel = loadrobot("universalUR16e");
robotModel.DataFormat = "column";

tagFamily = 'tag36h11';
tagSize = .049;
ilumi_last = "Low";

% logData = table;
csvFile = "Experimento_v2.csv";

for bloco = shuffledBlocos
    if bloco == "B1"
        metodo = "Tsai";
        ilumi = "Low";
    
    elseif bloco == "B2"
        metodo = "Tsai";
        ilumi = "Medium";

    elseif bloco == "B3"
        metodo = "Tsai";
        ilumi = "High";
    
    elseif bloco == "B4"
        metodo = "Parkin";
        ilumi = "Low";

    elseif bloco == "B5"
        metodo = "Parkin";
        ilumi = "Medium";

    elseif bloco == "B6"
        metodo = "Parkin";
        ilumi = "High";

    elseif bloco == "B7"
        metodo = "XC2";
        ilumi = "Low";

    elseif bloco == "B8"
        metodo = "XC2";
        ilumi = "Medium";

    elseif bloco == "B9"
        metodo = "XC2";
        ilumi = "High";
    end

    switch metodo
        case "Tsai"
            cameraToEndEffectorTform = cameraToEndEffectorTform_tsai;
        case "Parkin"
            cameraToEndEffectorTform = cameraToEndEffectorTform_parkin;
        case "XC2"
            cameraToEndEffectorTform = cameraToEndEffectorTform_xc2;
    end

    if ilumi ~= ilumi_last
        disp("Ajuste iluminação para: " + ilumi);
        input("Pressione ENTER para continuar...");
    end

    apriltag_list = apriltags(randperm(length(apriltags)));

    for apriltag = apriltag_list
        
        for index = 1:30

            switch apriltag
                case 1
                    pose_py = pose_april1_py;
                    pose_joint = pose_april1';
                    coord_real = coord_real_apriltag1;
                case 2
                    pose_py = pose_april2_py;
                    pose_joint = pose_april2';
                    coord_real = coord_real_apriltag2;
                case 3
                    pose_py = pose_april3_py;
                    pose_joint = pose_april3';
                    coord_real = coord_real_apriltag3;
                case 4
                    pose_py = pose_april4_py;
                    pose_joint = pose_april4';
                    coord_real = coord_real_apriltag4;
                case 5
                    pose_py = pose_april5_py;
                    pose_joint = pose_april5';
                    coord_real = coord_real_apriltag5;
            end
        
            img_py = py.move_takephoto.move_and_capture(pose_home_py, pose_py);  
            img = uint8(img_py);
            img = img(:,:, [3 2 1]);
        
            % --- gerar nome da imagem ---
            timestamp = datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS');
            timestamp_str = datestr(timestamp,'yyyy-mm-dd_HH-MM-SS-FFF');
            filename = "img_" + timestamp_str + ".png";
        
            undistortedImage = undistortImage(img,ld);
            endEffectorToBaseTform = getTransform(robotModel,pose_joint,"tool0");
        
            [~,~,aprilTagToCameraTform] = readAprilTag(undistortedImage,tagFamily,intrinsics,tagSize);
            % 
            % Find the transformation from the camera to the robot base.
            cameraToBaseTestTform = rigidtform3d(endEffectorToBaseTform * cameraToEndEffectorTform.A);
            % 
            % Find the transformation from the April Tag to the robot base.
            tagToBaseTestTform = cameraToBaseTestTform.A * aprilTagToCameraTform.A;
            cubePosition = tagToBaseTestTform(1:3,4) * 1000;
            cubePosition(1) = cubePosition(1) * (-1);
            cubePosition(2) = cubePosition(2) * (-1);
            cubePosition
        
            save_csv(csvFile, filename, bloco, ilumi, apriltag, coord_real, cubePosition, timestamp, metodo);
        
        end

    end
    ilumi_last = ilumi;
end
