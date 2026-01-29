function cameraPose = estimateCameraRobotTransform_parkMartin(camExtrinsics, gripperPose, configuration)
% estimateCameraRobotTransform_parkMartin
% Drop-in replacement for MATLAB's estimateCameraRobotTransform, but using
% Park–Martin hand-eye calibration.
%
% Inputs:
%   camExtrinsics : rigidtform3d vector (camera poses / extrinsics)
%   gripperPose   : rigidtform3d vector (gripper poses)
%   configuration : "moving-camera" | "stationary-camera"
%
% Output:
%   cameraPose    : rigidtform3d (Hcg), camera pose relative to gripper.
%
% Method:
%   Park, F. C., Martin, B. J. (1994). Robot Sensor Calibration:
%   Solving AX = XB on the Euclidean Group.
%
% Notes:
%   - Rotation: solve R that best maps rotation vectors b_k -> a_k (SVD).
%   - Translation: solve stacked linear system (Ra - I) t = R*tb - ta.

    arguments
        camExtrinsics rigidtform3d {mustBeVector}
        gripperPose   rigidtform3d {mustBeVector}
        configuration {mustBeTextScalar}
    end

    iValidateInputLengths(camExtrinsics, gripperPose);
    iValidateConfiguration(configuration);
    iValidatePoseVariance(camExtrinsics, gripperPose);

    cameraPose = iCalibrateHandEye_ParkMartin(camExtrinsics, gripperPose, configuration);
end

%--------------------------------------------------------------------------
function iValidateInputLengths(camExtrinsics, gripperPose)
    if numel(camExtrinsics) ~= numel(gripperPose)
        error('vision:estimateCameraRobotTransform:inputLengthMustMatch', ...
              'camExtrinsics and gripperPose must have the same length.');
    end
    if numel(camExtrinsics) < 2
        error('vision:estimateCameraRobotTransform:insufficientCalibrationData', ...
              'At least 2 poses are required.');
    end
end

%--------------------------------------------------------------------------
function iValidateConfiguration(configuration)
    validatestring(configuration, {'moving-camera','stationary-camera'}, mfilename, "configuration");
end

%--------------------------------------------------------------------------
function iValidatePoseVariance(camExtrinsics, gripperPose)
    avgRotVar = iGetPoseVariance(camExtrinsics);
    if avgRotVar == 0
        error('vision:estimateCameraRobotTransform:repeatedInputs', ...
              'Repeated inputs detected in cameraExtrinsics.');
    end

    avgRotVar = iGetPoseVariance(gripperPose);
    if avgRotVar == 0
        error('vision:estimateCameraRobotTransform:repeatedInputs', ...
              'Repeated inputs detected in gripperPose.');
    end
end

%--------------------------------------------------------------------------
function avgRotVar = iGetPoseVariance(poses)
    numPoses = numel(poses);
    rotVar = zeros(max(numPoses-1,1),1);
    for i = 1:(numPoses-1)
        j = i+1;
        R1 = poses(i).R;
        R2 = poses(j).R;
        dR = R2 * R1';
        w  = so3LogVec(dR);
        rotVar(i) = norm(w);
    end
    avgRotVar = mean(rotVar);
end

%--------------------------------------------------------------------------
function cameraPose = iCalibrateHandEye_ParkMartin(camExtrinsics, gripperPose, configuration)
    [Rcg, rflag] = iEstimateRotationMatrix_ParkMartin(camExtrinsics, gripperPose, configuration);
    [tcg, tflag] = iEstimateTranslationVector_ParkMartin(camExtrinsics, gripperPose, configuration, Rcg);

    cameraPose = rigidtform3d(Rcg, tcg(:)'); % translation row

    % Keep "flags" style similar to the original (lsqr flags).
    if rflag ~= 0
        error('vision:estimateCameraRobotTransform:rotationSolveFailed', ...
              'Rotation solve reported numerical issues (flag=%d).', rflag);
    end
    if tflag ~= 0
        error('vision:estimateCameraRobotTransform:translationSolveFailed', ...
              'Translation solve reported numerical issues (flag=%d).', tflag);
    end
end

%--------------------------------------------------------------------------
function [Rcg, rflag] = iEstimateRotationMatrix_ParkMartin(camExtrinsics, gripperPose, configuration)
% Rotation solve (Park–Martin style):
% For each pair (i,j):  Aij X = X Bij
% with Aij = Hgij, Bij = Hcij
% Convert rotations to vectors via log in so(3):
%   a_k = log(Ra_k)^\vee
%   b_k = log(Rb_k)^\vee
% Solve R that best maps b -> a: minimize Σ||a - R b||^2.
% Closed form via SVD of M = Σ a b^T.

    numPoses = numel(camExtrinsics);
    M = zeros(3,3);

    % Count only "usable" pairs (avoid near-zero rotations).
    usable = 0;
    for i = 1:numPoses
        for j = i+1:numPoses
            Hci = camExtrinsics(i).A;
            Hcj = camExtrinsics(j).A;

            if configuration == "stationary-camera"
                Hgi = invert(gripperPose(i)).A;
                Hgj = invert(gripperPose(j)).A;
            else
                Hgi = gripperPose(i).A;
                Hgj = gripperPose(j).A;
            end

            Hgij = Hgj \ Hgi;  % inv(Hgj)*Hgi
            Hcij = Hcj / Hci;  % Hcj*inv(Hci)

            Ra = Hgij(1:3,1:3);
            Rb = Hcij(1:3,1:3);

            a = so3LogVec(Ra);
            b = so3LogVec(Rb);

            % Skip degenerate very-small rotations (adds noise / rank issues)
            if norm(a) < 1e-10 || norm(b) < 1e-10
                continue;
            end

            M = M + (a * b.');
            usable = usable + 1;
        end
    end

    if usable < 1
        % Not enough informative motion for rotation
        Rcg = eye(3);
        rflag = 1;
        return;
    end

    [U,~,V] = svd(M);
    S = eye(3);
    if det(U*V') < 0
        S(3,3) = -1;
    end
    Rcg = U*S*V';

    % Basic sanity
    if any(isnan(Rcg), "all") || abs(det(Rcg) - 1) > 1e-3
        rflag = 1;
    else
        rflag = 0;
    end
end

%--------------------------------------------------------------------------
function [tcg, tflag] = iEstimateTranslationVector_ParkMartin(camExtrinsics, gripperPose, configuration, Rcg)
% Translation solve from AX=XB:
%   (Ra - I) t = R * tb - ta
% where A=[Ra,ta], B=[Rb,tb], X=[R,t]

    numPoses = numel(camExtrinsics);
    numPairs = numPoses*(numPoses-1)/2;

    A = zeros(3*numPairs, 3);
    b = zeros(3*numPairs, 1);

    pairCount = 0;
    for i = 1:numPoses
        for j = i+1:numPoses
            Hci = camExtrinsics(i).A;
            Hcj = camExtrinsics(j).A;

            if configuration == "stationary-camera"
                Hgi = invert(gripperPose(i)).A;
                Hgj = invert(gripperPose(j)).A;
            else
                Hgi = gripperPose(i).A;
                Hgj = gripperPose(j).A;
            end

            Hgij = Hgj \ Hgi;
            Hcij = Hcj / Hci;

            Ra = Hgij(1:3,1:3);
            ta = Hgij(1:3,4);

            % B = Hcij (camera relative motion)
            tb = Hcij(1:3,4);

            pairCount = pairCount + 1;
            idx = (pairCount-1)*3 + (1:3);

            A(idx,:) = (Ra - eye(3));
            b(idx)   = (Rcg * tb - ta);
        end
    end

    % Solve least squares. Use lsqr to mimic MathWorks style.
    try
        [tcg, tflag] = lsqr(A, b);
    catch
        % Fallback if lsqr unavailable
        tcg = A \ b;
        tflag = 0;
    end

    if any(isnan(tcg))
        tflag = 1;
        tcg = zeros(3,1);
    end
end

%--------------------------------------------------------------------------
% so3LogVec: returns rotation vector w (3x1) such that exp([w]x) = R
% Robust-ish for small angles and near pi.
%--------------------------------------------------------------------------
function w = so3LogVec(R)
    % Clamp trace for numerical safety
    tr = trace(R);
    c = (tr - 1) / 2;
    c = min(1, max(-1, c));
    theta = acos(c);

    if theta < 1e-12
        w = zeros(3,1);
        return;
    end

    if abs(pi - theta) < 1e-6
        % Near pi: use diagonal-based axis extraction (one common approach)
        % Choose the largest diagonal element for stability
        ax = zeros(3,1);
        [~,k] = max([R(1,1), R(2,2), R(3,3)]);
        switch k
            case 1
                ax(1) = sqrt(max(0, (R(1,1)+1)/2));
                ax(2) = R(1,2)/(2*max(ax(1),1e-12));
                ax(3) = R(1,3)/(2*max(ax(1),1e-12));
            case 2
                ax(2) = sqrt(max(0, (R(2,2)+1)/2));
                ax(1) = R(1,2)/(2*max(ax(2),1e-12));
                ax(3) = R(2,3)/(2*max(ax(2),1e-12));
            case 3
                ax(3) = sqrt(max(0, (R(3,3)+1)/2));
                ax(1) = R(1,3)/(2*max(ax(3),1e-12));
                ax(2) = R(2,3)/(2*max(ax(3),1e-12));
        end
        ax = ax / max(norm(ax), 1e-12);
        w = theta * ax;
        return;
    end

    % Standard log map
    wx = (R - R') / (2*sin(theta));
    ax = [wx(3,2); wx(1,3); wx(2,1)];
    w  = theta * ax;
end
