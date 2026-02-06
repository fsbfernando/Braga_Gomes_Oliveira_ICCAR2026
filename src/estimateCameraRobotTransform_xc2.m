function cameraPose = estimateCameraRobotTransform_xc2(camExtrinsics,gripperPose,configuration)
% estimateCameraRobotTransform_XC2  Hand-eye calibration using XC2 cost (Eq. 6)
%
% Signature, inputs, and output are intentionally identical to
% estimateCameraRobotTransform (Tsai-Lenz) so you can swap calls 1:1.
%
% Inputs:
%   camExtrinsics : rigidtform3d vector
%   gripperPose   : rigidtform3d vector
%   configuration : "moving-camera" or "stationary-camera"
%
% Output:
%   cameraPose    : rigidtform3d (same type/meaning as Tsai: solves Hgij*X = X*Hcij)

%   Copyright 2026

    arguments
        camExtrinsics rigidtform3d {mustBeVector}
        gripperPose   rigidtform3d {mustBeVector}
        configuration {mustBeTextScalar}
    end

    % Same validations as Tsai-Lenz wrapper
    iValidateInputLengths(camExtrinsics,gripperPose);
    iValidateConfiguration(configuration);
    iValidatePoseVariance(camExtrinsics,gripperPose);

    cameraPose = iCalibrateHandEye_XC2(camExtrinsics,gripperPose,configuration);
end

%--------------------------------------------------------------------------
function iValidateInputLengths(camExtrinsics,gripperPose)
    msgId = 'vision:estimateCameraRobotTransform:inputLengthMustMatch';
    vision.internal.errorIf(length(camExtrinsics) ~= length(gripperPose),msgId);

    minNumPoses = 2;
    numPoses = length(camExtrinsics);
    msgId = 'vision:estimateCameraRobotTransform:insufficientCalibrationData';
    vision.internal.errorIf(numPoses < minNumPoses,msgId);
end

%--------------------------------------------------------------------------
function iValidateConfiguration(configuration)
    validatestring(configuration,{'moving-camera','stationary-camera'},...
        mfilename,"configuration");
end

%--------------------------------------------------------------------------
function iValidatePoseVariance(camExtrinsics,gripperPose)
    avgRotVar = iGetPoseVariance(camExtrinsics);
    msgId = 'vision:estimateCameraRobotTransform:repeatedInputs';
    vision.internal.errorIf(avgRotVar == 0,msgId,"cameraExtrinsics");

    avgRotVar = iGetPoseVariance(gripperPose);
    msgId = 'vision:estimateCameraRobotTransform:repeatedInputs';
    vision.internal.errorIf(avgRotVar == 0,msgId,"gripperPose");
end

%--------------------------------------------------------------------------
function avgRotVar = iGetPoseVariance(poses)
    numPoses = length(poses);
    rotVar = zeros(numPoses-1,1);
    for i = 1:(numPoses-1)
        j = i+1;
        rotPose1 = quaternion(poses(i).R,"rotmat","frame");
        rotPose2 = quaternion(poses(j).R,"rotmat","frame");
        rotVar(i) = 2*acos(abs(parts(rotPose2*conj(rotPose1))));
    end
    avgRotVar = mean(rotVar);
end

%--------------------------------------------------------------------------
function cameraPose = iCalibrateHandEye_XC2(camExtrinsics,gripperPose,configuration)
    % Build relative motions (Aij = Hgij, Bij = Hcij) with SAME convention as Tsai code
    [Aset,Bset] = iBuildRelativeMotions(camExtrinsics,gripperPose,configuration);

    % Initial guess: try Tsai-Lenz (same output type), otherwise identity
    x0 = iInitialGuessFromTsai(camExtrinsics,gripperPose,configuration); % 7x1 [qw qx qy qz tx ty tz]

    % Solve nonlinear least squares: minimize sum || log( inv(X*Bij*invX)*Aij ) ||^2
    % Levenberg-Marquardt
    if exist('lsqnonlin','file') == 2
        opts = optimoptions('lsqnonlin',...
            'Display','off',...
            'MaxIterations',200,...
            'FunctionTolerance',1e-12,...
            'StepTolerance',1e-12);
        x = lsqnonlin(@(p)iResidualXC2(p,Aset,Bset), x0, [], [], opts);
    else
        % Fallback without Optimization Toolbox: use fminsearch on SSE (slower/less robust)
        opts = optimset('Display','off','MaxIter',5000,'MaxFunEvals',20000);
        x = fminsearch(@(p)iSSE_XC2(p,Aset,Bset), x0, opts);
    end

    % Convert final parameters to rigidtform3d
    X = iParamsToTform(x);
    cameraPose = rigidtform3d(X(1:3,1:3), X(1:3,4));
end

%--------------------------------------------------------------------------
function [Aset,Bset] = iBuildRelativeMotions(camExtrinsics,gripperPose,configuration)
    numPoses = length(camExtrinsics);
    numPairs = numPoses*(numPoses-1)/2;

    Aset = cell(numPairs,1);
    Bset = cell(numPairs,1);

    pairCount = 0;
    for i = 1:numPoses
        for j = i+1:numPoses
            % Camera extrinsics
            Hci = camExtrinsics(i).A;
            Hcj = camExtrinsics(j).A;

            % Gripper/base poses (match Tsai convention exactly)
            if (configuration == "stationary-camera")
                Hgi = invert(gripperPose(i)).A;
                Hgj = invert(gripperPose(j)).A;
            else
                Hgi = gripperPose(i).A;
                Hgj = gripperPose(j).A;
            end

            % Relative transforms (same lines as Tsai)
            Hgij = Hgj\Hgi;
            Hcij = Hcj/Hci;

            pairCount = pairCount + 1;
            Aset{pairCount} = Hgij;
            Bset{pairCount} = Hcij;
        end
    end
end

%--------------------------------------------------------------------------
function x0 = iInitialGuessFromTsai(camExtrinsics,gripperPose,configuration)
    % Default: identity (unit quaternion + zero translation)
    x0 = double([1;0;0;0; 0;0;0]);

    try
        % Use Tsai-Lenz as initializer if available in your path
        X0 = estimateCameraRobotTransform(camExtrinsics,gripperPose,configuration);
        R0 = X0.R;
        t0 = X0.Translation(:);

        q0 = quaternion(R0,"rotmat","frame");  % quaternion object
        p  = parts(q0);                       % returns [w x y z]
        if p(1) < 0
            p = -p; % keep a consistent hemisphere
        end

        x0 = [p(:); t0(:)];
    catch
        % keep identity
    end

    x0 = double(x0(:));
    if numel(x0) > 7, x0 = x0(1:7); end
    if numel(x0) < 7, x0 = double([1;0;0;0;0;0;0]); end

end

%--------------------------------------------------------------------------
function r = iResidualXC2(p,Aset,Bset)
    % Normalize quaternion inside the residual to keep rotation valid
    [q,t] = iSplitParams(p);
    q = iNormalizeQuat(q);

    X    = iQuatTransToTform(q,t);
    invX = iInvertTform(X);

    nPairs = numel(Aset);
    r = zeros(6*nPairs,1);

    for k = 1:nPairs
        A = Aset{k};
        B = Bset{k};

        % XC2 transport: A ≈ X*B*invX
        M = X * B * invX;

        % Use a proper SE(3) error: Delta = inv(M)*A (should be Identity)
        Delta = M \ A;

        % Rotation residual as axis-angle vector (3x1)
        rotVec = rotmat2vec3d(Delta(1:3,1:3));  % same helper used in Tsai codebase
        if any(~isfinite(rotVec))
            rotVec = zeros(3,1);
        end

        % Translation residual (3x1)
        transErr = Delta(1:3,4);

        idx = (k-1)*6 + (1:6);
        r(idx) = [rotVec(:); transErr(:)];
    end
end

%--------------------------------------------------------------------------
function sse = iSSE_XC2(p,Aset,Bset)
    rr = iResidualXC2(p,Aset,Bset);
    sse = rr.'*rr;
end

%--------------------------------------------------------------------------
function X = iParamsToTform(p)
    [q,t] = iSplitParams(p);
    q = iNormalizeQuat(q);
    X = iQuatTransToTform(q,t);
end

%--------------------------------------------------------------------------
function [q,t] = iSplitParams(p)
    % Robust split for params: allow row/col, allow extra elements.
    % Expected: [qw qx qy qz tx ty tz]
    
    % Convert to double vector safely
    try
        p = double(p);
    catch
        error('estimateCameraRobotTransform_XC2:BadParamType',...
              'Parameters must be numeric. Got type: %s', class(p));
    end

    % If it comes as a matrix, vectorize
    p = p(:);

    % If empty or too short, fail clearly
    if numel(p) < 7
        error('estimateCameraRobotTransform_XC2:BadParamSize',...
              'Expected at least 7 parameters [qw qx qy qz tx ty tz], got %d.', numel(p));
    end

    % If it comes longer (can happen), keep first 7
    p = p(1:7);

    q = p(1:4);
    t = p(5:7);
end

%--------------------------------------------------------------------------
function qn = iNormalizeQuat(q)
    q = q(:);
    n = norm(q);
    if n < eps
        qn = [1;0;0;0];
    else
        qn = q./n;
        if qn(1) < 0
            qn = -qn; % keep a consistent hemisphere
        end
    end
end

%--------------------------------------------------------------------------
function X = iQuatTransToTform(q,t)
    % q is [w x y z] (scalar-first), t is 3x1
    qobj = quaternion(q(1),q(2),q(3),q(4));
    R = rotmat(qobj,'frame');
    X = eye(4);
    X(1:3,1:3) = R;
    X(1:3,4)   = t(:);
end

%--------------------------------------------------------------------------
function invT = iInvertTform(T)
    R = T(1:3,1:3);
    t = T(1:3,4);
    invT = eye(4);
    invT(1:3,1:3) = R.';
    invT(1:3,4)   = -R.'*t;
end
