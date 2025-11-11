classdef ekfNavINS < handle
    properties (Constant)
        % Noise and covariance related constants
        SIG_W_A = 0.05; % standard deviation of accelerometer output noise (m/s²)
        SIG_W_G = 0.00175; % standard deviation of gyro output noise (rad/s)
        SIG_A_D = 0.01; % standard deviation of Accelerometer Markov Bias
        TAU_A = 100.0; % % correlation time constant for accelerometer bias (s)
        SIG_G_D = 0.00025; % standard deviation of correlated gyro bias
        TAU_G = 50.0; % Correlation time constant for gyro bias (s)
        SIG_GPS_P_NE = 3.0; % standard deviation of GPS North East position measurement noise (m)
        SIG_GPS_P_D = 6.0; % standard deviation of GPS Down position measurement noise (m)
        SIG_GPS_V_NE = 0.5; % standard deviation of GPS North East velocity measurement noise (m/s)
        SIG_GPS_V_D = 1.0; % standard deviation of GPS Down velocity measurement noise (m/s)
        P_P_INIT = 10.0; % initial position covariance
        P_V_INIT = 1.0; % initial velocity covariance
        P_A_INIT = 0.34906; % initial attitude covariance
        P_HDG_INIT = 3.14159; % initial heading covariance
        P_AB_INIT = 0.9810; % initial accelerometer bias covariance
        P_GB_INIT = 0.01745; % initial gyro bias covariance
        G = 9.807; % acceleration due to gravity (m/s²)
        ECC2 = 0.0066943799901;  % major eccentricity squared
        EARTH_RADIUS = 6378137.0; % earth semi-major axis radius (m)
    end

    properties
        % system status flag
        initialized_ = false; % whether EKF has been initialized

        % time related variables
        tprev; % previous update time
        previousTOW = 0; % previous GPS time of week

        % Euler angle attitude estimates (rad)
        phi = 0; % Roll angle (rotation around X-axis)
        theta = 0; % Pitch angle (rotation around Y-axis)
        psi = 0; % Yaw/Heading angle (rotation around Z-axis)

        % NED frame velocity estimates (m/s)
        vn_ins = 0; % North velocity
        ve_ins = 0; % East velocity
        vd_ins = 0; % Down velocity

        % Geodetic position estimates (rad, rad, m)
        lat_ins = 0; % latitude
        lon_ins = 0; % longitude
        alt_ins = 0; % altitude

        % Magnetometer related variables
        Bxc = 0; % X magnetic component corrected for roll and pitch
        Byc = 0; % Y magnetic component corrected for roll and pitch

        % accelerometer bias estimates (m/s²)
        abx = 0; % X-axis accelerometer bias
        aby = 0; % Y-axis accelerometer bias
        abz = 0; % Z-axis accelerometer bias

        % Gyro bias estimates (rad/s)
        gbx = 0; % X-axis gyro bias
        gby = 0; % Y-axis gyro bias
        gbz = 0; % Z-axis gyro bias

        % ?? lack Re Rn denom ekfNavINS.h:170

        % EKF related matrices
        Fs; % state transition Jacobian matrix (15x15)
        PHI; % discrete state transition matrix (15x15)
        P; % state covariance matrix (15x15)
        Gs; % process noise distribution matrix (15x12)
        Rw; % process noise covariance matrix (12x12)
        Q; % discrete process noise covariance matrix (15x15)

        % physical quantity vectors
        grav; % gravity vector [0; 0; G] (3x1)
        om_ib; % Gyro angular rate measurements (minus bias) (rad/s)
        f_b; % specific force measurements (minus bias) (m/s²)

        % coordinate transformation matrices
        C_N2B; % Navigation to Body frame rotation matrix
        C_B2N; % Body to Navigation frame rotation matrix (transpose of C_N2B)

        dx;
        dxd;
        estimated_ins;

        % state vectors in matrix form
        V_ins; % INS velocity estimate [vn; ve; vd] (m/s)
        lla_ins; % INS position estimate [lat; lon; alt] (rad, rad, m)
        V_gps; % GPS velocity measurement [vn; ve; vd] (m/s)
        lla_gps; % GPS position measurement [lat; lon; alt] (rad, rad, m)

        % coordinate transformation related
        pos_ecef_ins; % ECEF coordinates of INS position
        pos_ecef_gps; % ECEF coordinates of GPS position
        pos_ned_gps; % NED coordinates of GPS position (relative to INS position)

        % quaternion related
        quat; % current attitude quaternion [w; x; y; z]
        dq; % quaternion delta

        % Kalman filter related
        y; % measurement residual vector (6x1)
        R; % GPS measurement noise covariance matrix (6x6)
        x; % state correction vector (15x1)
        K; % Kalman gain matrix (15x6)
        H; % measurement matrix (6x15)

        % sensor data structures
        gpsCoor; % GPS coordinate data
        gpsVel; % GPS velocity data
        imuDat; % IMU sensor data
    end

    methods
        function obj = ekfNavINS()
            % EKF Navigation INS class constructor
            % Initializes all matrices and variables to appropriate sizes and initial values

            obj.Fs = eye(15);
            obj.PHI = zeros(15);
            obj.P = zeros(15);
            obj.Gs = zeros(15, 12);
            obj.Rw = zeros(12);
            obj.Q = zeros(15);
            % obj.grav = [0; 0; obj.G]; % ?? not zeros(3,1) ekfNavINS.h:183
            obj.grav = zeros(3, 1);
            obj.om_ib = zeros(3, 1);
            obj.f_b = zeros(3, 1);
            % obj.C_N2B = eye(3); % ?? not zeros(3) ekfNavINS.h:189
            obj.C_N2B = zeros(3, 3);
            % obj.C_B2N = eye(3); % ?? not zeros(3) ekfNavINS.h:191
            obj.C_B2N = zeros(3, 3);
            obj.dx = zeros(3, 1);
            obj.dxd = zeros(3, 1);
            obj.estimated_ins = zeros(3, 1);
            obj.V_ins = zeros(3, 1);
            obj.lla_ins = zeros(3, 1);
            obj.V_gps = zeros(3, 1);
            obj.lla_gps = zeros(3, 1);
            obj.pos_ecef_ins = zeros(3, 1);
            % ?? lack obj.pos_ned_ins = zeros(3,1) ekfNavINS.h:208
            obj.pos_ecef_gps = zeros(3, 1);
            obj.pos_ned_gps = zeros(3, 1);
            % obj.quat = [1; 0; 0; 0]; % ?? not zeros(4,1) ekfNavINS.h:216
            obj.quat = zeros(4, 1);
            obj.dq = zeros(4, 1);
            obj.y = zeros(6, 1);
            obj.R = zeros(6);
            obj.x = zeros(15, 1);
            obj.K = zeros(15, 6);
            obj.H = zeros(6, 15);

            % ?? lack sk=skew symmetric ekfNavINS.h:226

            obj.gpsCoor = struct('lat', 0, 'lon', 0, 'alt', 0);
            obj.gpsVel = struct('vN', 0, 'vE', 0, 'vD', 0);
            obj.imuDat = struct('gyroX', 0, 'gyroY', 0, 'gyroZ', 0, ...
                'accX', 0, 'accY', 0, 'accZ', 0, ...
                'hX', 0, 'hY', 0, 'hZ', 0);
        end

        % done
        function ekf_init(obj, time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz)
            % EKF initialization function
            % Inputs:
            %   time - current time (ms)
            %   vn, ve, vd - initial North, East, Down velocities (m/s)
            %   lat, lon, alt - initial latitude, longitude, altitude (rad, rad, m)
            %   p, q, r - Gyro XYZ angular rates (rad/s)
            %   ax, ay, az - accelerometer XYZ measurements (m/s²)
            %   hx, hy, hz - magnetometer XYZ measurements (mT)

            % initialize gyro biases with first measurement values
            obj.gbx = p;
            obj.gby = q;
            obj.gbz = r;

            % calculate initial attitude from accelerometer and magnetometer
            [obj.theta, obj.phi, obj.psi] = obj.getPitchRollYaw(ax, ay, az, hx, hy, hz);

            % convert Euler angles to quaternion representation
            obj.quat = obj.toQuaternion(obj.phi, obj.theta, obj.psi);

            % initialize gravity vector
            %obj.grav = [0; 0; obj.G];
            obj.grav(3) = obj.G;

            % initialize measurement matrix H (GPS position and velocity measurements)
            obj.H(1:5, 1:5) = eye(5);

            % initialize process noise covariance matrix Rw
            obj.Rw(1:3, 1:3) = (obj.SIG_W_A^2) * eye(3);
            obj.Rw(4:6, 4:6) = (obj.SIG_W_G^2) * eye(3);
            obj.Rw(7:9, 7:9) = 2 * (obj.SIG_A_D^2) / obj.TAU_A * eye(3);
            obj.Rw(10:12, 10:12) = 2 * (obj.SIG_G_D^2) / obj.TAU_G * eye(3);

            % initialize state covariance matrix P
            obj.P(1:3, 1:3) = (obj.P_P_INIT^2) * eye(3);
            obj.P(4:6, 4:6) = (obj.P_V_INIT^2) * eye(3);
            obj.P(7:8, 7:8) = (obj.P_A_INIT^2) * eye(2);
            obj.P(9, 9) = obj.P_HDG_INIT^2;
            obj.P(10:12, 10:12) = (obj.P_AB_INIT^2) * eye(3);
            obj.P(13:15, 13:15) = (obj.P_GB_INIT^2) * eye(3);

            % initialize measurement noise covariance matrix R
            obj.R(1:2, 1:2) = (obj.SIG_GPS_P_NE^2) * eye(2);
            obj.R(3, 3) = obj.SIG_GPS_P_D^2;
            obj.R(4:5, 4:5) = (obj.SIG_GPS_V_NE^2) * eye(2);
            obj.R(6, 6) = obj.SIG_GPS_V_D^2;

            % initialize states using GPS data
            obj.lat_ins = lat;
            obj.lon_ins = lon;
            obj.alt_ins = alt;
            obj.vn_ins = vn;
            obj.ve_ins = ve;
            obj.vd_ins = vd;

            % initialize specific force measurements
            obj.f_b = [ax; ay; az];

            % save initial time
            obj.tprev = time;
        end

        % done
        function ekf_update(obj, time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz)
            % Main EKF update function - handles IMU prediction and GPS measurement update
            % Input parameters same as ekf_init

            % check if EKF is initialized
            if ~obj.initialized_
                % if not initialized, perform initialization
                obj.ekf_init(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
                obj.initialized_ = true;
            else
                % calculate time delta (convert to seconds)
                dt = (time - obj.tprev) / 1e6;

                % update sensor biases and compute corrected measurements
                obj.updateBias(ax, ay, az, p, q, r);

                % update INS state vectors
                obj.updateINS();

                % attitude update - using quaternion integration
                % build quaternion delta
                obj.dq = [1; 0.5 * obj.om_ib(1) * dt; 0.5 * obj.om_ib(2) * dt; 0.5 * obj.om_ib(3) * dt];

                % update attitude using quaternion multiplication
                obj.quat = obj.qmult(obj.quat, obj.dq);
                % normalize quaternion
                obj.quat = obj.quat / norm(obj.quat);

                % avoid quaternion sign flip (keep w component positive)
                if obj.quat(1) < 0
                    obj.quat = -obj.quat;
                end

                % calculate coordinate transformation matrices
                % navigation to Body frame direction cosine matrix
                obj.C_N2B = obj.quat2dcm(obj.quat);
                % body to Navigation frame direction cosine matrix (transpose)
                obj.C_B2N = obj.C_N2B';

                % extract Euler angles from quaternion for output
                [obj.phi, obj.theta, obj.psi] = obj.toEulerAngles(obj.quat);

                % velocity update - transform specific force from body to navigation frame and add gravity
                obj.dx = obj.C_B2N * obj.f_b + obj.grav;
                % integrate to get new velocities
                obj.vn_ins = obj.vn_ins + dt * obj.dx(1);
                obj.ve_ins = obj.ve_ins + dt * obj.dx(2);
                obj.vd_ins = obj.vd_ins + dt * obj.dx(3);

                % position update - compute geodetic coordinate rates and integrate
                obj.dxd = obj.llarate(obj.V_ins, obj.lla_ins);
                obj.lat_ins = obj.lat_ins + dt * obj.dxd(1);
                obj.lon_ins = obj.lon_ins + dt * obj.dxd(2);
                obj.alt_ins = obj.alt_ins + dt * obj.dxd(3);

                % update Jacobian matrix (linearization of state transition matrix)
                obj.updateJacobianMatrix();

                % update process noise and covariance matrix time propagation
                obj.updateProcessNoiseCovarianceTime(dt);

                % GPS measurement update (when new GPS data is available)
                if (time - obj.tprev) > 0
                    % save GPS measurements
                    obj.lla_gps = [lat; lon; alt];
                    obj.V_gps = [vn; ve; vd];

                    % update INS state vectors
                    obj.updateINS();

                    % calculate measurement residuals (difference between GPS measurements and INS predictions)
                    obj.updateCalculatedVsPredicted();

                    % calculate Kalman gain
                    obj.K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + obj.R);

                    % update covariance matrix (using Joseph form for numerical stability)
                    I_temp = eye(15);
                    obj.P = (I_temp - obj.K * obj.H) * obj.P * (I_temp - obj.K * obj.H)' + obj.K * obj.R * obj.K';

                    % state correction
                    obj.x = obj.K * obj.y;

                    % apply Kalman filter corrections to 15 state variables
                    obj.update15statesAfterKF();

                    % update timestamp
                    obj.tprev = time;
                end

                % update sensor biases again
                obj.updateBias(ax, ay, az, p, q, r);
            end
        end

        % done
        function [theta, phi, psi] = getPitchRollYaw(obj, ax, ay, az, hx, hy, hz)
            % Calculate initial attitude and heading from accelerometer and magnetometer
            % Inputs: raw accelerometer and magnetometer measurements
            % Outputs: pitch, roll, yaw angles (radians)

            % calculate pitch and roll using gravity vector
            % pitch angle
            theta = asin(ax/obj.G);
            % roll angle
            phi = -asin(ay/(obj.G * cos(theta)));

            % magnetometer heading correction - compensate for roll and pitch angles
            % corrected X magnetic component
            obj.Bxc = hx * cos(theta) + (hy * sin(phi) + hz * cos(phi)) * sin(theta);
            % corrected Y magnetic component
            obj.Byc = hy * cos(phi) - hz * sin(phi);

            % calculate heading angle
            psi = -atan2(obj.Byc, obj.Bxc);
        end

        % done
        function updateINS(obj)
            % Update INS state vectors - convert scalar values to matrix form
            obj.lla_ins = [obj.lat_ins; obj.lon_ins; obj.alt_ins];
            obj.V_ins = [obj.vn_ins; obj.ve_ins; obj.vd_ins];
        end

        % done
        function updateBias(obj, ax, ay, az, p, q, r)
            % Update sensor biases and compute corrected measurements
            % Inputs: raw accelerometer and gyro measurements

            % accelerometer measurements minus bias estimates
            obj.f_b = [ax - obj.abx; ay - obj.aby; az - obj.abz];
            % gyro measurements minus bias estimates
            obj.om_ib = [p - obj.gbx; q - obj.gby; r - obj.gbz];
        end

        % done
        function updateJacobianMatrix(obj)
            % Update Jacobian matrix (linearization of state transition matrix)
            % Describes how state variables influence each other

            % reset Jacobian matrix
            obj.Fs(:) = 0;

            % position to velocity relationship (position rate equals velocity)
            obj.Fs(1:3, 4:6) = eye(3);

            % velocity to position relationship - account for Earth curvature and gravity variation
            obj.Fs(6, 3) = -2.0 * obj.G / obj.EARTH_RADIUS;

            % velocity to attitude relationship - specific force effect on velocity change
            obj.Fs(4:6, 7:9) = -2.0 * obj.C_B2N * obj.sk(obj.f_b);

            % velocity to accelerometer bias relationship
            obj.Fs(4:6, 10:12) = -obj.C_B2N;

            % attitude to attitude relationship - angular rate effect on attitude change
            obj.Fs(7:9, 7:9) = -obj.sk(obj.om_ib);

            % attitude to gyro bias relationship
            obj.Fs(7:9, 13:15) = -0.5 * eye(3);

            % accelerometer Markov bias model
            obj.Fs(10:12, 10:12) = -1.0 / obj.TAU_A * eye(3);
            % gyro Markov bias model
            obj.Fs(13:15, 13:15) = -1.0 / obj.TAU_G * eye(3);
        end

        % done
        function updateProcessNoiseCovarianceTime(obj, dt)
            % Update process noise and covariance matrix time propagation
            % Input: dt - time delta (seconds)

            % calculate discrete state transition matrix (first order approximation)
            obj.PHI = eye(15) + obj.Fs * dt;

            % process noise distribution matrix
            obj.Gs(:) = 0;
            % specific force noise to velocity states
            obj.Gs(4:6, 1:3) = -obj.C_B2N;
            % angular rate noise to attitude states
            obj.Gs(7:9, 4:6) = -0.5 * eye(3);
            % bias noise to bias states
            obj.Gs(10:15, 7:12) = eye(6);

            % calculate discrete process noise covariance matrix
            Q_temp = obj.PHI * dt * obj.Gs * obj.Rw * obj.Gs';
            % ensure symmetry
            obj.Q = 0.5 * (Q_temp + Q_temp');

            % covariance time update (prediction step)
            P_temp = obj.PHI * obj.P * obj.PHI' + obj.Q;
            % ensure symmetry
            obj.P = 0.5 * (P_temp + P_temp');
        end

        % done
        function updateCalculatedVsPredicted(obj)
            % Calculate measurement residuals - difference between GPS measurements and INS predictions

            % convert INS position from geodetic to ECEF coordinates
            obj.pos_ecef_ins = obj.lla2ecef(obj.lla_ins);
            % convert GPS position from geodetic to ECEF coordinates
            obj.pos_ecef_gps = obj.lla2ecef(obj.lla_gps);
            % convert GPS-INS position difference from ECEF to NED frame (using INS position as reference)
            obj.pos_ned_gps = obj.ecef2ned(obj.pos_ecef_gps-obj.pos_ecef_ins, obj.lla_ins);

            % build measurement residual vector
            % position residuals (meters)
            obj.y(1) = obj.pos_ned_gps(1);  % north position difference
            obj.y(2) = obj.pos_ned_gps(2);  % east position difference
            obj.y(3) = obj.pos_ned_gps(3);  % down position difference
            % velocity residuals (m/s)
            obj.y(4) = obj.V_gps(1) - obj.V_ins(1); % north velocity difference
            obj.y(5) = obj.V_gps(2) - obj.V_ins(2); % east velocity difference
            obj.y(6) = obj.V_gps(3) - obj.V_ins(3); % down velocity difference
        end

        % done
        function update15statesAfterKF(obj)
            % Apply Kalman filter corrections to 15 state variables

            % position correction - convert state corrections to geodetic coordinate changes
            obj.estimated_ins = obj.llarate(obj.x(1:3), obj.lat_ins, obj.alt_ins);
            obj.lat_ins = obj.lat_ins + obj.estimated_ins(1);
            obj.lon_ins = obj.lon_ins + obj.estimated_ins(2);
            obj.alt_ins = obj.alt_ins + obj.estimated_ins(3);

            % velocity corrections
            obj.vn_ins = obj.vn_ins + obj.x(4);
            obj.ve_ins = obj.ve_ins + obj.x(5);
            obj.vd_ins = obj.vd_ins + obj.x(6);

            % attitude correction - using quaternion multiplication
            obj.dq = [1; obj.x(7); obj.x(8); obj.x(9)];
            obj.quat = obj.qmult(obj.quat, obj.dq);
            % quaternion normalization
            obj.quat = obj.quat / norm(obj.quat);

            % extract Euler angles from corrected quaternion
            [obj.phi, obj.theta, obj.psi] = obj.toEulerAngles(obj.quat);

            % sensor bias corrections
            obj.abx = obj.abx + obj.x(10);
            obj.aby = obj.aby + obj.x(11);
            obj.abz = obj.abz + obj.x(12);
            obj.gbx = obj.gbx + obj.x(13);
            obj.gby = obj.gby + obj.x(14);
            obj.gbz = obj.gbz + obj.x(15);
        end

        % done
        function C = sk(obj, w)
            % Convert vector to skew-symmetric matrix
            % Input: w - 3x1 vector
            % Output: C - 3x3 skew-symmetric matrix
            % Used to represent cross product operation: C*v = w × v

            C = [0, -w(3), w(2); ...
                w(3), 0, -w(1); ...
                -w(2), w(1), 0];
        end

        % done
        function [Rew, Rns] = earthradius(obj, lat)
            % Calculate Earth curvature radii at given latitude
            % Input: lat - latitude (radians)
            % Output:
            %   Rew - East-West curvature radius (meters)
            %   Rns - North-South curvature radius (meters)

            denom = abs(1.0-(obj.ECC2 * (sin(lat)^2)));
            Rew = obj.EARTH_RADIUS / sqrt(denom);
            Rns = obj.EARTH_RADIUS * (1.0 - obj.ECC2) / (denom * sqrt(denom));
        end

        % done
        function lla_dot = llarate(obj, V, varargin)
            % Calculate geodetic coordinate rates
            % Inputs:
            %   V - velocity vector [vn; ve; vd] (m/s)
            %   lla - position vector [lat; lon; alt] OR lat, alt separately
            % Output: lla_dot - geodetic coordinate rates [dlat/dt; dlon/dt; dalt/dt]

            if nargin == 3
                % input is complete lla vector
                lla = varargin{1};
            elseif nargin == 4
                % input is separate lat and alt
                lat = varargin{1};
                alt = varargin{2};
                lla = [lat; 0; alt]; % longitude set to 0 (doesn't affect lat/alt calculations)
            else
                error('invalid input args for llarate');
            end

            % calculate Earth curvature radii
            [Rew, Rns] = obj.earthradius(lla(1));

            % calculate geodetic coordinate rates
            lla_dot = [V(1) / (Rns + lla(3)); ... % latitude rate
                V(2) / ((Rew + lla(3)) * cos(lla(1))); ... % longitude rate
                -V(3)]; % altitude rate
        end

        % done
        function ecef = lla2ecef(obj, lla)
            % Convert geodetic coordinates (lat, lon, alt) to Earth-Centered Earth-Fixed (ECEF) coordinates
            % Input: lla - [latitude; longitude; altitude] (rad, rad, m)
            % Output: ecef - [x; y; z] (meters)

            [Rew, ~] = obj.earthradius(lla(1));
            ecef = [(Rew + lla(3)) * cos(lla(1)) * cos(lla(2)); ... % x coordinate
                (Rew + lla(3)) * cos(lla(1)) * sin(lla(2)); ... % y coordinate
                (Rew * (1.0 - obj.ECC2) + lla(3)) * sin(lla(1))]; % z coordinate
        end

        % done
        function ned = ecef2ned(obj, ecef, pos_ref)
            % Convert ECEF coordinate vector to NED coordinates (with reference position as origin)
            % Inputs:
            %   ecef - ECEF coordinate vector [x; y; z] (meters)
            %   pos_ref - reference point geodetic coordinates [lat; lon; alt] (rad, rad, m)
            % Output: ned - NED coordinates [north; east; down] (meters)

            ned = [-sin(pos_ref(1)) * cos(pos_ref(2)) * ecef(1) - sin(pos_ref(1)) * sin(pos_ref(2)) * ecef(2) + cos(pos_ref(1)) * ecef(3); ... % north component
                -sin(pos_ref(2)) * ecef(1) + cos(pos_ref(2)) * ecef(2); ... % east component
                -cos(pos_ref(1)) * cos(pos_ref(2)) * ecef(1) - cos(pos_ref(1)) * sin(pos_ref(2)) * ecef(2) - sin(pos_ref(1)) * ecef(3)]; % down component
        end

        % done
        function C_N2B = quat2dcm(obj, q)
            % Convert quaternion to direction cosine matrix
            % Input: q - quaternion [w; x; y; z]
            % Output: C_N2B - Navigation to Body frame rotation matrix

            C_N2B = [2 * (q(1)^2) - 1 + 2 * (q(2)^2), 2 * q(2) * q(3) + 2 * q(1) * q(4), 2 * q(2) * q(4) - 2 * q(1) * q(3); ...
                2 * q(2) * q(3) - 2 * q(1) * q(4), 2 * (q(1)^2) - 1 + 2 * (q(3)^2), 2 * q(3) * q(4) + 2 * q(1) * q(2); ...
                2 * q(2) * q(4) + 2 * q(1) * q(3), 2 * q(3) * q(4) - 2 * q(1) * q(2), 2 * (q(1)^2) - 1 + 2 * (q(4)^2)];
        end

        % done
        function r = qmult(obj, p, q)
            % Quaternion multiplication
            % Input: p, q - quaternions [w; x; y; z]
            % Output: r - product quaternion

            r = [p(1) * q(1) - (p(2) * q(2) + p(3) * q(3) + p(4) * q(4)); ... % w component
                p(1) * q(2) + q(1) * p(2) + p(3) * q(4) - p(4) * q(3); ... % x component
                p(1) * q(3) + q(1) * p(3) + p(4) * q(2) - p(2) * q(4); ... % y component
                p(1) * q(4) + q(1) * p(4) + p(2) * q(3) - p(3) * q(2)]; % z component
        end

        % done
        function dta = constrainAngle180(obj, dta)
            % Constrain angle to [-π, π] range
            % Input: dta - input angle (radians)
            % Output: dta - constrained angle (radians)

            if dta > pi
                dta = dta - 2 * pi;
            end
            if dta < -pi
                dta = dta + 2 * pi;
            end
        end

        % done
        function dta = constrainAngle360(obj, dta)
            % Constrain angle to [0, 2π] range
            % Input: dta - input angle (radians)
            % Output: dta - constrained angle (radians)

            dta = mod(dta, 2*pi);
            if dta < 0
                dta = dta + 2 * pi;
            end
        end

        % done
        function q = toQuaternion(obj, yaw, pitch, roll)
            % Calculate quaternion from Euler angles
            % Input: yaw, pitch, roll - yaw, pitch, roll angles (radians)
            % Output: q - quaternion [w; x; y; z]
            % Rotation order: ZYX (yaw -> pitch -> roll)
            
            % compute half-angle trigonometric functions
            cy = cos(yaw*0.5);
            sy = sin(yaw*0.5);
            cp = cos(pitch * 0.5);
            sp = sin(pitch * 0.5);
            cr = cos(roll*0.5);
            sr = sin(roll*0.5);

            % quaternion calculation
            q = [cr * cp * cy + sr * sp * sy; ... % w component
                cr * cp * sy - sr * sp * cy; ... % x component
                cr * sp * cy + sr * cp * sy; ... % y component
                sr * cp * cy - cr * sp * sy]; % z component
        end

        % done
        function [roll, pitch, yaw] = toEulerAngles(obj, quat)
            % Calculate Euler angles from quaternion
            % Input: quat - quaternion [w; x; y; z]
            % Output: roll, pitch, yaw - roll, pitch, yaw angles (radians)
            % Rotation order: ZYX (yaw -> pitch -> roll)
            
            % roll angle (x-axis rotation)
            sinr_cosp = 2 * (quat(1) * quat(2) + quat(3) * quat(4));
            cosr_cosp = 1 - 2 * (quat(2)^2 + quat(3)^2);
            roll = atan2(sinr_cosp, cosr_cosp);

            % pitch angle (y-axis rotation)
            sinp = 2 * (quat(1) * quat(3) - quat(2) * quat(4));
            if abs(sinp) >= 1
                % handle singularity
                pitch = sign(sinp) * pi / 2;
            else
                pitch = asin(sinp);
            end

            % yaw angle (z-axis rotation)
            siny_cosp = 2 * (quat(2) * quat(3) + quat(1) * quat(4));
            cosy_cosp = 1 - 2 * (quat(3)^2 + quat(4)^2);
            yaw = atan2(siny_cosp, cosy_cosp);
        end

        % ==================== Getter Methods ====================

        function result = initialized(obj)
            % Return whether EKF is initialized
            result = obj.initialized_;
        end

        function result = getPitch_rad(obj)
            % return pitch angle (rad)
            result = obj.theta;
        end

        function result = getRoll_rad(obj)
            % return roll angle (rad)
            result = obj.phi;
        end

        function result = getHeadingConstrainAngle180_rad(obj)
            % return heading angle constrained to [-π, π] range (rad)
            result = obj.constrainAngle180(obj.psi);
        end

        function result = getHeading_rad(obj)
            % return raw heading angle (rad)
            result = obj.psi;
        end

        function result = getLatitude_rad(obj)
            % return latitude (rad)
            result = obj.lat_ins;
        end

        function result = getLongitude_rad(obj)
            % return longitude (rad)
            result = obj.lon_ins;
        end

        function result = getAltitude_m(obj)
            % Return altitude (m)
            result = obj.alt_ins;
        end

        function result = getVelNorth_ms(obj)
            % return north velocity (m/s)
            result = obj.vn_ins;
        end

        function result = getVelEast_ms(obj)
            % return east velocity (m/s)
            result = obj.ve_ins;
        end

        function result = getVelDown_ms(obj)
            % return down velocity (m/s)
            result = obj.vd_ins;
        end

        function result = getGroundTrack_rad(obj)
            % return ground track angle (rad)
            result = atan2(obj.ve_ins, obj.vn_ins);
        end

        function result = getGyroBiasX_rads(obj)
            % return X-axis gyro bias estimate (rad/s)
            result = obj.gbx;
        end

        function result = getGyroBiasY_rads(obj)
            % return Y-axis gyro bias estimate (rad/s)
            result = obj.gby;
        end

        function result = getGyroBiasZ_rads(obj)
            % return Z-axis gyro bias estimate (rad/s)
            result = obj.gbz;
        end

        function result = getAccelBiasX_mss(obj)
            % return X-axis accelerometer bias estimate (m/s²)
            result = obj.abx;
        end

        function result = getAccelBiasY_mss(obj)
            % return Y-axis accelerometer bias estimate (m/s²)
            result = obj.aby;
        end

        function result = getAccelBiasZ_mss(obj)
            % return Z-axis accelerometer bias estimate (m/s²)
            result = obj.abz;
        end
    end
end
