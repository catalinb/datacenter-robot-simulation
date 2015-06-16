classdef Robot < Navigation
    properties
        % real position
        currentPos;
        radius;

        % where the robot thinks it is
        estimatedPos;
        particles;
        particleScore;

        deviation;
        lcov; % covariance for the likelyhood
        landmarks;
    end

    methods
        function robot = Robot(radius, landmarks, posX, posY, varargin)
            robot = robot@Navigation(varargin{:});

            robot.deviation = 0.001;
            robot.lcov = diag([0.001 0.001]);

            robot.radius = radius;
            robot.landmarks = landmarks;
            robot.currentPos = [posX, posY];
            robot.estimatedPos = [posX, posY];

            robot.particles = randn(200, 2) * 0.5 + repmat(robot.estimatedPos, [200,1]);

            robot.particleScore = 1 / 200 * ones(200, 1);
            
        end

        function newPos = stepTowards(robot, newPos, maxStepSize)
            delta = newPos - robot.estimatedPos;
            maxDistance = min(abs(delta), [maxStepSize, maxStepSize]);
            delta = maxDistance .* sign(delta);
            newPos = robot.currentPos + delta;

            % add noise
            odo = delta + randn(1,2) * robot.deviation;

            % predict movement for all particles
            robot.predict(odo);

            % check sensors and assign weights
            robot.observe();

            % select particles
            robot.select();

            robot.estimatedPos = mean(robot.particles);
            position_error = robot.currentPos - robot.estimatedPos
        end

        function predict(robot, odo)
            part = robot.particles;
            robot.particles = part + repmat(odo, [size(part, 1), 1]) + randn(size(part, 1), 2) * robot.deviation * 10;
        end

        function observe(robot)
            z = robot.sensorBearing(robot.currentPos)
            z = z(1,:);
            for p = 1:robot.particles
                % expected measurement
                z_pred = robot.sensorBearing(robot.particles(p, :));
                z_pred = z_pred(1,:);

                err = zeros(2, 1);
                err(1) = z(1) - z_pred(1);
                err(2) = angdiff(z(2), z_pred(2));

                %weight = exp(-0.5 * err(1) * 0.1) + 0.05;
                weight = exp(-0.5*err'*inv(robot.lcov)*err) + 0.005;
                robot.particleScore(p) = weight;
            end
        end

        function select(r)
            CDF = cumsum(r.particleScore) / sum(r.particleScore);
            len = size(r.particles, 1);
            selected = rand(len, 1);
            nextGen = interp1(CDF, 1:len, selected, 'nearest', 'extrap');
            r.particles = r.particles(nextGen, :);
        end

        function z = sensorBearing(robot, pos)
            pos = repmat(pos, [size(robot.landmarks, 1), 1]);
            d = robot.landmarks - pos;
            z = zeros(size(robot.landmarks, 1), 2);
            z(:,1) = colnorm([d(:,1), d(:, 2)]')';
            %z(:,1) = sqrt(d(:,1)^2 + d(:,2)^2);
            z(:,2) = atan2(d(:,2), d(:,1));

            % todo add noise for the angle
            z(:,1) = z(:,1) + randn(size(robot.landmarks, 1), 1) * robot.deviation;
        end

        function s = char(prm)
            s = char@Navigation(prm);

            s = char(s, sprintf('  graph size: %d', prm.npoints));
            s = char(s, sprintf('  dist thresh: %f', prm.distthresh));
            s = char(s, char(prm.graph) );
        end

        function n = next(prm, p)
            n = []
        end

        function plan(prm)
        end


    end
end
