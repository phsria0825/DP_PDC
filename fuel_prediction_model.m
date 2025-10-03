function FC = fuel_prediction_model(v, a, theta)
%FUEL_PREDICTION_MODEL Estimate instantaneous fuel consumption rate.
%   FC = fuel_prediction_model(v, a, theta) computes the instantaneous fuel
%   consumption rate for the provided velocity (m/s), acceleration (m/s^2)
%   and road grade (rad) using an experimentally tuned polynomial model.
%
%   The implementation follows the model described in the project README and
%   supports scalar or vectorised inputs. All operations are performed using
%   element-wise arithmetic to remain numerically stable when evaluated in
%   MATLAB Function blocks inside Simulink.
%
%   See also: VELOCITYOPTIMIZ_DP.

    arguments
        v {mustBeReal}
        a {mustBeReal, mustBeSizeCompatibleWith(v)}
        theta {mustBeReal, mustBeSizeCompatibleWith(v)}
    end

    %% Vehicle parameters
    rho = 1.2256;       % Air density (kg/m^3)
    Cd = 0.615;         % Aerodynamic drag coefficient
    Ch = 1 - 0.085 * 0; % Altitude correction coefficient (placeholder)
    Af = 10.2;          % Frontal area (m^2)
    Cr = 1.25;          % Rolling resistance coefficient
    c1 = 0.0328;        % Rolling resistance speed coefficient 1
    c2 = 4.575;         % Rolling resistance speed coefficient 2
    mass = 23000;       % Vehicle mass (kg)
    eta_d = 0.85;       % Drivetrain efficiency

    %% 1. Driving resistances
    R_drag = (rho/25.92) * Cd * Ch * Af .* (v.^2); % Aerodynamic drag
    R_roll = 9.81 * (Cr/1000) .* (c1 .* v + c2);   % Rolling resistance
    R_clm  = 9.81 * mass .* sin(theta);            % Climbing resistance
    R = R_drag + R_roll + R_clm;                   % Total resistance

    %% 2. Required power
    P = ((R + 1.04 * mass .* a) ./ (3600 * eta_d)) .* v;

    %% 3. Power-to-fuel conversion (quadratic model)
    a0 = 0.7607578263;
    a1 = 0.0336310284;
    a2 = 0.0000630368;

    FC_raw = a0 + a1 .* P + a2 .* (P.^2);

    %% Final scaling
    FC = FC_raw ./ 850;
end

function mustBeSizeCompatibleWith(x, y)
%MUSTBESIZECOMPATIBLEWITH Ensure inputs can be broadcast together.
    if ~isscalar(x) && ~isscalar(y) && ~isequal(size(x), size(y))
        error("fuel_prediction_model:SizeMismatch", ...
              "Inputs must be scalar or match in size.");
    end
end
