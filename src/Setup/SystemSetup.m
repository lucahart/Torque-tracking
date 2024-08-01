function sys = SystemSetup(sys_pre)
    % Setup of the physical system.
    % Input:
    %   sys_pre (optional): Pre-set system parameters. Everything that is set in
    %     this struct will overwrite the default parameters.
    % Output:
    %   sys: Struct containing all information about the physical system.
    %
    % Parameters: Find detailled explanations in the book "Model Predictive
    %   Control of High Power Converters and Industrial Drives" by Tobias
    %   Geyer.
    
   
    % *********************************************************************
    %% Default system parameters (Make changes to this section)
    % *********************************************************************
    
%     % Machine parameters is SI unit system
%     V = 3100; %V = 400; % voltage [V]
%     I = 1477.3; %I = 4.4; % current [A]
%     S = 7.765e6; %S = 3.048e3; % aparent power [VA]
%     P = 6.6e6; %P = 2.4079e3; % active power [W]
%     f_base = 65.6; %f_base = 50; % base frequency of stator electric field [rps]
%     p = 1; % number of poles on rotor
%     f_m_si = 979.1/60; %f_m_si = 2875/60; % frequency of mechanical rotor [rps]

    % Machine parameters is SI unit system
    V = 3300; %V = 400; % voltage [V]
    I = 356; %I = 4.4; % current [A]
    S = 2e6; %S = 3.048e3; % aparent power [VA]
    P = 1.587e6; %P = 2.4079e3; % active power [W]
    f_base = 50; %f_base = 50; % base frequency of stator electric field [rps]
    p = 1; % number of poles on rotor
    f_m_si = 596/60; %f_m_si = 2875/60; % frequency of mechanical rotor [rps]

%     % Machine parameters in pu system
%     R_s = .0048; %0.0514;
%     R_r = .0050; %0.0457;
%     X_ls = .1469; %0.0591;
%     X_lr = .0974; %0.0705;
%     X_m = 2.8468; %2.3625;
%     v_dc = 4294/V; %1.9902; % dc-voltage [pu]
%     std = 5e-4; % noise standard deviation

    % Machine parameters in pu system
    R_s = .0108; %0.0514;
    R_r = .0091; %0.0457;
    X_ls = .1493; %0.0591;
    X_lr = .1104; %0.0705;
    X_m = 2.3489; %2.3625;
    v_dc = 1.5937; %1.9902; % dc-voltage [pu]
    std = 5e-4; % noise standard deviation

    X_s = X_m + X_ls;
    X_r = X_m + X_lr;
    D = X_s*X_r - X_m^2;
    w_base = 2*pi*f_base;
    omega_m_si = 2*pi*f_m_si; % mechanical angular speed of rotor in si
    omega_r_si = p*omega_m_si; % electric angular speed of rotor in si
    omega_r = omega_r_si/w_base; % electric angular speed of rotor in pu
    f_r = p*f_m_si/f_base; % electric frequency of rotor
    
    % Parameters needed outside of setup
    sys.omega_r = omega_r;
    sys.f_r = f_r;
    sys.pf = P/S;
    sys.D = D;
    sys.X_m = X_m;
    sys.X_s = X_s;
    sys.X_r = X_r;
    sys.w_base = w_base;
    sys.f_base = f_base;
    sys.std = std;
    
    
    % *********************************************************************
    %% Do NOT make any changes to this section (System dynamics)
    % *********************************************************************
    
    % Clarke transform and its inverse
    sys.K = 2/3*[1 -0.5 -0.5; 0 sqrt(3)/2 -sqrt(3)/2];
    sys.K_inv = [1 0; -0.5 sqrt(3)/2; -0.5 -sqrt(3)/2];
        
    % Use pre-specified simulation parameters if given
    % Overwrites the default setup of the section above
    if nargin >= 1
        fn = fieldnames(sys_pre);
        for i = 1:numel(fn)
            sys.(fn{i}) = sys_pre.(fn{i});
        end
    end
    
    % Continuous time dynamics
    sys.F_1 = -R_s*X_r/D*eye(2);
    sys.F_2 = omega_r*[0 -1; 1 0] - R_r*X_s/D*eye(2);
    sys.G_1 = R_s*X_m/D*eye(2);
    sys.G_2 = v_dc/2*sys.K;
    sys.G_3 = R_r*X_m/D*eye(2);
    sys.G_4 = zeros(2,3); % Independent of currents, see (8) in Nemeth et al.
                        % "STATE SPACE MODELING THEORY OF INDUCTION 
                        % MACHINES" or in Section 4.3.3 of Geyer, 
                        % "MPC of High Power Converters and Industrial Drives"
    
    
end