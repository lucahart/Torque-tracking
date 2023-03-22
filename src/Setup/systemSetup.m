function sys = systemSetup(sys_pre)
    % Setup of all physical system parameters. 
    % Returns a struct sys.
    
   
    % *********************************************************************
    %% Make changes to this section (System parameters)
    % *********************************************************************
    
    % Machine parameters is SI unit system
    V = 3300; % voltage [V]
    I = 356; % current [A]
    P = 1.587e6; % active power [W]
    S = 2.035e6; % aparent power [VA]
    f_base = 50; % base frequency of stator electric field [rps]
    p = 2; % number of poles on rotor
    f_m_si = 1494/60; % frequency of mechanical rotor [rps]
    
    % Machine parameters in pu system
    R_s = 0.0108;
    R_r = 0.0091;
    X_m = 2.349;
    X_ls = 0.1493*2;
    X_lr = 0.1104*2;
    v_dc = 1.930;
    std = 5e-4; % noise standard deviation
%     
%     V = 6000;
%     I = 98.9;
%     P = 850e3;
%     S = 1.028e6;
%     f_base = 50;
%     p = 2;
%     f_m_si = 1494/60;
%     R_s = .0057;
%     R_r = .0045;
%     X_ls = .0894;
%     X_lr = .0930;
%     X_m = 2.492;
%     v_dc = 2e3;
    
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
                        % "MPCofHighPowerConvIndDrives"
    
    
end