function [val_opt,fitness,Computer] = PSOD(ad,bd)

% Definirea intervalelor pentru fiecare parametru [min, max]

Ts = 1;

        param_ranges = [
    ad, (ad+bd)/2; % Td_11  ad, (ad + bd)/2
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
    
];
    disp(['Configuratia stabilita: Td11 = [', num2str(ad), ',', num2str((ad+bd)/2), ']']);


Td_11 = ad + ((ad+bd)/2-ad)* rand;
Td_12 = ad + (bd -ad)*rand; 
Td_21 = ad + (bd -ad)*rand;
Td_22 = ad + (bd -ad)*rand; 
Nd = 10;  
Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];


%%
% Extractăm limitele minime și maxime pentru particleswarm
lb = param_ranges(:, 1); % Limitele minime
ub = param_ranges(:, 2); % Limitele maxime


% Numarul de parametri de optimizat
nVars = 4;
% Setarea optiunilor pentru particleswarm

% Opțiuni pentru algoritmul PSO

%%
options = optimoptions('particleswarm', ...
    'SwarmSize', 150, ...  
    'MaxIterations', 1000, ...  
    'InertiaRange', [0.4 0.9], ...  
    'SelfAdjustmentWeight', 1.19, ...  
    'SocialAdjustmentWeight', 1.19, ...  
    'Display', 'iter'); 

%%

[x, fval] = particleswarm(@evaluateD, nVars, lb, ub, options);

% Extragem parametrii optimi pentru RST1 și RST2


val_opt = x;
fitness = fval;
F= 100/(1+0.65*fval);
end


