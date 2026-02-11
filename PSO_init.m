function [val_opt,fitness,Computer] = PSO_init(ap,bp,ai,bi,ad,bd);

% Definirea intervalelor pentru fiecare parametru [min, max]

Ts = 1;
Computer = input('Introduceti nr calculatorului (1-8): ');
disp(['Computer = ', num2str(Computer)]);

switch Computer
    case 1
        param_ranges = [
    ap, (ap+bp)/2;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    ai, (ai+bi)/2;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    ad, (ad+bd)/2; % Td_11  ad, (ad + bd)/2
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
    
];
    disp(['Configuratia stabilita: Kp11 = [', num2str(ap), ',', num2str((ap + bp) / 2), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str(ai), ',', num2str((ai + bi) / 2), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str(ad), ',', num2str((ad + bd) / 2), ']']);

Ti_11 = ai + ((ai+bi)/2-ai)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

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

Kp_11 = ap + ((ap+bp)/2 -ap)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];

        
    case 2
        param_ranges = [
    ap, (ap+bp)/2;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    ai, (ai+bi)/2;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    (ad+bd)/2, bd; % Td_11  
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22

];

    disp(['Configuratia stabilita: Kp11 = [', num2str(ap), ',', num2str((ap + bp) / 2), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str(ai), ',', num2str((ai + bi) / 2), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str((ad + bd) / 2), ',', num2str(bd), ']']);

Ti_11 = ai + ((ai+bi)/2-ai)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

Td_11 = (ad+bd)/2 + (bd-(ad+bd)/2)* rand;
Td_12 = ad + (bd -ad)*rand; 
Td_21 = ad + (bd -ad)*rand;
Td_22 = ad + (bd -ad)*rand; 
Nd = 10;  
Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

Kp_11 = ap + ((ap+bp)/2 -ap)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];
    case 3
        param_ranges = [
    ap, (ap+bp)/2;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    (ai+bi)/2, bi;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    ad, (ad+bd)/2; % Td_11  ad, (ad + bd)/2
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
    
];
    disp(['Configuratia stabilita: Kp11 = [', num2str(ap), ',', num2str((ap + bp) / 2), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str((ai+bi)/2), ',', num2str(bi), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str(ad), ',', num2str((ad+bd)/2), ']']);

       alegere = input('Pentru initializare custom apasati 1, pentru cea normala apasati 2 :');
    
       if alegere == 1
disp('Alegere = Custom');
data = load('PID_cls_Eln_3_78.11 1.mat');
Nd = 10;
Kp_11 = data.Kp_11;
Kp_12 = data.Kp_12;
Kp_21 = data.Kp_21;
Kp_22 = data.Kp_22;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];

Ti_11 = data.Ti_11;
Ti_12 = data.Ti_12;
Ti_21 = data.Ti_21;
Ti_22 = data.Ti_22;

Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

Td_11 = data.Td_11;
Td_12 = data.Td_12;
Td_21 = data.Td_21;
Td_22 = data.Td_22;

Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

initialSwarm = [Kp_11 Kp_12 Kp_21 Kp_22 Ti_11 Ti_12 Ti_21 Ti_22 Td_11 Td_12 Td_21 Td_22];


       else
  disp('Alegere = Normala');
Ti_11 = (ai+bi)/2 + (bi-(ai+bi)/2)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

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

Kp_11 = ap + ((ap+bp)/2 -ap)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];

       end
    case 4
        param_ranges = [
    ap, (ap+bp)/2;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    (ai+bi)/2, bi;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    (ad+bd)/2, bd; % Td_11  ad, (ad + bd)/2
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
];
    disp(['Configuratia stabilita: Kp11 = [', num2str(ap), ',', num2str((ap + bp) / 2), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str((ai+bi)/2), ',', num2str(bi), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str((ad + bd) / 2), ',', num2str(bd), ']']);

Ti_11 = (ai+bi)/2 + (bi-(ai+bi)/2)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

Td_11 = (ad+bd)/2 + (bd-(ad+bd)/2)* rand;
Td_12 = ad + (bd -ad)*rand; 
Td_21 = ad + (bd -ad)*rand;
Td_22 = ad + (bd -ad)*rand; 
Nd = 10;  
Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

Kp_11 = ap + ((ap+bp)/2 -ap)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];
    case 5
        param_ranges = [
    (ap+bp)/2, bp;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    ai, (ai+bi)/2;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    ad, (ad+bd)/2; % Td_11  
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
];
    disp(['Configuratia stabilita: Kp11 = [', num2str((ap+bp)/2), ',', num2str(bp), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str(ai), ',', num2str((ai + bi) / 2), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str(ad), ',', num2str((ad + bd) / 2), ']']);

Ti_11 = ai + ((ai+bi)/2-ai)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

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

Kp_11 = (ap+bp)/2 + (bp-(ap+bp)/2)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];
    case 6
        param_ranges = [
   (ap+bp)/2, bp;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    ai, (ai+bi)/2;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    (ad+bd)/2, bd; % Td_11  
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
];
    disp(['Configuratia stabilita: Kp11 = [', num2str((ap+bp)/2), ',', num2str(bp), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str(ai), ',', num2str((ai + bi) / 2), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str((ad+bd)/2), ',', num2str(bd), ']']);

Ti_11 = ai + ((ai+bi)/2-ai)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

Td_11 = (ad+bd)/2 + (bd-(ad+bd)/2)* rand;
Td_12 = ad + (bd -ad)*rand; 
Td_21 = ad + (bd -ad)*rand;
Td_22 = ad + (bd -ad)*rand; 
Nd = 10;  
Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

Kp_11 = (ap+bp)/2 + (bp-(ap+bp)/2)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];
    case 7
        param_ranges = [
    (ap+bp)/2, bp;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    (ai+bi)/2, bi;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    ad, (ad+bd)/2; % Td_11  
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
];

    disp(['Configuratia stabilita: Kp11 = [', num2str((ap+bp)/2), ',', num2str(bp), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str((ai+bi)/2), ',', num2str(bi), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str(ad), ',', num2str((ad + bd) / 2), ']']);
    alegere = input('Pentru initializare custom apasati 1, pentru cea normala apasati 2 :');

    if alegere == 1


    disp('Alegere = Custom');
data = load('PID_cls_Eln_7_79.88 1.mat');
Nd = 10;

Kp_11 = data.Kp_11;
Kp_12 = data.Kp_12;
Kp_21 = data.Kp_21;
Kp_22 = data.Kp_22;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];

Ti_11 = data.Ti_11;
Ti_12 = data.Ti_12;
Ti_21 = data.Ti_21;
Ti_22 = data.Ti_22;

Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

Td_11 = data.Td_11;
Td_12 = data.Td_12;
Td_21 = data.Td_21;
Td_22 = data.Td_22;

Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

initialSwarm = [Kp_11 Kp_12 Kp_21 Kp_22 Ti_11 Ti_12 Ti_21 Ti_22 Td_11 Td_12 Td_21 Td_22];

    else
        disp('Alegere = Normala');

Ti_11 = (ai+bi)/2 + (bi-(ai+bi)/2)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

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

Kp_11 = (ap+bp)/2 + (bp-(ap+bp)/2)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];
    
end

    case 8
        param_ranges = [
    (ap+bp)/2, bp;  % Kp_11
    ap, bp;  % Kp_12
    ap, bp;  % Kp_21
    ap, bp;  % Kp_22
    (ai+bi)/2, bi;  % Ti_11
    ai, bi;  % Ti_12
    ai, bi;  % Ti_21
    ai, bi;  % Ti_22
    (ad+bd)/2, bd; % Td_11  
    ad, bd; % Td_12
    ad, bd; % Td_21
    ad, bd  % Td_22
];
    disp(['Configuratia stabilita: Kp11 = [', num2str((ap+bp)/2), ',', num2str(bp), ']']);
    disp(['Configuratia stabilita: Ti11 = [', num2str((ai+bi)/2), ',', num2str(bi), ']']);
    disp(['Configuratia stabilita: Td11 = [', num2str((ad+bd)/2), ',', num2str(bd), ']']);

Ti_11 = (ai+bi)/2 + (bi-(ai+bi)/2)* rand;
Ti_12 = ai + (bi -ai)*rand;
Ti_21 = ai + (bi -ai)*rand;
Ti_22 = ai + (bi -ai)*rand;
Gi_11 = tf([Ts, Ts], [2*Ti_11, -2*Ti_11], Ts);
Gi_12 = tf([Ts, Ts], [2*Ti_12, -2*Ti_12], Ts);
Gi_21 = tf([Ts, Ts], [2*Ti_21, -2*Ti_21], Ts);
Gi_22 = tf([Ts, Ts], [2*Ti_22, -2*Ti_22], Ts);
Ti_z =[Gi_11,Gi_12;Gi_21,Gi_22];

Td_11 = (ad+bd)/2 + (bd-(ad+bd)/2)* rand;
Td_12 = ad + (bd -ad)*rand; 
Td_21 = ad + (bd -ad)*rand;
Td_22 = ad + (bd -ad)*rand; 
Nd = 10;  
Gd_11 = tf([2*Td_11*Nd, -2*Td_11*Nd], [(Nd*Ts + 2*Td_11), (Nd*Ts - 2*Td_11)], Ts);
Gd_12 = tf([2*Td_12*Nd, -2*Td_12*Nd], [(Nd*Ts + 2*Td_12), (Nd*Ts - 2*Td_12)], Ts);
Gd_21 = tf([2*Td_21*Nd, -2*Td_21*Nd], [(Nd*Ts + 2*Td_21), (Nd*Ts - 2*Td_21)], Ts);
Gd_22 = tf([2*Td_22*Nd, -2*Td_22*Nd], [(Nd*Ts + 2*Td_22), (Nd*Ts - 2*Td_22)], Ts);

Td_z = [Gd_11,Gd_12; Gd_21,Gd_22];

Kp_11 = (ap+bp)/2 + (bp-(ap+bp)/2)*rand;
Kp_12 = ap + (bp-ap)*rand;
Kp_21 = ap + (bp-ap)*rand;
Kp_22 = ap + (bp-ap)*rand;

Kp_z = [Kp_11 Kp_12; Kp_21 Kp_22];
end

%%
% Extractăm limitele minime și maxime pentru particleswarm
lb = param_ranges(:, 1); % Limitele minime
ub = param_ranges(:, 2); % Limitele maxime


% Numarul de parametri de optimizat
nVars = 12;
% Setarea optiunilor pentru particleswarm

% Opțiuni pentru algoritmul PSO

%%
if alegere == 1
options = optimoptions('particleswarm', ...
    'InitialSwarmMatrix', initialSwarm,...
    'SwarmSize', 150, ...  
    'MaxIterations', 1000, ...  
    'InertiaRange', [0.4 0.9], ...  
    'SelfAdjustmentWeight', 1.19, ...  
    'SocialAdjustmentWeight', 1.19, ...  
    'Display', 'iter'); 
else
    options = optimoptions('particleswarm', ...
    'SwarmSize', 150, ...  
    'MaxIterations', 1000, ...  
    'InertiaRange', [0.4 0.9], ...  
    'SelfAdjustmentWeight', 1.19, ...  
    'SocialAdjustmentWeight', 1.19, ...  
    'Display', 'iter'); 
end
%%

[x, fval] = particleswarm(@evaluatePID, nVars, lb, ub, options);

% Extragem parametrii optimi pentru RST1 și RST2


val_opt = x;
fitness = fval;
F= 100/(1+0.65*fval);
end


