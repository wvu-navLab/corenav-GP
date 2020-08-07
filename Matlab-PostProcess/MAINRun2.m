% Authors: Cagri Kilic and Jason Gross
% Date: September 2018
%%  -----------------------------------------------------------------------
% clc;
clear;
load data/testrun9_2.mat % LshapeData.mat %ForwardDrive.mat
% gpsHeading
yaw=151.5;
yawRad=yaw*pi/180
%s1320,s5.313% s5:313%s3s4:134%-s1t7-319 %shrt1:143
%t1:194.5,t8-t6-t4-t2:312.5 t9:151.5,t3:158.5 t5=167.5 t7= t10:47 t11:238,
%t8:306.5,t4:312,5, t6:315, longrun:283
%% initialize variables
MainRunInit2 % LshapeInit.mat %ForwardDriveInit.mat
psiEst=insAtt(3,1);
%% odometry calculations
odom

for i=2:L
    dtIMU=tTimu(i)-tTimu(i-1); % calculates imu delta time from rostomat output tTimu
    TimeIMU(i)=dtIMU+TimeIMU(i-1); % creates imu time from delta time starting from zero
    
    omega_b_ib=[Gx(i-1),Gy(i-1),Gz(i-1)]-bg(:,i-1)';
    alpha_ib_b =omega_b_ib*dtIMU;
    mag_alpha = sqrt(alpha_ib_b' * alpha_ib_b);
    Alpha_ib_b = Skew_symmetric(alpha_ib_b); 
    
    f_ib_b = [Ax(i-1),Ay(i-1),Az(i-1)]-ba(:,i-1)'; %m/s^2 IMU acceleration minus estimated acce bias
    v_ib_b= f_ib_b* dtIMU; %m/s acceleraion times delta IMU = velocity
    
    psiEst_minus=insAtt(3,i-1);
    
    %% Attitude Update
    [insAttPlus, Cb2nPlus,Cb2nMinus,Omega_n_en,Omega_n_ie,R_N,R_E,omega_n_in,omega_n_ie] = AttUpdate(insAtt(:,i-1),omega_ie,insLLH(:,i-1),omega_b_ib,ecc,Ro,insVel(:,i-1),dtIMU);
    insAtt(:,i)=insAttPlus;
    %% Velocity Update
    [insVelPlus] = VelUpdate(Cb2nMinus, Cb2nPlus, v_ib_b,insVel(:,i-1),insLLH(:,i-1),omega_ie,Ro,ecc,dtIMU);
    insVel(:,i)=insVelPlus;
    %% Position Update
    [insLLHPlus,R_EPlus,r_eb_e] = PosUpdate(insLLH(:,i-1),insVel(:,i),insVel(:,i-1),Ro,ecc,dtIMU);
    insLLH(:,i)=insLLHPlus;
    insXYZ(:,i)=r_eb_e;
    %% Error State Model--eq 14.63
    [STM] = insErrorStateModel_LNF(R_EPlus,R_N,insLLH(:,i),insVel(:,i),dtIMU,Cb2nPlus,omega_ie,omega_n_in,f_ib_b);
    %% Propagation of the Error
    x_err=STM*x_err;
    %% Q matrix --
    F21= -skewsymm(Cb2nPlus*(f_ib_b'));
    Q=getQins(F21,Cb2nPlus,insLLH(:,i),R_N,R_E,dtIMU);
    %% P matrix
    P = STM*P*STM' + Q;
    %% positiveDefiniteCheck of P and Q matrices
    positiveDefiniteCheck
    %% store fixed values before GP
    STM_fixed=STM;
    Q_fixed=Q;
    P_fixed=P;
    P_pred=P;
    %% Integrate IMU specific measurements for odometry update
    if odomUpdate()
        insIntegrationforOdomUpdate
    end
    %% NonHolonomic motion constraints as a Pseudo-Update
    if nonHolo()
        [insAtt(:,i),insVel(:,i),insLLH(:,i),x_err,P]= nonHolonomic(ang_z(kk),insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_n_ie,omega_b_ib,A);
    end
    
    if abs(rearVel(kk))<0.005 && sign(frontRightVel(kk))*sign(frontLeftVel(kk))~=1% triggers zupt
        zeroUptCount=zeroUptCount+1;
        if zeroUpdate == true % && kk<50
            [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,postFitZero] = zeroUpd(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_b_ib);
        end
        zCtr(i)=cttr3+offsetCtr;
        LLHcorrected(:,cttr3)=insLLH(:,i);
        cttr3=cttr3+1;
    else
        zCtr(i)=0;
        offsetCtr=offsetCtr+1;
    end 

    xState{1,i}=x_err;
    PStore{1,i}=P;
    STMStore{1,i}=STM+eye(15).*x_err;
    
    %% Check for wheel odometry update availability
    if kk<min(min(length(heading),length(lin_x))) 
        if tTimu(i)>=tTodom(kk) % Odometry update is available
            odometryUpdate
        end
    end
    
    ba(1:3,i)=ba(1:3,i-1)+x_err(10:12);
    bg(1:3,i)=bg(1:3,i-1)+x_err(13:15);
    
    x_err(10:12) = [0;0;0];
    x_err(13:15) = [0;0;0];
    
    sig1(i)=3*sqrt(abs(P(1,1))); % 3 sigma values of att_x -roll
    sig2(i)=3*sqrt(abs(P(2,2))); % 3 sigma values of att_y -pitch
    sig3(i)=3*sqrt(abs(P(3,3))); % 3 sigma values of att_z -yaw
    
    sig4(i)=3*sqrt(abs(P(4,4))); % 3 sigma values of vel_x -forward
    sig5(i)=3*sqrt(abs(P(5,5))); % 3 sigma values of vel_y -left
    sig6(i)=3*sqrt(abs(P(6,6))); % 3 sigma values of vel_z -down
    
    sig7(i)=3*sqrt(abs(P(7,7))); % 3 sigma values of pos_x -latitude
    sig8(i)=3*sqrt(abs(P(8,8))); % 3 sigma values of pos_y -longitude
    sig9(i)=3*sqrt(abs(P(9,9))); % 3 sigma values of pos_z -height
    
    if gpsResults()
        gpsLonger(:,i)=[llhGPS(1,kk);llhGPS(2,kk);llhGPS(end,kk)];
    end
    %     x_State(:,i)=[insAtt(:,i);insVel(:,i);insLLH(:,i);ba(:,i);bg(:,i)];
    %     v_in(i)=[1,0,0]*Cn2b_corr*(insVel(:,i));
    %     insAttCorr(:,i)=dcm2eulr((eye(3)-skewsymm(x_err(1:3)))*Cn2b_corr');
    %     insVelCorr(:,i)=insVel(:,i)-x_err(4:6);
    %     insLLHCorr(:,i)=insLLH(:,i)-x_err(7:9);
    Cn2b_corr= eulr2dcm(insAtt(:,i));
    v_in(i)=[1,0,0]*Cn2b_corr*(insVel(:,i));
    insAttCorr(:,i)=insAtt(:,i);
    insVelCorr(:,i)=insVel(:,i);
    insLLHCorr(:,i)=insLLH(:,i);
end
gpsLonger(:,1)=gpsLonger(:,2);

figureGeneration
figure;plot(tTodom-tTodom(1),bearing);
figure;plot(tTimu-tTimu(1),YawErr2);
