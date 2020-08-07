            bb(counter)=i; % counts for the number of `i` when the loop goes into odometry updates
            dt_odom=tTodom(kk)-tTodom(kk-1);
            %% Odometry Update
            if odomUpdate()
                odomUptCount=odomUptCount+1;
                P_old=P;
                insAtt_old= insAtt(:,i);
                insVel_old= insVel(:,i);
                insLLH_old= insLLH(:,i);
                x_err_old=x_err;
                
                [insAtt_new,insVel(:,i),insLLH(:,i),x_err,P,postFitOdom]= odomUpt(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,...
                    psiEst_minus,psiEst,dt_odom,rearVel(kk),headRate(kk),s_or,...
                    H11,H12,H21,H31,H32,H41,H42,z11,z21,z31,z41,T_r,...
                    sigma_or_L,sigma_or_R,sigma_cmc,s_delta_or);

                insAtt(:,i)=insAtt_new;
                psiEst=insAtt(3,i);

                
                %% Slip Calculation
                slipCalculation
                %% GP process
                gpProcess
                %% Store detected slipped locations
                if abs(slipBL(1,odomUptCount))>0.2 || abs(slipBR(1,odomUptCount))>0.2 || abs(slipFL(1,odomUptCount))>0.2 || abs(slipFR(1,odomUptCount))>0.2
                    LLHcorrected1(:,cttr0)=insLLH(:,i);
                    cttr0=cttr0+1;
                end
            end % odom update
            %% Destroy H and z values for the next values
            H11=zeros(1,3);
            H12=zeros(1,3);
            H21=zeros(1,3);
            H31=zeros(1,3);
            H32=zeros(1,3);
            H24=zeros(1,3);
            H41=zeros(1,3);
            H42=zeros(1,3);
            z11=0;
            z21=0;
            z31=0;
            z41=0;
            counter=counter+1;
            kk=kk+1;