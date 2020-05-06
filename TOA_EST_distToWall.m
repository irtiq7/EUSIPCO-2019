clc;clear all;
%%
%******************************NOTES***************************************
% - This script is used to test the performance of the algorithm
% plots of RMSE against distToWall

%%
initialization_step;
plot_enable = 1;

sampFreq=44100;
% sampFreq = 48000;

% downsampling
downSample = 44100;
downSample = sampFreq/downSample;

%SNR range
SNR=(-40:10:40)';

%Candidate delay to estimate the TimeDelay for each reflected sound before
%it reaches the microphone
candidateDelays=linspace(6,1000,4000);
candidateGains=linspace(0.00,0.95,200);

%Signal length
signalLength = 2000;

%calculate RIR using the below parameter
dimensions=[10 10 6]; %Dimension of soundlab is 5.4 x 6.38 x 4.05
reverbTime=(0.8)';
nsamples= [];
micType='omnidirectional';
%micType='bipolar';
soundSpeed=343;
rirOrder=-1;
distSourceToReceiv=0.1;
distToWall=(0.1:0.1:2)';
offset = 513;
% sourcePos=[distToWall,dimensions(2)/6,dimensions(3)/2];
% receivPos=[sourcePos(1:2),sourcePos(3)-distSourceToReceiv];
% RIRd=rir_generator(soundSpeed,sampFreq,receivPos,sourcePos,dimensions,...
% reverbTime,reverbLength,micType,0,3,[]); % reflection Order = 0 to simulate direct reflection
% 
% RIRs=rir_generator(soundSpeed,sampFreq,receivPos,sourcePos,dimensions,...
% reverbTime,reverbLength,micType,rirOrder,3,[pi,0]);

for zz=[-20 -10 0 10 20]
for ll = 1:1:50
    k = 1;
for ii=0.1:0.1:2
        sourcePos=[ii,dimensions(2)/2,dimensions(3)/2];
        receivPos=[sourcePos(1:2),sourcePos(3)-distSourceToReceiv];
%         [Room,source, receiver, options] = MCroom_init(sourcePos, receivPos, dimensions, true,sampFreq,reverbTime,soundSpeed);
%         RIRd = RunMCRoomSim(source,receiver,Room,options);
        [Room,source, receiver, options] = MCroom_init(sourcePos, receivPos, dimensions, false,sampFreq,reverbTime,soundSpeed);
        RIRs = RunMCRoomSim(source,receiver,Room,options);
        %         [signalClean,tx,sampFre] = GMSP_TimeDomain(0.1,1800,0.5);
        %         signalClean = (signalClean');
        signalClean=randn(signalLength,1);
        Fs = sampFreq;
        Ts = 1/Fs;
        dt = (0:length(signalClean)-1)/sampFreq;
        [droneSound, sampFreq] = audioread('allMotors_70.wav');
%         find mean of all microphones
        droneSound = mean(droneSound');
%        recording from one microphone 
%         droneNoise = droneNoise(:,1);
        droneSound = droneSound(0.4e6:0.5e6)';
        droneSound = droneSound(1:signalLength);
%         RIRd = RIRd(offset:end);
        RIRs = RIRs(offset:end);
        
        % resampling RIRs to 1/6 of Fs
        RIRs = resample(RIRs,1,downSample);
        
        fftLength=2000;%default 2^14
        w=linspace(0,2*pi,fftLength);


        % generate filtered signals
%         signalDirect=fftfilt(RIRd,signalClean);
        signalDroneSound = fftfilt(RIRs,droneSound);
%         signalDirect=signalDirect(1:end);
        signalClean=fftfilt(RIRs,signalClean);
        signalReceived=fftfilt(RIRs,signalClean);
        signalReceived = signalReceived(1:end);

        signalClean = signalClean(1:end);
        
        %calculate variance of the noise
%         var_noise = var(fftfilt(RIRs,signalClean))/(10^(SNR/10));

%         normalizing signal
%         signalClean_snr = signalReceived;
        
%         signalDroneSound = signalDroneSound/std(signalDroneSound)*sqrt(var_noise);

%         soundsc(signalReceived,Fs)
%         signalReceived = signalDroneSound + signalClean_snr;
        
        %Introduce AWGN
        signalReceived = awgn(signalReceived,zz, 'measured', 'dB');
%         soundsc(signalReceived,Fs)
        %Substract Direct path component from the received signal
%         signalReflections = signalReceived - signalDirect ;
        signalReflections = signalReceived;


        % compute ffts
        signalReflectionsFft  = fft(signalReflections,fftLength);
%         signalDirectFft =fft(signalDirect,fftLength);
        signalCleanFft=fft(signalClean,fftLength);
        signalReceivedFft=fft(signalReceived,fftLength);

        %truncate the signal to reduce the size of the signal
        signalReceivedFft=signalReceivedFft(1:signalLength);
        signalCleanFft=signalCleanFft(1:signalLength);
        signalReflectionsFft=signalReflectionsFft(1:signalLength);
        
        %compute the rir from the observed signals and input signals
        rirEstFft=signalReceivedFft./signalCleanFft;
        rirEst=ifft(rirEstFft);
        RMSE_against_SNR = zeros(length(init.SNR),length(init.iteration));
        % NLS with RELAX procedure for gain and tau estimation
        response = zeros(init.fftLength,init.R-1);
        response_sim = 0;

        delayEst = zeros(init.R-1,1);
        delayEst_control_vec = zeros(init.R-1,1);
        delayEst_control_vec1 = zeros(init.R-1,1);


        gainEst = zeros(init.R-1,1);
        gainEst_control_vec = zeros(init.R-1,1);
        gainEst_control_vec1 = zeros(init.R-1,1);

        control_vector = ones(init.R-1,1);
            % control_vector(1)=0;
        control_vector1 = ones(init.R-1,1);

        convergence_array = zeros(length(signalReceivedFft),1);
        delayEst_control_vec(1) =1;
        gainEst_control_vec(1) =1;
        delayEst_control_vec1(1) =1;
        gainEst_control_vec1(1) =1;
        for rr=1:1:init.R-1
            i=1;
        %     Step 1 : calculating first-order reflection Y2
        %     while 1

            response_sim = response*control_vector1;
        
        [gainEst(rr,1),delayEst(rr,1),response(:,rr),costFunctionDelay2]=...
            delayEstimationFIR_filter(signalReflectionsFft, signalCleanFft,response_sim,candidateDelays);

%         delayEst
%         gainEst2
            control_vector1 = circshift(control_vector1,1);
            delayEst_control_vec1 = circshift(delayEst_control_vec1,1);
            gainEst_control_vec1 = circshift(gainEst_control_vec1,1);      

        %   RELAX algorithm ----> nothing relaxed about it
            if rr>1
               while 1
                   for mm=rr:-1:1
                       control_vector = ones(init.R-1,1);
                       control_vector(mm)=0;
                       response_sim = response*control_vector;
                       delayEst_value = delayEst'*delayEst_control_vec;

                        [gainEst(rr,1),delayEst(rr,1),response(:,rr),costFunctionDelay2]=...
                         delayEstimationFIR_filter(signalReflectionsFft, signalCleanFft,response_sim,candidateDelays);

                   end
                   convergence_array(i) = norm(signalReceivedFft - response_sim.*signalCleanFft);
                   i=i+1;
                   if i>2
                       if ((convergence_array(i-1)-convergence_array(i-2)).^2<init.epsilon)
                           break
                       end
                   end
               end 
            end
        end

        disp('Est Distance to wall DelayEst2');
        est_dist= (delayEst(1)/sampFreq);


        % %calculate groundTruth using image source method
        [a,b,c,d,e]=true_TOA_est(dimensions, sourcePos, distSourceToReceiv,sampFreq);
        groundTruth = (b/sampFreq); %calculated using image source model
        dist_groundTruth(ll,k) = b*soundSpeed/Fs;
        
        aa(ll,k) = est_dist;
        k = k+1;
end
end

[RMSE_model_based_est] = calculate_rmse(aa,dist_groundTruth./soundSpeed);
% 
% %Plot the RMSE against SNR graph
if plot_enable==1
%       
    figure(3)
%     plot(SNR',log10(RMSE_peak_picking), '-o')
    hold on
    plot(distToWall',log10(RMSE_model_based_est),'-o')
    xlabel('distance to the wall(m)');
    ylabel('log_{10}(RMSE of TOA)(\tau)');
    legend('SNR = -20','SNR = -10','SNR = 0','SNR = 10','SNR = 20');
%     title({'Performance of the two method at a','dist = 1 and reverbTime =0.8'})
end
end

save(['RMSE_TOA_vs_Dist_SNR' num2str(zz) '_Fs_44_1k_simulation_50_MCRoom_10_10_6_no_AWGN.mat']);
% save('RMSE_TOA_vs_Absorption_SNR_0_dist_0_1_reverbTime_0_8_Fs_48k_simulation_50_MCRoom_6_6_2_4.fig');

