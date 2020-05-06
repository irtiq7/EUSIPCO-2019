clear;clc;
%  _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
% |This script is mainly used to extract TOAs using model-based approach    |
% |and RELAX procedure. Iterating over MONTE CARLO SIMULATION and           |
% |performance measure over SNR and distances are found in this script      |
% |main.m consist of barebone code with RELAX method without iteration func |
% |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _|

% Initialize all the parameters including room dim, candidateDelay,
% Absorption Rate etc
initialization_step;

for nn=init.SNR
    
    % Generate RIRs using parameters defined in initialization step
    RIRs = generate_RIR(init.distToWall,init.dimensions,init.distSourceToReceiv,init.sampFreq,init.absorptionRate,init.soundSpeed,init.offset);
    % filter the clean signal with RIR generated earlier
    signalReceived=fftfilt(RIRs,init.signalClean);

    % background noise 
    % 1 - drone Noise
    % 2 - AWGN
    signalReceived = Add_backgroundNoise(1,init.signalClean,signalReceived,init.signalLength,RIRs, nn);

    % Computer FFT
    signalCleanFft=fft(init.signalClean,init.fftLength);
    signalReceivedFft=fft(signalReceived,init.fftLength);

    %compute the rir from the observed signals and input signals
    rirEstFft=signalReceivedFft./signalCleanFft;
    rirEst=ifft(rirEstFft);

    for bb=1:1:init.iteration
            k=1;
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

            tic
            [delayEst(rr,1),costFunction3] = delayEstimation(signalReceivedFft-signalCleanFft.*response_sim,signalCleanFft,init.candidateDelays);
            toc
            tic
            delayEst_value = delayEst'*delayEst_control_vec1;
            [gainEst(rr,1)] = gainEstimation(signalReceivedFft,signalCleanFft,delayEst_value, response_sim);
            toc
            tic
            gainEst_value=gainEst'*gainEst_control_vec1;
            [response(:,rr)] = findFirFilterResponse(gainEst_value,delayEst_value,length(signalCleanFft));
            toc
            delayEst
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
        %                control_vector
        %                delayEst
                       [delayEst(mm,1),costFunction3] = delayEstimation(signalReceivedFft-signalCleanFft.*response_sim,signalCleanFft,init.candidateDelays);
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

        delayEst_in_time = delayEst./init.sampFreq;

        % %calculate groundTruth using image source method
        [a,b,c,d,e]=true_TOA_est(init.dimensions, init.sourcePos, init.distSourceToReceiv,init.sampFreq);
        groundTruth = (b/init.sampFreq); %calculated using image source model
        dist_groundTruth(bb,k) = b*init.soundSpeed/init.sampFreq/2;
        dist_est_from_method(bb,k) = delayEst(1)*init.soundSpeed/init.sampFreq/2;
    end
end


