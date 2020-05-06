function [signalReceived] = Add_backgroundNoise(m,signalClean, signalReceived,signalLength,RIRs, SNR)

% BackgroundNoise
% 1 - Drone Noise
% 2 - Additive White Gaussian Noise

switch m
    case 1
        [droneSound, sampFreq] = audioread('allMotors_70.wav');
        %         find mean of all microphones
        droneSound = mean(droneSound');
        %        recording from one microphone 
        %         droneNoise = droneNoise(:,1);
        droneSound = droneSound(0.4e6:0.5e6)';
        droneSound = droneSound(1:signalLength);
        % filter the drone sound with RIR generated earlier
        signalDroneSound = fftfilt(RIRs,droneSound);
%         signalReceived=fftfilt(RIRs,signalClean);
        var_noise = var(fftfilt(RIRs,signalClean))/(10^(SNR/10));
        signalClean_snr = signalReceived;
        signalDroneSound = signalDroneSound/std(signalDroneSound)*sqrt(var_noise);
        signalReceived = signalDroneSound + signalClean_snr;
%         soundsc(signalReceived)
    case 2
        signalReceived = awgn(signalReceived,SNR, 'measured', 'dB');
end
end