init = struct();
init.plot_enable = 1;
init.sampFreq = 44100;
init.downSamplingFreq = 44100;
init.downSamplingRate = init.sampFreq/init.downSamplingFreq;
init.SNR = (-40:5:40);
init.candidateDelays=linspace(6,2000,1000);
init.candidateGains=linspace(0.00,0.95,200);
init.dimensions=[6 6 2.4]; %Dimension of soundlab is 5.4 x 6.38 x 4.05
init.absorptionRate=(0.8)';
init.epsilon = 1e-5;
% init.nsamples= [];
init.micType='omnidirectional';
% init.micType='bipolar';
init.soundSpeed=343;
% init.rirOrder=-1;
init.distSourceToReceiv=0.1;
init.distToWall=(0.1:0.5:2);
% remove 512 samples from the MCRoomSim since these are artifact
init.offset = 513;  
init.iteration=5;
init.signalLength = 2000;
init.fftLength = 2000;
% Input Signal
init.signalClean=randn(init.signalLength,1);

% Relax Algorithm Calculate Reflections R-1
init.R = 10;

init.sourcePos=[init.distToWall,init.dimensions(2)/2,init.dimensions(3)/2];
init.receivPos=[init.sourcePos(1:2),init.sourcePos(3)-init.distSourceToReceiv];
