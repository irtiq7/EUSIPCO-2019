function [gainEst] = gainEstimation(signal_reflectionsFft,signal_cleanFft,delayEst, response)

w=(0:(length(signal_cleanFft)-1))/length(signal_cleanFft)*2*pi;
for ww=1:length(w)
    if w(ww)<=pi
        z(ww,1)=exp(1i*w(ww)*-delayEst);
    else
        tmp=w(ww)-2*pi;
        z(ww,1)=exp(1i*tmp*-delayEst);
    end
end
Xz=signal_cleanFft.*z;
Yb=signal_reflectionsFft-signal_cleanFft.*response;
gainEst=(Yb'*Xz+Xz'*Yb)/(2*Xz'*Xz);

% Find filter response so that we could estimate the next peak
% filterEst=findFirFilterResponse(gainEst,delayEst,length(signal_cleanFft));
end