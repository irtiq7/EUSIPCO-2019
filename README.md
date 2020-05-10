# EUSIPCO-2019
In this repository, I am sharing my codes of my conference paper "Sound-based Distance Estimation for Indoor Navigation in the Presence of Egonoise"

This paper was published and presented at European Signal Processing Conference in 2019.

## Abstract
An off-the-shelf drone for indoor operation would come with a variety of different sensors that are used concurrently to avoid collision with, e.g., walls, but these sensors are typically uni-directional and offers limited spatial awareness. In this paper, we propose a model-based technique for distance estimation using sound and its reflections. More specifically, the technique is estimating Time-of-Arrivals (TOAs) of the reflected sound that could infer knowledge about room geometry and help in the design of sound-based collision avoidance. Our proposed solution is thus based on probing a known sound into an environment and then estimating the TOAs of reflected sounds recorded by a single microphone. The simulated results show that our approach to estimating TOAs for reflector position estimation works up to a distance of at least 2 meters even with significant additive noise, e.g., drone ego noise.

## Running the code

Download and install MCRoomSim before running the simulation.
You will need MATLAB/OCTAVE to run the files.

## Proposed TOA estimator
If you are interested to use the TOA estimator alone then you could find the file "delayEstimation.m"

## Paper
Link to my paper https://ieeexplore.ieee.org/abstract/document/8902694

## Youtube Link
https://www.youtube.com/watch?v=e17R2gl52UE&t=234s

## Citation
@INPROCEEDINGS{
    saqib2019,  
    author={U. {Saqib} and J. R. {Jensen}},  
    booktitle={2019 27th European Signal Processing Conference (EUSIPCO)},   
    title={Sound-based Distance Estimation for Indoor Navigation in the Presence of Ego Noise},   
    year={2019},  
    volume={},  
    number={},  
    pages={1-5},
}
