clear all
close all
clc

file = importdata('training_data.txt');

inputs = [];
inputs(:, 1:21) = file(:, 1:21);

outputs = [];
outputs(:, 1:5) = file(:, 22:26);