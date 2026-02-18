clc; clear all; close all;

s = tf("s");
G1 = tf([6.4379e3],[1, 906.1205, 4.2852e4])
G2 = G1*1/s
p = pole(G2)

C = (s+abs(p(3)/100))/s

G = minreal(G2*C)