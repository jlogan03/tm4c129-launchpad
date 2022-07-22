#!/bin/sh

cargo +nightly call-stack --example udp > cg.dot;

dot -Tps cg.dot -o cg.ps;
