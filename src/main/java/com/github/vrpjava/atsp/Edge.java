package com.github.vrpjava.atsp;

public record Edge(int src, int dest) {
    @Override
    public String toString() {
        return src + "-->" + dest;
    }
}
