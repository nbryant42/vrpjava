package org.example;

public record Edge(int src, int dest) {
    @Override
    public String toString() {
        return src + "-->" + dest;
    }
}
