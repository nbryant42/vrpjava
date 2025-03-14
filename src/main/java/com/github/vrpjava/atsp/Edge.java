package com.github.vrpjava.atsp;

/**
 * Represents an edge in a directed graph.
 *
 * @param src  the source location, as an integer array index into the corresponding problem specification
 * @param dest the destination location, as an integer array index into the corresponding problem specification
 */
public record Edge(int src, int dest) {
    @Override
    public String toString() {
        return src + "-->" + dest;
    }
}
