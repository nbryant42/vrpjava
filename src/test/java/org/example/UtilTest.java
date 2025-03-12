package org.example;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.*;

class UtilTest {
    @Test
    void isLowerTriangular() {
        // true
        assertTrue(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ZERO},
                {ZERO, ZERO}}));
        assertTrue(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ZERO},
                {ONE, ZERO}}));

        // false
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ZERO},
                {ZERO, ONE}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ZERO},
                {ONE, ONE}}));

        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ONE},
                {ZERO, ZERO}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ONE},
                {ZERO, ONE}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ONE},
                {ONE, ZERO}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ZERO, ONE},
                {ONE, ONE}}));

        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ONE, ONE},
                {ZERO, ZERO}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ONE, ONE},
                {ZERO, ONE}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ONE, ONE},
                {ONE, ZERO}}));
        assertFalse(Util.isLowerTriangular(new BigDecimal[][]{
                {ONE, ONE},
                {ONE, ONE}}));
    }
}