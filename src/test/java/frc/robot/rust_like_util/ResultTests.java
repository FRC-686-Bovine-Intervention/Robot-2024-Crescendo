package frc.robot.rust_like_util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

import frc.robot.util.rust_like_util.Result;

public class ResultTests {
    @Test
    void equality() {
        assertEquals(Result.ok(1), Result.ok(1));
        assertNotEquals(Result.ok(1), Result.ok(2));
        assertNotEquals(Result.ok(1), Result.ok("a"));
        assertNotEquals(Result.ok(1), Result.err(1));
        assertEquals(Result.err(1), Result.err(1));
        assertNotEquals(Result.err(1), Result.err(2));
        assertNotEquals(Result.err(1), Result.err("a"));
        assertNotEquals(Result.err(1), Result.ok(1));
    }
}
