package frc.robot.rust_like_util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import frc.robot.util.rust_like_util.Option;
import frc.robot.util.rust_like_util.Result;

public class OptionTests {
    @Test
    void constructors() {
        assertEquals(Option.some(1), Option.some(1));
        assertEquals(Option.none(), Option.none());
        assertEquals(Option.if_then_some(true, 1), Option.some(1));
        assertEquals(Option.if_then_some(false, 1), Option.none());
    }
    @Test
    void boolean_functions() {
        assertTrue(Option.none().is_none());
        assertFalse(Option.none().is_some());
        assertFalse(Option.none().is_some_and((s) -> true));
        assertTrue(Option.some(1).is_some());
        assertFalse(Option.some(1).is_none());
        assertTrue(Option.some(1).is_some_and((s) -> s == 1));
        assertFalse(Option.some(1).is_some_and((s) -> s == 2));
    }
    @Test
    void equality() {
        assertEquals(Option.some(1), Option.some(1));
        assertNotEquals(Option.some(1), Option.some(2));
        assertNotEquals(Option.some(1), Option.some("a"));
        assertNotEquals(Option.some(1), Option.none());
        assertNotEquals(Option.none(), Option.some(1));
        assertEquals(Option.none(), Option.none());
    }
    @Test
    void map() {
        assertEquals(Option.some(1).map((s) -> s * 2), Option.some(2));
        assertEquals(Option.some(1).map((s) -> s.toString()), Option.some("1"));
    }
    @Test
    void flatten() {
        assertEquals(Option.flatten(Option.some(Option.some(1))), Option.some(1));
        assertEquals(Option.flatten(Option.some(Option.none())), Option.none());
        assertEquals(Option.flatten(Option.none()), Option.none());
    }
    @Test
    void logical() {
        assertEquals(Option.some(1).and(Option.some(2)), Option.some(2));
        assertEquals(Option.none().and(Option.some(2)), Option.none());
        assertEquals(Option.some(1).or(Option.some(2)), Option.some(1));
        
        assertEquals(Option.none().or(Option.some(2)), Option.some(2));
        assertEquals(Option.some(1).or(Option.none()), Option.some(1));
        assertEquals(Option.none().or(Option.none()), Option.none());
    }
    @Test
    void unwrappers() {
        assertEquals(Option.some(1).unwrap(), 1);
        assertThrows(NullPointerException.class, Option.none()::unwrap);
        assertEquals(Option.some(1).unwrap_to_nullable(), 1);
        assertEquals(Option.none().unwrap_to_nullable(), null);
        assertEquals(Option.some(1).unwrap_to_optional(), Optional.of(1));
        assertEquals(Option.none().unwrap_to_optional(), Optional.empty());
    }
    @Test
    void to_results() {
        assertEquals(Option.some(1).ok_or("a"), Result.ok(1));
        assertEquals(Option.none().ok_or("a"), Result.err("a"));
    }
}
