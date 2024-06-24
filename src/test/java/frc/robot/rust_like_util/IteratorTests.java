package frc.robot.rust_like_util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;

import frc.robot.util.rust_like_util.Option;
import frc.robot.util.rust_like_util.iter.Iterator;

public class IteratorTests {
    @Test
    void constructors() {
        var empty = Iterator.empty();
        assertEquals(empty.next(), Option.none());
        var once = Iterator.once(1);
        assertEquals(once.next(), Option.some(1));
        assertEquals(once.next(), Option.none());
        var list = Iterator.of(List.of(1,2));
        assertEquals(list.next(), Option.some(1));
        assertEquals(list.next(), Option.some(2));
        assertEquals(list.next(), Option.none());
        var array = Iterator.of(new Integer[]{1,2});
        assertEquals(array.next(), Option.some(1));
        assertEquals(array.next(), Option.some(2));
        assertEquals(array.next(), Option.none());
        var iterable = Iterator.of(Set.of(1,2));
        assertTrue(iterable.next().is_some());
        assertTrue(iterable.next().is_some());
        assertEquals(iterable.next(), Option.none());
    }
    @Test
    void map() {
        var map = Iterator.of(List.of(1,2)).map((i) -> i * 2);
        assertEquals(map.next(), Option.some(2));
        assertEquals(map.next(), Option.some(4));
        assertEquals(map.next(), Option.none());
    }
    @Test
    void filter() {
        var filter = Iterator.of(List.of(1,2)).filter((i) -> i == 1);
        assertEquals(filter.next(), Option.some(1));
        assertEquals(filter.next(), Option.none());
    }
    @Test
    void filter_map() {
        var filter_map = Iterator.of(List.of(Option.some(1), Option.none(), Option.some(3))).filter_map((i) -> i);
        assertEquals(filter_map.next(), Option.some(1));
        assertEquals(filter_map.next(), Option.some(3));
        assertEquals(filter_map.next(), Option.none());
    }
    @Test
    void flat_map() {
        var flat_map = Iterator.of(List.of(List.of(1,2), List.of(3), List.of(4,5))).flat_map((i) -> Iterator.of(i));
        assertEquals(flat_map.next(), Option.some(1));
        assertEquals(flat_map.next(), Option.some(2));
        assertEquals(flat_map.next(), Option.some(3));
        assertEquals(flat_map.next(), Option.some(4));
        assertEquals(flat_map.next(), Option.some(5));
        assertEquals(flat_map.next(), Option.none());
    }
    @Test
    void chain() {
        var chain = Iterator.of(List.of(1,2,3)).chain(Iterator.of(List.of(1,2,3)));
        assertEquals(chain.next(), Option.some(1));
        assertEquals(chain.next(), Option.some(2));
        assertEquals(chain.next(), Option.some(3));
        assertEquals(chain.next(), Option.some(1));
        assertEquals(chain.next(), Option.some(2));
        assertEquals(chain.next(), Option.some(3));
        assertEquals(chain.next(), Option.none());
    }
    @Test
    void zip() {
        var zip = Iterator.of(List.of(1,2,3)).zip(Iterator.of(List.of(4,5,6,7)));
        assertEquals(zip.next().map((o) -> o.getFirst()), Option.some(1));
        assertEquals(zip.next().map((o) -> o.getSecond()), Option.some(5));
        assertEquals(zip.next().map((o) -> o.getFirst()), Option.some(3));
        assertEquals(zip.next(), Option.none());
    }
    @Test
    void enumerate() {
        var enumerate = Iterator.of(List.of(4,5,6)).enumerate();
        assertEquals(enumerate.next().map((o) -> o.getFirst()), Option.some(1));
        assertEquals(enumerate.next().map((o) -> o.getSecond()), Option.some(5));
        assertEquals(enumerate.next().map((o) -> o.getFirst()), Option.some(3));
        assertEquals(enumerate.next(), Option.none());
    }
    @Test
    void take() {
        var take = Iterator.of(List.of(1,2,3)).take(2);
        assertEquals(take.next(), Option.some(1));
        assertEquals(take.next(), Option.some(2));
        assertEquals(take.next(), Option.none());
    }
    @Test
    void skip() {
        var skip = Iterator.of(List.of(1,2,3)).skip(2);
        assertEquals(skip.next(), Option.some(3));
        assertEquals(skip.next(), Option.none());
    }
}
