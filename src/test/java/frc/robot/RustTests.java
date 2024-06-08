package frc.robot;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.robot.util.rust_like_util.Iterator;
import frc.robot.util.rust_like_util.Option;
import frc.robot.util.rust_like_util.Unit;
import frc.robot.util.rust_like_util.Iterator.Collector;

public class RustTests {
    @Test
    void iterator_test() {
        var list = List.of(1,2,3,4,5);
        var collect = Iterator.of(list).skip(2).collect(Collector.array_list());
        System.out.println(collect);
        System.out.println(Iterator.of(collect).map((ele) -> ele.toString()).reduce((acc, ele) -> acc + "," + ele));
        System.out.println(Iterator.of(collect).reduce((acc, ele) -> acc * ele));
        System.out.println(
            Iterator.of(list)
            .enumerate()
            .filter((e) -> e.getSecond() != 4)
            .map((pair) -> String.format("(%s, %s)", pair.getFirst(), pair.getSecond()))
            .collect(Collector.array_list())
        );
    }
    @Test
    void result_test() {
        var option = Option.some(5);
        System.out.println(option);
        option = Option.none();
        System.out.println(option);
        var unit_result = option.ok_or(new Unit());
        System.out.println(unit_result);
        var string_result = option.ok_or("Nawoeij");
        System.out.println(string_result);
    }
}
