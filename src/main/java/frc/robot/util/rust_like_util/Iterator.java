package frc.robot.util.rust_like_util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import edu.wpi.first.math.Pair;

public interface Iterator<I> {
    public Option<I> next();

    // Constructors
    public static <I> DoubleEndedIterator<I> of(List<I> list) {
        return new DoubleEndedIterator<I>() {
            private int front_index = 0;
            private int back_index = list.size() - 1;
            @Override
            public Option<I> next() {
                if(front_index > back_index) return Option.none();
                return Option.some(list.get(front_index++));
            }
            @Override
            public Option<I> next_back() {
                if(front_index > back_index) return Option.none();
                return Option.some(list.get(back_index--));
            }
        };
    }

    // Adapters
    public default Iterator<I> inspect(Consumer<I> inspect_function) {
        var parent = this;
        return new Iterator<I>() {
            @Override
            public Option<I> next() {
                return parent.next().inspect(inspect_function);
            }
        };
    }
    public default <U> Iterator<U> map(Function<I, U> map_function) {
        var parent = this;
        return new Iterator<U>() {
            @Override
            public Option<U> next() {
                return parent.next().map(map_function);
            }
        };
    }
    public default Iterator<I> filter(Predicate<I> predicate) {
        var parent = this;
        return new Iterator<I>() {
            @Override
            public Option<I> next() {
                for(var ele = parent.next(); ele.is_some(); ele = parent.next()) {
                    if(predicate.test(ele.unwrap())) return ele;
                }
                return Option.none();
            }
        };
    }
    public default <U> Iterator<U> filter_map(Function<I, Option<U>> filter_map_function) {
        var parent = this;
        return new Iterator<U>() {
            @Override
            public Option<U> next() {
                for(var ele = parent.next(); ele.is_some(); ele = parent.next()) {
                    var test = filter_map_function.apply(ele.unwrap());
                    if(test.is_some()) return test;
                }
                return Option.none();
            }
        };
    }
    public default Iterator<I> chain(Iterator<I> other) {
        var parent = this;
        return new Iterator<I>() {
            @Override
            public Option<I> next() {
                return parent.next().or_else(other::next);
            }
        };
    }
    public default <U> Iterator<Pair<I, U>> zip(Iterator<U> other) {
        var parent = this;
        return new Iterator<Pair<I, U>>() {
            @Override
            public Option<Pair<I, U>> next() {
                var a = parent.next();
                if(a.is_none()) return Option.none();
                var b = other.next();
                if(b.is_none()) return Option.none();
                return Option.some(Pair.of(a.unwrap(), b.unwrap()));
            }
        };
    }
    public default Iterator<Pair<Integer, I>> enumerate() {
        var parent = this;
        return new Iterator<Pair<Integer,I>>() {
            private int current_index = 0;
            @Override
            public Option<Pair<Integer, I>> next() {
                return parent.next().map((ele) -> Pair.of(current_index++, ele));
            }
        };
    }
    public default Iterator<I> skip(int n) {
        for(int i = 0; i < n; i++) {
            next();
        }
        return this;
    }

    // Consumers
    public default void for_each(Consumer<I> for_each) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            for_each.accept(ele.unwrap());
        }
    }
    public interface Collector<I, C extends Collection<I>> {
        public void collect(I element);
        public C build();

        public static <I> Collector<I, ArrayList<I>> array_list() {
            return new Collector<I,ArrayList<I>>() {
                private final ArrayList<I> list = new ArrayList<>();
                @Override
                public void collect(I element) {
                    list.add(element);
                }
                @Override
                public ArrayList<I> build() {
                    return list;
                }
            };
        }
    }
    public default <C extends Collection<I>> C collect(Collector<I, C> collector) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            collector.collect(ele.unwrap());
        }
        return collector.build();
    }
    public default int count() {
        var count = 0;
        for(var ele = next(); ele.is_some(); ele = next()) {
            count++;
        }
        return count;
    }
    public default Option<I> reduce(BiFunction<I, I, I> reduction_function) {
        var accumulator = next();
        for(var ele = next(); ele.is_some(); ele = next()) {
            accumulator = Option.some(reduction_function.apply(accumulator.unwrap(), ele.unwrap()));
        }
        return accumulator;
    }
    public default I fold(I initial, BiFunction<I, I, I> folding_function) {
        var accumulator = initial;
        for(var ele = next(); ele.is_some(); ele = next()) {
            accumulator = folding_function.apply(accumulator, ele.unwrap());
        }
        return accumulator;
    }
    public default Option<I> find(Predicate<I> predicate) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            if(predicate.test(ele.unwrap())) return ele;
        }
        return Option.none();
    }
    public default Option<Integer> position(Predicate<I> predicate) {
        var index = 0;
        for(var ele = next(); ele.is_some(); ele = next()) {
            if(predicate.test(ele.unwrap())) return Option.some(index);
            index++;
        }
        return Option.none();
    }
    public default boolean any(Predicate<I> predicate) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            if(predicate.test(ele.unwrap())) return true;
        }
        return false;
    }
    public default boolean all(Predicate<I> predicate) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            if(!predicate.test(ele.unwrap())) return false;
        }
        return true;
    }

    public static interface DoubleEndedIterator<I> extends Iterator<I> {
        public Option<I> next_back();

        public default DoubleEndedIterator<I> rev() {
            var prev = this;
            return new DoubleEndedIterator<I>() {
                @Override
                public Option<I> next() {
                    return prev.next_back();
                }

                @Override
                public Option<I> next_back() {
                    return prev.next();
                }
            };
        }
    }
}
