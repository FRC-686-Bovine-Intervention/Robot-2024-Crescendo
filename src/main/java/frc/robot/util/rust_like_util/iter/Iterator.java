package frc.robot.util.rust_like_util.iter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import edu.wpi.first.math.Pair;
import frc.robot.util.rust_like_util.Option;

public interface Iterator<I> extends IntoIterator<I> {
    public Option<I> next();
    @Override
    public default Iterator<I> into_iter() {return this;}

    // Constructors
    public static <I> DoubleEndedIterator<I> empty() {
        return new DoubleEndedIterator<I>() {
            @Override
            public Option<I> next() {
                return Option.none();
            }

            @Override
            public Option<I> next_back() {
                return Option.none();
            }
        };
    }
    public static <I> DoubleEndedIterator<I> once(I once) {
        return new DoubleEndedIterator<I>() {
            private boolean consumed = false;
            @Override
            public Option<I> next() {
                return Option.if_then(!consumed, () -> {
                    consumed = true;
                    return once;
                });
            }

            @Override
            public Option<I> next_back() {
                return next();
            }
        };
    }
    public static <I> DoubleEndedIterator<I> of(I[] array) {
        return new DoubleEndedIterator<I>() {
            private int front_index = 0;
            private int back_index = array.length - 1;
            @Override
            public Option<I> next() {
                return Option.if_then(front_index <= back_index, () -> array[front_index++]);
            }
            @Override
            public Option<I> next_back() {
                return Option.if_then(front_index <= back_index, () -> array[back_index--]);
            }
        };
    }
    public static <I> DoubleEndedIterator<I> of(List<I> list) {
        return new DoubleEndedIterator<I>() {
            private int front_index = 0;
            private int back_index = list.size() - 1;
            @Override
            public Option<I> next() {
                return Option.if_then(front_index <= back_index, () -> list.get(front_index++));
            }
            @Override
            public Option<I> next_back() {
                return Option.if_then(front_index <= back_index, () -> list.get(back_index--));
            }
        };
    }
    public static <I> Iterator<I> of(Iterable<I> iter) {
        return new Iterator<I>() {
            private final java.util.Iterator<I> iterator = iter.iterator();
            @Override
            public Option<I> next() {
                return Option.if_then(iterator.hasNext(), iterator::next);
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
    public default Iterator<I> chain(IntoIterator<I> other) {
        var parent = this;
        var other_iter = other.into_iter();
        return new Iterator<I>() {
            @Override
            public Option<I> next() {
                return parent.next().or_else(other_iter::next);
            }
        };
    }
    public default <U> Iterator<Pair<I, U>> zip(IntoIterator<U> other) {
        var parent = this;
        var other_iter = other.into_iter();
        return new Iterator<Pair<I, U>>() {
            @Override
            public Option<Pair<I, U>> next() {
                var a = parent.next();
                if(a.is_none()) return Option.none();
                var b = other_iter.next();
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
    public default Iterator<I> take(int n) {
        var parent = this;
        return new Iterator<I>() {
            private int current_index = 0;
            @Override
            public Option<I> next() {
                if(current_index >= n) return Option.none();
                current_index++;
                return parent.next();
            }
        };
    }
    public default Iterator<I> skip(int n) {
        var parent = this;
        return new Iterator<I>() {
            private int current_index = 0;
            @Override
            public Option<I> next() {
                for(; current_index < n; current_index++) {
                    parent.next();
                }
                return parent.next();
            }
        };
    }
    public default Iterator<I> take_while(Predicate<I> predicate) {
        var parent = this;
        return new Iterator<I>() {
            private boolean taking = true;
            @Override
            public Option<I> next() {
                if(!taking) return Option.none();
                var item = parent.next();
                taking = item.is_some_and(predicate);
                return item;
            }
        };
    }
    public default Iterator<I> skip_while(Predicate<I> predicate) {
        var parent = this;
        return new Iterator<I>() {
            private boolean skipping = true;
            @Override
            public Option<I> next() {
                var item = parent.next();
                skipping = item.is_some_and(predicate);
                return skipping ? Option.none() : item;
            }
        };
    }
    public default <U> Iterator<U> map_while(Function<I, Option<U>> predicate) {
        var parent = this;
        return new Iterator<U>() {
            private boolean mapping = true;
            @Override
            public Option<U> next() {
                if(!mapping) return Option.none();
                var item = parent.next().and_then(predicate);
                mapping = item.is_some();
                return item;
            }
        };
    }
    public static <I> Iterator<I> flatten(Iterator<Iterator<I>> fat) {
        return new Iterator<I>() {
            private Option<Iterator<I>> current_iterator;
            @Override
            public Option<I> next() {
                if(current_iterator == null) {
                    current_iterator = fat.next();
                }
                if(current_iterator.is_none()) return Option.none();
                var item = current_iterator.unwrap().next();
                if(item.is_none()) {
                    current_iterator = fat.next();
                    return next();
                }
                return item;
            }
        };
    }
    public default <U> Iterator<U> flat_map(Function<I, IntoIterator<U>> map) {
        var parent = this;
        return new Iterator<U>() {
            private Option<Iterator<U>> current_iterator;
            @Override
            public Option<U> next() {
                if(current_iterator == null) {
                    current_iterator = parent.next().map(map).map((iter) -> iter.into_iter());
                }
                if(current_iterator.is_none()) return Option.none();
                var item = current_iterator.unwrap().next();
                if(item.is_none()) {
                    current_iterator = parent.next().map(map).map((iter) -> iter.into_iter());
                    return next();
                }
                return item;
            }
        };
    }

    // Consumers
    public default void for_each(Consumer<I> for_each) {
        for(var ele = next(); ele.is_some(); ele = next()) {
            for_each.accept(ele.unwrap());
        }
    }
    public interface Collector<I, C> {
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
        public static Collector<Character, String> string() {
            return new Collector<Character, String>() {
                private String string = "";
                @Override
                public void collect(Character element) {
                    string = string.concat(element.toString());
                }
                @Override
                public String build() {
                    return string;
                }
            };
        }
    }
    public default <C> C collect(Collector<I, C> collector) {
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
