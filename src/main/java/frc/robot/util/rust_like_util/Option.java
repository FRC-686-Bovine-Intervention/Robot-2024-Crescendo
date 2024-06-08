package frc.robot.util.rust_like_util;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class Option<T> {
    private final T some;
    private final boolean has_val;

    private Option(T some) {
        this.some = some;
        this.has_val = this.some != null;
    }

    public static <T> Option<T> some(T some) {
        return new Option<T>(some);
    }
    public static <T> Option<T> none() {
        return new Option<T>(null);
    }

    public boolean is_some() {
        return has_val;
    }
    public boolean is_none() {
        return !has_val;
    }

    public T unwrap() {
        if(!is_some()) throw new NullPointerException("Unwrapped a None Option");
        return some;
    }
    public T unwrap_to_nullable() {
        return some;
    }
    public Optional<T> to_optional() {
        return Optional.ofNullable(some);
    }

    public <U> Option<U> map(Function<T, U> map_function) {
        if(is_none()) return none();
        return some(map_function.apply(some));
    }

    public <U> Option<U> and(Option<U> other) {
        return is_none() ? none() : other;
    }
    public <U> Option<U> and_then(Function<T, Option<U>> other) {
        return is_none() ? none() : other.apply(some);
    }
    public Option<T> or(Option<T> other) {
        return is_some() ? this : other;
    }
    public Option<T> or_else(Supplier<Option<T>> other) {
        return is_some() ? this : other.get();
    }

    public <E> Result<T, E> ok_or(E err) {
        return is_some() ? Result.ok(some) : Result.err(err);
    }
    public <E> Result<T, E> ok_or_else(Supplier<E> err) {
        return is_some() ? Result.ok(some) : Result.err(err.get());
    }

    @Override
    public String toString() {
        return is_some() ? String.format("Some(%s)", some) : "None";
    }
}
