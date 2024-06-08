package frc.robot.util.rust_like_util;

import java.util.function.Function;

public class Result<T, E> {
    private final T ok;
    private final E err;
    private final boolean is_ok;

    private Result(T ok, E err) {
        this.ok = ok;
        this.err = err;
        this.is_ok = this.ok != null;
    }

    public static <T, E> Result<T, E> ok(T ok) {
        return new Result<T, E>(ok, null);
    }
    public static <T, E> Result<T, E> err(E err) {
        return new Result<T, E>(null, err);
    }

    public boolean is_ok() {
        return is_ok;
    }
    public boolean is_err() {
        return !is_ok;
    }

    public T unwrap() {
        if(is_err()) throw new NullPointerException("Unwrapped a Err Result");
        return ok;
    }
    public T unwrap_or(T other) {
        return is_err() ? other : ok;
    }
    public T unwrap_or_else(Function<E,T> other) {
        return is_err() ? other.apply(err) : ok;
    }
    public E unwrap_err() {
        if(is_ok()) throw new NullPointerException("Unwrapped a Ok Result");
        return err;
    }

    public Option<T> ok() {
        return Option.some(ok);
    }
    public Option<E> err() {
        return Option.some(err);
    }

    public <U> Result<U, E> map(Function<T, U> map_function) {
        return is_err() ? err(err) : ok(map_function.apply(ok));
    }
    public <U> Result<T, U> map_err(Function<E, U> map_function) {
        return is_ok() ? ok(ok) : err(map_function.apply(err));
    }

    public <U> Result<U, E> and(Result<U,E> other) {
        return is_err() ? err(err) : other;
    }
    public <U> Result<U, E> and_then(Function<T, Result<U, E>> other) {
        return is_err() ? err(err) : other.apply(ok);
    }
    public <U> Result<T, U> or(Result<T, U> other) {
        return is_ok() ? ok(ok) : other;
    }
    public <U> Result<T, U> or_else(Function<E, Result<T, U>> other) {
        return is_ok() ? ok(ok) : other.apply(err);
    }

    @Override
    public String toString() {
        return is_ok() ? String.format("Ok(%s)", ok) : String.format("Err(%s)", err);
    }
}
