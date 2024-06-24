package frc.robot.util.rust_like_util;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;

import frc.robot.util.rust_like_util.iter.IntoIterator;
import frc.robot.util.rust_like_util.iter.Iterator;

/**
 * A Java version of Rust's Result<T, E> enum <p>
 * {@code Result<T, Unit>} is isomorphic to {@code Option<T>} and {@code Optional<T>}<p>
 * {@code Result<Unit, Unit>} is isomorphic to {@code boolean}, {@code Option<Unit>}, and {@code Optional<Unit>}
 */
public class Result<T, E> implements IntoIterator<T> {
    private final T ok;
    private final E err;
    private final boolean is_ok;

    private Result(T ok, E err) {
        this.ok = ok;
        this.err = err;
        this.is_ok = this.ok != null;
    }

    /**
     * Wraps {@code ok} in a {@code Result<T, E>}
     * @param <T> The {@code Ok} type
     * @param <E> The {@code Err} type (usually inferred)
     * @param ok The {@code Ok} value
     * @return The wrapper {@code Result}
     */
    public static <T, E> Result<T, E> ok(T ok) {
        return new Result<T, E>(ok, null);
    }
    /**
     * Wraps {@code err} in a {@code Result<T, E>}
     * @param <T> The {@code Ok} type (usually inferred)
     * @param <E> The {@code Err} type
     * @param ok The {@code Err} value
     * @return The wrapper {@code Result}
     */
    public static <T, E> Result<T, E> err(E err) {
        return new Result<T, E>(null, err);
    }

    public boolean is_ok() {
        return is_ok;
    }
    public boolean is_err() {
        return !is_ok;
    }

    /**
     * @return The wrapped {@code Ok} value
     * @throws NullPointerException If {@code Err}
     */
    public T unwrap() throws NullPointerException {
        if(is_err()) throw new NullPointerException("Unwrapped a Err Result");
        return ok;
    }
    /**
     * @param other The value to use if {@code Err}
     * @return The wrapped {@code Ok} value OR {@code other} if {@code Err}
     */
    public T unwrap_or(T other) {
        return is_err() ? other : ok;
    }
    /**
     * @param other The function to generate the return value if {@code Err}
     * @return The wrapped {@code Ok} value OR {@code other.apply(err)} if {@code Err} 
     */
    public T unwrap_or_else(Function<E,T> other) {
        return is_err() ? other.apply(err) : ok;
    }
    /**
     * @return The wrapped {@code Err} value
     * @throws NullPointerException If {@code Ok}
     */
    public E unwrap_err() throws NullPointerException {
        if(is_ok()) throw new NullPointerException("Unwrapped a Ok Result");
        return err;
    }

    /**
     * Converts {@code this} into {@code Option<T>}
     * @return The converted {@code Option}, mapping {@code Ok(ok)} to {@code Option.some(ok)} and {@code Err(err)} to {@code Option.none()}
     */
    public Option<T> ok() {
        return Option.some(ok);
    }
    /**
     * Converts {@code this} into {@code Option<E>}
     * @return The converted {@code Option}, mapping {@code Ok(ok)} to {@code Option.none()} and {@code Err(err)} to {@code Option.some(err)}
     */
    public Option<E> err() {
        return Option.some(err);
    }

    /**
     * Maps the wrapped {@code Ok} value using {@code map_function}
     * @param <U> The new {@code Ok} type
     * @param map_function The function to use to map the wrapped {@code Ok} value
     * @return A {@code Result<U, E>} containing the mapped {@code Ok} value, or {@code Err(err)}
     */
    public <U> Result<U, E> map(Function<T, U> map_function) {
        return is_err() ? err(err) : ok(map_function.apply(ok));
    }
    /**
     * Maps the wrapped {@code Err} value using {@code map_function}
     * @param <U> The new {@code Err} type
     * @param map_function The function to use to map the wrapped {@code Err} value
     * @return A {@code Result<T, U>} containing the mapped {@code Err} value, or {@code Ok(ok)}
     */
    public <U> Result<T, U> map_err(Function<E, U> map_function) {
        return is_ok() ? ok(ok) : err(map_function.apply(err));
    }

    public static <T, E> Result<T, E> flatten(Result<Result<T, E>, E> fat) {
        return fat.and_then((o) -> o);
    }

    /**
     * @param inspect_function Function to call if {@code Ok}
     * @return {@code this}
     */
    public Result<T, E> inspect(Consumer<T> inspect_function) {
        if(is_ok()) {
            inspect_function.accept(ok);
        }
        return this;
    }
    /**
     * @param inspect_function Function to call if {@code Err}
     * @return {@code this}
     */
    public Result<T, E> inspect_err(Consumer<E> inspect_function) {
        if(is_err()) {
            inspect_function.accept(err);
        }
        return this;
    }

    /**
     * If {@code Err} returns {@code Err(err)}, otherwise returns {@code other}
     * @param <U> The type of the wrapped {@code Ok} value
     * @param other The {@code Result} to return if {@code Ok}
     * @return {@code other} if {@code Ok}
     */
    public <U> Result<U, E> and(Result<U,E> other) {
        return is_err() ? err(err) : other;
    }
    /**
     * If {@code Err} returns {@code Err(err)}, otherwise returns {@code other.get()}<p>
     * Also known as {@code flat_map}, as it is equivalent to {@code flatten(this.map(other))}
     * @param <U> The type of the wrapped {@code Ok} value
     * @param other The function to poll and return if {@code Ok}
     * @return {@code other.get()} if {@code Ok}
     */
    public <U> Result<U, E> and_then(Function<T, Result<U, E>> other) {
        return is_err() ? err(err) : other.apply(ok);
    }
    /**
     * If {@code Err} returns {@code other}, otherwise returns {@code this}
     * @param other The {@code Result} to return if {@code Err}
     * @return {@code other} if {@code Err}, otherwise {@code this}
     */
    public <U> Result<T, U> or(Result<T, U> other) {
        return is_ok() ? ok(ok) : other;
    }
    /**
     * If {@code Err} returns {@code other.get()}, otherwise returns {@code this}
     * @param other The function to poll and return if {@code Err}
     * @return {@code other.get()} if {@code Err}, otherwise {@code this}
     */
    public <U> Result<T, U> or_else(Function<E, Result<T, U>> other) {
        return is_ok() ? ok(ok) : other.apply(err);
    }

    /**
     * Converts {@code this} into an {@code Iterator<T>} that returns {@code this.ok()} once then {@code None} after
     * @return The converted {@code Iterator<T>}
     */
    @Override
    public Iterator<T> into_iter() {
        return ok().into_iter();
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof Result<?,?> other) {
            if(is_ok && Objects.equals(ok, other.ok)) {
                return true;
            }
            if(!is_ok && Objects.equals(err, other.err)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public String toString() {
        return is_ok() ? String.format("Ok(%s)", ok) : String.format("Err(%s)", err);
    }
}
