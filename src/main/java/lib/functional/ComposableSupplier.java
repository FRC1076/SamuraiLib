package lib.functional;

import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;

@FunctionalInterface
public interface ComposableSupplier<T> extends Supplier<T> {
    public default <R> ComposableSupplier<R> andThen(Function<? super T,? extends R> after) {
        Objects.requireNonNull(after);
        return () -> after.apply(this.get());
    }
}
