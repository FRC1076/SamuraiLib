package lib.functional;

import java.util.Objects;

@FunctionalInterface
public interface TriConsumer<T,U,V> {
    public abstract void accept(T t, U u, V v);

    public default TriConsumer<T,U,V> andThen(TriConsumer<? super T,? super U,? super V> after) {
        Objects.requireNonNull(after);
        return (t,u,v) -> {
            this.accept(t,u,v);
            after.accept(t,u,v);
        };
    }
}
