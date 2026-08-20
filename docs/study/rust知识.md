# rust 知识库

> 本部分为个人对 rust 中陌生的 API、类型、语法的学习总结。

## `impl`

`impl` 在 rust 中负责为类定义和实现方法，也可以在其中定义一些常量。

## `Into` 和 `From`

`From` trait 允许一种类型定义从另一种类型转换到自己的方法：

```rust
let my_str = "hello";
let my_string = String::from(my_str);
```

自定义类型转换机制：

```rust
use std::convert::From;

#[derive(Debug)]
struct Number {
    value: i32,
}

impl From<i32> for Number {
    fn from(item: i32) -> Self {
        Number {value: item}
    }
}
```

`Into` 就是 `From` 的反向，所以在实践中只需要实现 `From`：

```rust
use std::convert::From;

#[derive(Debug)]
struct Number {
    value: i32,
}

impl From<i32> for Number {
    fn from(item: i32) -> Self {
        Number {value: item}
    }
}

fn main() {
    let num = Number::from(30);
    let int = 50;
    // 必须注明目标类型，否则编译器无法推断 into() 要转换成什么类型
    let num2: Number = int.into();
}
```

对于易出错的类型转换，可以使用 `TryFrom` 和 `TryInto`。

如果想把类型转换成 `String` 类型，实现 `ToString` trait，实现 `fmt::Display` 也可以自动实现。

## `&`、`*`、`ref` 和 `ref mut`

`&` 用于对一个变量创建引用，下方代码中的 `value_ref` 类型是 `&i32`：

```rust
let value = 1;
let value_ref = &value;
```

如果想要访问引用的对象，需要使用 `*` 来表示：

```rust
let value = &2;
let value2 = *value + 1;
```

同时还可以使用 `ref` 字段创建引用，下面代码中的 `value` 实际上是 `&i32` 类型：

```rust
let ref value = 1;
```

对于 `mut val`，可以使用 `let mut ref` 创建引用值。

```rust
let mut source = 1;
let ref mut value = source;
*value += 1;
```

`ref` 与 `&` 的区别：`&` 出现在表达式侧（`let r = &v;`），`ref` 出现在模式侧（`let ref r = v;` 或 `match` 分支中绑定引用）。






