# C++
### Google C++ 风格指南
**Google C++ 风格指南是一套由 Google 制定的，用于指导其内部软件开发的 C++ 编程规范。这些规则旨在确保 Google 的代码库具有高度的一致性和可读性，同时也方便维护和扩展。**
1. 命名约定

    类名：使用大驼峰式命名（CamelCase），如 MyClassName。
    变量名：使用小驼峰式命名（camelCase），如 myVariable。
    常量和枚举名：常量使用 k 后跟大驼峰式命名，如 kMaxCount。枚举值使用全部大写，单词间用下划线分隔，如 ENUM_VALUE。
    函数名：使用小驼峰式命名，如 myFunction。
    私有成员：使用末尾下划线，如 my_variable_。

2. 格式化

    缩进：使用两个空格作为缩进，不使用制表符（Tab）。
    行长度：每行代码长度不超过 80 个字符。
    花括号风格：左花括号 { 放在每个语句的同一行的末尾，右花括号 } 单独放在一行。

3. 指针和引用

    指针或引用符号与变量名紧密相连，如 int* ptr。

4. 头文件

    保护每个 .h 文件，避免头文件被多次包含。

5. 类设计

    明确区分 public、protected 和 private 区域。
    限制类的大小，避免过大的类设计。

6. 继承

    优先使用组合而非继承。
    如果使用公共继承，那么派生类是基类的子类型。

7. 异常

    Google C++ 代码中不使用 C++ 异常。推荐使用错误码或其他机制来处理错误。

8. 智能指针

    使用标准库提供的智能指针（如 std::unique_ptr 和 std::shared_ptr）来管理资源。

9. 并发

    明确并且安全地使用多线程。
---
### 模板
eg:

```
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node)
```

使用：

```
SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this)
```

---
### 函数指针
函数指针是 C++ 中的一种指针类型，它指向函数而不是数据。函数指针可以用来存储函数的地址，从而使得可以通过指针来调用函数。这在编程中特别有用，因为它允许将函数作为参数传递给其他函数，或者在运行时选择要调用的函数。

```
返回类型 (*指针名称)(参数类型1, 参数类型2, ...);
```
示例

```
int add(int a, int b) {
    return a + b;
}

int subtract(int a, int b) {
    return a - b;
}

int (*operation)(int, int);

// 将指针指向 add 函数
operation = add;
std::cout << "Addition: " << operation(5, 3) << std::endl;  // 输出: Addition: 8

// 将指针指向 subtract 函数
operation = subtract;
std::cout << "Subtraction: " << operation(5, 3) << std::endl;  // 输出: Subtraction: 2

```

---

### Lambda
```
[捕获列表](参数列表) -> 返回类型 {
    函数体
};
```
    **捕获列表**：定义了 lambda 函数体内可以访问的外部变量。可以是值捕获、引用捕获或不捕获。
    **参数列表**：与普通函数的参数列表类似。
    **返回类型**：可以省略，编译器会自动推导。
    **函数体**：执行 lambda 表达式时的代码。

例子

```
auto add = [](int a, int b) {
    return a + b;
};
std::cout << add(5, 3) << std::endl;  // 输出 8
```

### 值捕获

```
int x = 10;
auto add_to_x = [x](int a) { return x + a; };
std::cout << add_to_x(5) << std::endl;  // 输出 15
```

---

