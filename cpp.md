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
[捕获列表](参数列表) -> 返回类型 { 函数体 };
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

值捕获

```
int x = 10;
auto add_to_x = [x](int a) { return x + a; };
std::cout << add_to_x(5) << std::endl;  // 输出 15
```

---

### std::map中“map.at(key)”和"map[key]"异同
在 C++ 中，std::map 的 at() 方法和 operator[]（即 map[key]）都用于访问键为 key 的元素，但它们在行为和用途上有一些重要的区别：

`std::map::at(key)`

    访问已存在的元素：如果键 key 存在于映射中，at() 返回对应的值的引用。

    边界检查：如果键 key 不存在于映射中，at() 抛出 std::out_of_range 异常。

    只用于访问：at() 不能用于插入新元素，它只用于访问或修改已存在的元素。
---

### std::copy_backward 
std::copy_backward 是 C++ 标准库中的一个函数，用于将一个元素范围内的元素复制到另一个范围，同时保持元素的顺序不变。与 std::copy 不同，std::copy_backward 从范围的末尾开始复制，这对于在目标范围有重叠的情况下非常有用，因为它可以防止源范围的元素被过早覆盖。

```
std::copy_backward(InputIt first, InputIt last, OutputIt d_last);
```
first 和 last 分别是要复制的元素范围的起始和结束迭代器。
d_last 是目标范围的结束迭代器。

函数将 [first, last) 范围内的元素复制到以 d_last 结束的范围中。复制是从 last 指向的最后一个元素开始的，复制到 d_last 之前的位置，并从后向前继续进行。

example: /home/chopin/autoware/autoware.universe/localization/ekf_localizer/src/ekf_module.cpp

```
//更新 accumulated_delay_times_ 容器，将新的延迟时间 dt 累积到现有的延迟时间中
void EKFModule::accumulate_delay_time(const double dt)
{
  // Shift the delay times to the right.
  std::copy_backward(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end() - 1,
    accumulated_delay_times_.end());

  // Add a new element (=0) and, and add delay time to the previous elements.
  accumulated_delay_times_.front() = 0.0;
  for (size_t i = 1; i < accumulated_delay_times_.size(); ++i) {
    accumulated_delay_times_[i] += dt;
  }
}
```
### constexpr和const int有什么区别?
#### 初始化时机：
const 变量可以在运行时初始化，而 constexpr 变量必须在编译时就确定其值。
#### 用途：
const 更多地用于防止变量被修改，适用于运行时的值；constexpr 用于定义编译时常量，适用于需要编译时确定值的场景。


```
const int runtimeConst = someFunction();  // 可以在运行时初始化
constexpr int compileTimeConst = 5 * 2;   // 必须在编译时就能计算出结果
```

在实际应用中，选择 const 还是 constexpr 取决于你的需求：如果你需要在编译时确定变量的值，或者变量的值是通过计算得到的编译时常量，那么应该使用 constexpr。如果只是想防止变量在运行时被修改，使用 const 就足够了。
