1. 在写action的回调函数的时候，发现怎么写怎么报错，结果是
```
    send_goal_options.goal_response_callback =
        std::bind(&UR5e::goal_response_callback, this, std::placeholders::_1);

    ///···///

void UR5e::goal_response_callback(const rclcpp_action::ClientGoalHandle<FlwJntTra>::SharedPtr & future) {

}
```

这段代码的作用是将`UR5e::goal_response_callback`这个成员函数与`send_goal_options.goal_response_callback`绑定，`send_goal_options.goal_response_callback`是一个可调用对象，在客户端发送的目标被接受时被调用，然后使用`std::bind`将它和`UR5e::goal_response_callback`进行绑定，然后就会触发``UR5e::goal_response_callback``

- 首先需要在绑定的时候，第一个参数要是`UR5e::goal_response_callback`的指针，也就是`&UR5e::goal_response_callback`；
- 第二个`this`表示指向当前的对象实例，使得其在调用`goal_response_callback`的时候可以访问非静态成员（个人理解：类似于给权限？）；
- `std::placeholders::_1`表示传递给最终生成的可调用对象的第一个参数，可以理解为在触发回调条件的时候，会有一个或多个变量供使用，`_1`就是第一个变量；


