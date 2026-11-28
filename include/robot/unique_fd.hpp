#pragma once

#include <unistd.h>

namespace robot {
    
    class UniqueFd {
    public:
        // 写上 = default 等于告诉编译器：“别偷懒，请照常帮我生成默认无参构造”。
        // 创建出来的对象会采用 L30 的默认值 fd_ = -1。 因为私有成员 fd_ 是 int 类型，默认值 -1 
        UniqueFd() = default;
        /*
            这个是函数重载，传入一个 int 类型的 fd 参数，构造出来的对象会把 fd_ 初始化为传入的 fd 参数值。
            explicit（禁止隐式类型转换）：如果不写 explicit，别人写成 UniqueFd u = 3; 
            或者把一个普通整数 3 直接传给需要 UniqueFd 参数的函数时，
            编译器会“热心”地偷偷把 3 变成 UniqueFd(3)——这非常危险（函数一结束就偷偷把 fd=3 给 close 掉了！）。
        */ 
        explicit UniqueFd(int fd) noexcept : fd_(fd) {}
        ~UniqueFd() { reset(); }
        /*
            知识点 1：拷贝构造函数。用来处理 UniqueFd b(a); 或 UniqueFd b = a;。参数必须是 const UniqueFd&（常量左值引用，避免无限递归且不修改原对象）。
            知识点 2：= delete（删除函数，C++11）。明确封死拷贝能力！因为一个 fd 只能有一个主人，如果拷贝出两份，
            两个对象析构时会连续 close 同一个 fd 两次，造成严重 Bug。
        */
        UniqueFd(const UniqueFd&) = delete;
        /*
            知识点：拷贝赋值运算符重载。operator= 重载的是赋值号 =（比如 b = a;）。同样用 = delete 封死。
        */
        UniqueFd& operator=(const UniqueFd&) = delete;
        
        UniqueFd(UniqueFd&& other) noexcept : fd_(other.release()) {}
        UniqueFd& operator=(UniqueFd&& other) noexcept {
            if (this != &other) reset(other.release());
            return *this;
        }

        int get() const noexcept { return fd_; }
        explicit operator bool() const noexcept { return fd_ >= 0; }
        int release() noexcept { int out = fd_; fd_ = -1; return out; }
        void reset(int fd = -1) noexcept {
            if (fd_ >= 0) ::close(fd_);
            fd_ = fd;
        }

    private:
        int fd_{-1};
    };

}  // namespace robot
