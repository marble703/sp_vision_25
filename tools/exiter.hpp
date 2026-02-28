#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
class Exiter
{
public:
  Exiter();

  bool exit() const;
};

// 全局函数，用于检查是否需要退出
bool should_exit();

}  // namespace tools

#endif  // TOOLS__EXITER_HPP