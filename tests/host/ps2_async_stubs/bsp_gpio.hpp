#pragma once
#include "usertypes.hpp"
namespace bsp::gpio {
enum class input { data };
enum class output { cmd, clk, cs };
types::status write(output, bool);
types::status read(input, bool&);
types::status set_active(output, bool);
}
