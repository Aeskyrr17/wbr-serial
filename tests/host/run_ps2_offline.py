"""Compile actual merge/cache code extracted from remoter.cpp with a fake message input.
This isolates link expiry; it does not simulate GPIO or RTOS scheduling.
"""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--cxx', required=True)
p.add_argument('--zig', action='store_true')
p.add_argument('--build-dir', type=Path, required=True)
a = p.parse_args()
root = Path(__file__).resolve().parents[2]
source = (root / 'pnx_modules/remoter/src/remoter.cpp').read_text(encoding='utf-8')
cache = source[source.index('struct source_cache'):source.index('} // namespace')]
start = source.index('            ps2_state ps2_msg{};')
end = source.index('\n        }', start)
merge = source[start:end]
program = r'''#include <cstdint>
#include <cassert>
#include <cstdio>
using ULONG = std::uint32_t;
enum class source {none, dr16, vt03, ps2, ps2_uart};
struct state { bool offline = true; unsigned ps2_buttons = 0; };
struct ps2_state { state data{}; bool frame_valid = false; std::uint32_t last_signal_tick = 0; };
namespace types { enum class status {ok}; }
ps2_state incoming;
ULONG now;
ULONG tx_time_get() { return now; }
namespace msg { types::status read(int, ps2_state& out) { out = incoming; return types::status::ok; } }
struct service { int ps2_sub_ = 0; };
''' + cache + r'''
void receive(source_cache& ps2) {
    service instance;
    auto* self = &instance;
''' + merge + r'''
}
int main() {
    source_cache ps2, empty;
    auto selected = [&] { return choose_source(source::ps2, empty, empty, ps2, empty, now, 120); };
    // Boot with repeated invalid frames must never become online.
    for (now = 0; now < 200; ++now) { receive(ps2); assert(selected() == source::none); }
    // Successful frame with held buttons, followed by failed polls carrying stale data.
    incoming = {{false, 0x100}, true, 200};
    now = 200; receive(ps2); assert(selected() == source::ps2);
    incoming.frame_valid = false;
    for (now = 201; now <= 500; ++now) {
        receive(ps2);
        assert(ps2.tick == 200);
        assert(selected() == (now <= 320 ? source::ps2 : source::none));
    }
    // A delayed valid message must use acquisition time, not consumption time.
    incoming.frame_valid = true;
    now = 600; receive(ps2); assert(selected() == source::none);
    incoming.last_signal_tick = 600;
    receive(ps2); assert(selected() == source::ps2);
    // Tick wraparound preserves expiry behavior.
    incoming.last_signal_tick = 0xfffffff0U;
    now = incoming.last_signal_tick; receive(ps2);
    incoming.frame_valid = false;
    now = 0x68U; receive(ps2); assert(selected() == source::ps2);
    now = 0x69U; receive(ps2); assert(selected() == source::none);
    std::puts("PASS: invalid boot, stale failed polls, acquisition timestamp, reconnect, tick wrap");
}
'''
a.build_dir.mkdir(parents=True, exist_ok=True)
cpp = a.build_dir / 'ps2_offline.cpp'
exe = a.build_dir / 'ps2_offline.exe'
cpp.write_text(program, encoding='utf-8')
compiler = [a.cxx, 'c++'] if a.zig else [a.cxx]
subprocess.run(compiler + ['-std=c++17', '-Wall', '-Wextra', str(cpp), '-o', str(exe)], check=True)
subprocess.run([str(exe.resolve())], check=True)
