#include "config.hpp"
#if WBR_APPLICATION
#include "robot.hpp"
#endif

extern "C" void diagnose_start();

extern "C" void app_start()
{
    if constexpr (params::test::auto_run_on_boot)
    {
        diagnose_start();
    }
    else
    {
#if WBR_APPLICATION
        (void)robot::application::start();
#endif
    }
}

