/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "tasks/task_cli.hpp"

#include "cli/cli.hpp"
#include "cli/settings.hpp"
#include "comm/stream_group.hpp"
#include "config/globals.hpp"
#include "util/log.h"

namespace task {

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void Cli::Run() noexcept {
  log_raw("USB config started");
  log_raw("CATS is now ready to receive commands...");

  // Release-visible canard warnings (the boot log shows them only in CATS_DEV builds).
  cli_print_canard_boot_warnings();
  cli_enter();
  while (true) {
    if (stream_length(USB_SG.in) > 0) {
      cli_process();
    }

    osDelay(10);
  }
}

}  // namespace task
