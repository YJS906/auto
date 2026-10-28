#!/usr/bin/env bash
set -euo pipefail
AUTORACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AUTORACE_COMMIT=7ef355cd515e806b26f498d38bfe2abe4f3a9728
mkdir -p "$AUTORACE_ROOT/vendor"
touch "$AUTORACE_ROOT/vendor/COLCON_IGNORE"
if [ ! -d "$AUTORACE_ROOT/vendor/limo_pro/.git" ]; then
  git clone https://github.com/WeGo-Robotics/limo_pro.git "$AUTORACE_ROOT/vendor/limo_pro"
  git -C "$AUTORACE_ROOT/vendor/limo_pro" checkout --detach "$AUTORACE_COMMIT"
fi
test "$(git -C "$AUTORACE_ROOT/vendor/limo_pro" rev-parse HEAD)" = "$AUTORACE_COMMIT" || {
  echo "Existing vendor checkout differs from pinned commit; inspect it before changing it." >&2; exit 1;
}
echo "WEGO source revision verified: $AUTORACE_COMMIT"
