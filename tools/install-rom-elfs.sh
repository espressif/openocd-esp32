#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later

set -euo pipefail

DEST=/opt/esp/tools/esp-rom-elfs
VER1=20240305
VER2=20241011
VER3=20260528
TAR=esp-rom-elfs-$VER3.tar.gz # VER3 covers the previous version
URL=https://github.com/espressif/esp-rom-elfs/releases/download/$VER3/$TAR

sudo mkdir -p "$DEST"/{$VER1,$VER2,$VER3}

cd "$DEST"
sudo wget -q -O "$TAR" "$URL"

for d in "$VER1" "$VER2" "$VER3"; do
  sudo tar -xzf "$TAR" -C "$d"
done

sudo rm "$TAR"
echo "Done – esp-rom-elf files are now in:"
echo "  $DEST/$VER1"
echo "  $DEST/$VER2"
echo "  $DEST/$VER3"
