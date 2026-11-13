#!/usr/bin/env bash
# Run inside the Linux ROS container. Installs only the patched engine and base ABI library.
set -euo pipefail
prefix="${1:?usage: build.sh INSTALL_PREFIX}"
mkdir -p "$prefix"
prefix="$(cd "$prefix" && pwd)"
script_dir="$(cd "$(dirname "$0")" && pwd)"
revision=189471c86fa06ffad9c9bdb199f6928eb2fc02f3
fingerprint="$revision:$(sha256sum "$script_dir/patch.py" | cut -d' ' -f1):origin-rpath-v1"
if [[ -f "$prefix/version.txt" && "$(cat "$prefix/version.txt")" == "$fingerprint" && -f "$prefix/lib/libgz-physics-ringquad-bullet-plugin.so" ]]; then
  echo 'RingQuad physics engine is current.'
  exit 0
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
work="$(mktemp -d /tmp/ringquad-physics.XXXXXX)"
trap 'rm -rf "$work"' EXIT
git clone --quiet --depth 1 --branch gz-physics7_7.8.0 https://github.com/gazebosim/gz-physics.git "$work/source"
[[ "$(git -C "$work/source" rev-parse HEAD)" == "$revision" ]]
python3 "$script_dir/patch.py" "$work/source"
cmake -S "$work/source" -B "$work/build" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON
cmake --build "$work/build" --target gz-physics7-bullet-featherstone-plugin -j "${RINGQUAD_BUILD_JOBS:-4}"
mkdir -p "$prefix/lib"
cp -a "$work"/build/lib/libgz-physics7.so* "$work"/build/lib/libgz-physics7-bullet-featherstone-plugin.so* "$prefix/lib/"
ln -sfn libgz-physics7-bullet-featherstone-plugin.so "$prefix/lib/libgz-physics-ringquad-bullet-plugin.so"
printf '%s\n' "$fingerprint" > "$prefix/version.txt"
echo "Installed patched RingQuad engine: $prefix/lib"
