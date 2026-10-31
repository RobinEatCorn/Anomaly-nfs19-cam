#!/usr/bin/env bash
# build-mingw.sh — 用 MinGW-w64 交叉编译 NFS 2015 Chase Cam 插件
#
# 插件只跨 Anomaly 的纯 C ABI，SDK 包里的 Anomaly::sdk 也只是 header-only 的
# INTERFACE target，所以可以用 MinGW-w64 构建；Runtime 和官方工具仍然需要
# MSVC/VS2022。脚本负责：定位已安装的 AnomalySDK、用仓库内的 mingw.cmake 配置
# 并编译、校验 plugin.dll 的导入表（只允许系统模块）和导出符号。
#
# 用法: ./build-mingw.sh [选项]
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$SCRIPT_DIR"
BUILD_DIR="$SCRIPT_DIR/build.mingw"
BUILD_TYPE="RelWithDebInfo"
SDK_ARG="${ANOMALY_SDK_DIR:-}"
PARALLEL="$(nproc 2>/dev/null || echo 4)"
FRESH=0
CLEAN=0
CONFIGURE_ONLY=0
BUILD_TESTS=ON
RUN_TESTS=0
VERIFY=1
INSTALL_PREFIX=""
MAKE_ZIP=0
PACKAGE_NAME="Nfs2015ChaseCam"

c_reset=$'\033[0m'; c_info=$'\033[36m'; c_ok=$'\033[32m'; c_warn=$'\033[33m'; c_err=$'\033[31m'
if [[ ! -t 1 ]]; then c_reset=""; c_info=""; c_ok=""; c_warn=""; c_err=""; fi

info() { printf '%s==>%s %s\n' "$c_info" "$c_reset" "$*"; }
ok()   { printf '%s==>%s %s\n' "$c_ok" "$c_reset" "$*"; }
warn() { printf '%swarn:%s %s\n' "$c_warn" "$c_reset" "$*" >&2; }
die()  { printf '%serror:%s %s\n' "$c_err" "$c_reset" "$*" >&2; exit 1; }

usage() {
    cat <<'EOF'
用法: ./build-mingw.sh [选项]

默认在 ./build.mingw 配置并编译，产物是
./build.mingw/package/Nfs2015ChaseCam/{plugin.dll,manifest.json,locales}。

选项
  -s DIR, --sdk DIR       AnomalySDK 安装前缀，或它的 lib/cmake/AnomalySDK 目录
                          默认依次尝试 ../Anomaly/.build/windows-vs2022/package/sdk、
                          ../build-ninja/package/sdk、../Anomaly/build/package/sdk
  -b DIR, --build-dir DIR 构建树（默认 ./build.mingw）
  -c TYPE, --build-type   CMAKE_BUILD_TYPE（默认 RelWithDebInfo）
  -j N, --parallel N      并行度（默认 CPU 核数；-j 不带数字等于默认值）
      --fresh             删除构建树后重新 configure（SDK 换路径时用）
      --clean             先 clean 再全量重编（保留构建树与缓存）
      --configure-only    只 configure，不编译
      --no-tests          不构建相机模型验收夹具（nfs2015_chase_cam_test.exe）
      --run-tests         用 wine 跑验收夹具；没有 wine 时报错
      --no-verify         跳过 plugin.dll 的导入/导出校验
      --install DIR       构建后安装到游戏根目录（<DIR>/Anomaly/plugins/Nfs2015ChaseCam）
                          WSL 里可以直接写 Windows 路径，如 D:/Games/NTE
      --zip               打包 dist/Nfs2015ChaseCam-<version>.zip（manifest.json 在 ZIP 根）
  -h, --help              显示本帮助

环境变量
  ANOMALY_SDK_DIR   同 --sdk
  MINGW_CXX / MINGW_CC / MINGW_RC
                    自定义工具链二进制，交给 mingw.cmake 使用

依赖
  cmake >= 3.22、x86_64-w64-mingw32-g++-posix（Debian/Ubuntu:
  g++-mingw-w64-x86-64-posix）、已安装 SDK 组件：
  cmake --install <anomaly-build> --prefix <sdk> --component SDK

示例
  ./build-mingw.sh                                  # 日常构建
  ./build-mingw.sh --fresh --sdk ~/sdk              # 换 SDK
  ./build-mingw.sh --zip --install D:/Games/NTE     # 出包并安装到游戏目录
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--sdk)          [[ $# -ge 2 ]] || die "$1 需要一个参数"; SDK_ARG="$2"; shift ;;
        -b|--build-dir)    [[ $# -ge 2 ]] || die "$1 需要一个参数"; BUILD_DIR="$2"; shift ;;
        -c|--build-type)   [[ $# -ge 2 ]] || die "$1 需要一个参数"; BUILD_TYPE="$2"; shift ;;
        -j|--parallel)
            if [[ $# -ge 2 && "$2" =~ ^[0-9]+$ ]]; then PARALLEL="$2"; shift
            else PARALLEL="$(nproc 2>/dev/null || echo 4)"; fi ;;
        --fresh)           FRESH=1 ;;
        --clean)           CLEAN=1 ;;
        --configure-only)  CONFIGURE_ONLY=1 ;;
        --no-tests)        BUILD_TESTS=OFF ;;
        --run-tests)       RUN_TESTS=1 ;;
        --no-verify)       VERIFY=0 ;;
        --install)         [[ $# -ge 2 ]] || die "--install 需要一个参数"; INSTALL_PREFIX="$2"; shift ;;
        --zip)             MAKE_ZIP=1 ;;
        -h|--help)         usage; exit 0 ;;
        *) die "未知参数: $1（--help 查看用法）" ;;
    esac
    shift
done

# ---------------------------------------------------------------- 路径与工具
# WSL 里允许传 Windows 路径（--sdk C:/SDK、--install D:/Games/NTE），
# 本机的 Linux cmake 只认 /mnt/... 形式。
to_unix_path() {
    local path="$1"
    if [[ "$path" =~ ^[A-Za-z]:[\\/] ]] && command -v wslpath >/dev/null 2>&1; then
        wslpath -u "$path"
    else
        printf '%s\n' "$path"
    fi
}

[[ -f "$SOURCE_DIR/CMakeLists.txt" ]] || die "$SOURCE_DIR 不是插件工程（缺少 CMakeLists.txt）"
mkdir -p "$BUILD_DIR"
BUILD_DIR="$(cd -- "$BUILD_DIR" && pwd)"
SOURCE_DIR="$(cd -- "$SOURCE_DIR" && pwd)"
[[ "$BUILD_DIR" != "$SOURCE_DIR" ]] || die "构建树不能是源码目录"

command -v cmake >/dev/null 2>&1 || die "找不到 cmake"
cmake_version="$(cmake --version | awk 'NR == 1 { print $3 }')"
# CMakeLists.txt 要求 3.22；Ubuntu 20.04 自带的 3.16 不满足
[[ "$(printf '%s\n3.22\n' "${cmake_version%%-*}" | sort -V | head -1)" == "3.22" ]] ||
    die "CMake $cmake_version 低于项目要求的 3.22"

cxx="$(command -v x86_64-w64-mingw32-g++-posix || command -v x86_64-w64-mingw32-g++ || true)"
[[ -n "$cxx" ]] ||
    die "找不到 x86_64-w64-mingw32-g++，请安装 g++-mingw-w64-x86-64-posix"
toolchain_info="$("$cxx" -v 2>&1)"
if ! grep -q 'Thread model: posix' <<<"$toolchain_info"; then
    die "$cxx 用的是 win32 线程模型，libstdc++ 没有 std::mutex；请安装 g++-mingw-w64-x86-64-posix"
fi
objdump="$(command -v x86_64-w64-mingw32-objdump || true)"

# wine 只在 --run-tests 时才需要，提前检查，避免编译完才发现跑不了
runner=""
if [[ $RUN_TESTS -eq 1 ]]; then
    runner="$(command -v wine64 || command -v wine || true)"
    [[ -n "$runner" ]] ||
        die "--run-tests 需要 wine；也可以在 Windows 上直接跑 ctest --test-dir $BUILD_DIR"
fi

# ---------------------------------------------------------------- AnomalySDK
# 只接受真正安装过的 SDK 前缀：find_package 需要 Config + Targets + AnomalyPlugin
# 三个文件，Anomaly 的构建树根目录只有 Config，头文件和 export 文件不在那里。
sdk_from() {
    local candidate="${1%/}"
    [[ -n "$candidate" ]] || return 1
    candidate="$(to_unix_path "$candidate")"
    if [[ -f "$candidate/lib/cmake/AnomalySDK/AnomalySDKConfig.cmake" ]]; then
        printf '%s\n' "$candidate/lib/cmake/AnomalySDK"
    elif [[ -f "$candidate/AnomalySDKConfig.cmake" ]]; then
        printf '%s\n' "$candidate"
    else
        return 1
    fi
}

sdk_cmake_dir=""
if [[ -n "$SDK_ARG" ]]; then
    sdk_cmake_dir="$(sdk_from "$SDK_ARG")" ||
        die "--sdk $SDK_ARG 里没有 AnomalySDKConfig.cmake（需要已安装的 SDK 前缀）"
else
    for candidate in \
        "$SOURCE_DIR/../Anomaly/.build/windows-vs2022/package/sdk" \
        "$SOURCE_DIR/../build-ninja/package/sdk" \
        "$SOURCE_DIR/../Anomaly/build/package/sdk"; do
        if sdk_cmake_dir="$(sdk_from "$candidate")"; then break; fi
        sdk_cmake_dir=""
    done
    [[ -n "$sdk_cmake_dir" ]] || die "找不到 AnomalySDK：
  cmake --install <anomaly-build> --prefix <sdk> --component SDK
  ./build-mingw.sh --sdk <sdk>
或用 ANOMALY_SDK_DIR 指定。"
fi
for required in AnomalySDKTargets.cmake AnomalyPlugin.cmake; do
    [[ -f "$sdk_cmake_dir/$required" ]] ||
        die "$sdk_cmake_dir 不完整（缺少 $required），请重新安装 SDK 组件"
done
sdk_cmake_dir="$(cd -- "$sdk_cmake_dir" && pwd)"

# ---------------------------------------------------------------- configure
info "源码        $SOURCE_DIR"
info "构建树      $BUILD_DIR"
info "SDK         ${sdk_cmake_dir%/lib/cmake/AnomalySDK}"
info "工具链      $cxx"
info "构建类型    $BUILD_TYPE（$PARALLEL 路并行）"

if [[ $FRESH -eq 1 ]]; then
    info "--fresh: 删除 $BUILD_DIR"
    rm -rf -- "$BUILD_DIR"
    mkdir -p -- "$BUILD_DIR"
fi

configure_args=(
    -S "$SOURCE_DIR"
    -B "$BUILD_DIR"
    -DCMAKE_TOOLCHAIN_FILE="$SOURCE_DIR/mingw.cmake"
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    -DAnomalySDK_DIR="$sdk_cmake_dir"
    -DNFS2015_CHASE_CAM_BUILD_TESTS="$BUILD_TESTS"
)

info "configure"
cmake "${configure_args[@]}"

if [[ $CONFIGURE_ONLY -eq 1 ]]; then
    ok "configure 完成（未编译）"
    exit 0
fi

# ---------------------------------------------------------------- build
build_args=(--build "$BUILD_DIR" --parallel "$PARALLEL")
if [[ $CLEAN -eq 1 ]]; then build_args+=(--clean-first); fi
if [[ $CLEAN -eq 1 ]]; then info "build (clean-first)"; else info "build"; fi
cmake "${build_args[@]}"
ok "编译完成"

dll="$BUILD_DIR/package/$PACKAGE_NAME/plugin.dll"
package_dir="$BUILD_DIR/package/$PACKAGE_NAME"
[[ -f "$dll" ]] || die "编译成功但找不到 $dll"

# ---------------------------------------------------------------- 校验
if [[ $VERIFY -eq 1 ]]; then
    if [[ -z "$objdump" ]]; then
        warn "找不到 x86_64-w64-mingw32-objdump，跳过导入/导出校验"
    else
        imports="$("$objdump" -p "$dll" | sed -n 's/^[[:space:]]*DLL Name: //p' | sort -u)"
        # 插件加载器只放行系统 DLL 或包内自带的库；-static 让 libstdc++/libgcc/
        # libwinpthread 不再出现在导入表里，这里把回归挡在提交之前。
        extra="$(grep -viE '^(KERNEL32|msvcrt)\.dll$' <<<"$imports" || true)"
        if [[ -n "$extra" ]]; then
            warn "plugin.dll 导入了非系统模块（需要随包分发或静态链接）：$(tr '\n' ' ' <<<"$extra")"
        fi
        # 先落成变量再 grep：直接 `objdump | grep -q` 在 pipefail 下会因 SIGPIPE 误报
        symbols="$("$objdump" -p "$dll")"
        grep -q 'AnomalyPluginEntryV1' <<<"$symbols" ||
            die "plugin.dll 没有导出 AnomalyPluginEntryV1"
        ok "校验通过：$(tr '\n' ' ' <<<"$imports")｜导出 AnomalyPluginEntryV1"
    fi
fi

# ---------------------------------------------------------------- 安装 / 打包
if [[ -n "$INSTALL_PREFIX" ]]; then
    install_prefix="$(to_unix_path "$INSTALL_PREFIX")"
    info "install -> $install_prefix/Anomaly/plugins/$PACKAGE_NAME"
    cmake --install "$BUILD_DIR" --prefix "$install_prefix" --component GameRuntime
fi

if [[ $MAKE_ZIP -eq 1 ]]; then
    command -v zip >/dev/null 2>&1 || die "--zip 需要 zip 命令"
    version="$(awk -F'"' '/"version"[[:space:]]*:/ { print $4; exit }' \
        "$SOURCE_DIR/manifest.json")"
    [[ -n "$version" ]] || version="dev"
    mkdir -p "$SOURCE_DIR/dist"
    zip_path="$SOURCE_DIR/dist/$PACKAGE_NAME-$version.zip"
    rm -f -- "$zip_path"
    # ZIP 根目录必须直接包含 manifest.json
    (cd -- "$package_dir" && zip -qr "$zip_path" .)
    ok "打包 $zip_path"
fi

if [[ $RUN_TESTS -eq 1 ]]; then
    info "验收夹具"
    "$runner" "$BUILD_DIR/nfs2015_chase_cam_test.exe"
fi

# ---------------------------------------------------------------- 结果
size="$(du -h "$dll" | cut -f1)"
ok "完成：$package_dir"
printf '  plugin.dll  %s，PE32+ x86-64\n' "$size"
printf '  验收夹具    %s\n' "$BUILD_DIR/nfs2015_chase_cam_test.exe"
cat <<EOF

下一步（Windows 侧）
  ctest --test-dir $BUILD_DIR --output-on-failure
  anomaly-plugin validate $package_dir
  anomaly-test-host --plugin $package_dir --reload 10 --ticks 60
EOF
