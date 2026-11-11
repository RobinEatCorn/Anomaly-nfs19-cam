#!/usr/bin/env bash
# Release gate for the NFS 2015 Chase Cam package.
#
# This is the offline half of PLAN-anomaly-2.7.md task group 4. It verifies the
# manifest, package layout, ZIP root, DLL imports/exports and model fixture. The
# Windows-only host tools are run when they are on PATH; otherwise they are
# reported as blocked instead of being silently treated as passed.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build.mingw"
ZIP_PATH=""
ALLOW_MISSING_HOST_TOOLS=0
RUN_BUILD=0
SDK_ARG="${ANOMALY_SDK_DIR:-}"
PACKAGE_NAME="Nfs2015ChaseCam"
PACKAGE_ID="robineatcorn.nfs2015-chase-cam"

c_reset=$'\033[0m'; c_info=$'\033[36m'; c_ok=$'\033[32m'; c_warn=$'\033[33m'; c_err=$'\033[31m'
if [[ ! -t 1 ]]; then c_reset=""; c_info=""; c_ok=""; c_warn=""; c_err=""; fi
info() { printf '%s==>%s %s\n' "$c_info" "$c_reset" "$*"; }
ok()   { printf '%s==>%s %s\n' "$c_ok" "$c_reset" "$*"; }
warn() { printf '%swarn:%s %s\n' "$c_warn" "$c_reset" "$*" >&2; }
fail() { printf '%serror:%s %s\n' "$c_err" "$c_reset" "$*" >&2; FAILED=$((FAILED + 1)); }
blocked() { printf '%sblocked:%s %s\n' "$c_warn" "$c_reset" "$*"; BLOCKED=$((BLOCKED + 1)); }

usage() {
    cat <<'EOF'
用法: ./verify-release.sh [选项]

默认检查 build.mingw/package 和 dist/Nfs2015ChaseCam-<version>.zip。
发布前应先生成产物: ./build-mingw.sh --sdk <SDK> --zip

选项
  -b DIR, --build-dir DIR   构建树（默认 ./build.mingw）
  -z FILE, --zip FILE       指定 ZIP（默认按 manifest.json 版本推得）
  -s DIR, --sdk DIR         --build 时传给 build-mingw.sh 的 SDK
      --build               产物不存在时自动构建并打包
      --allow-missing-host-tools
                            缺少 anomaly-plugin / anomaly-test-host 时仍返回 0，
                            但汇总会明确标记为 blocked
  -h, --help                显示帮助

退出码: 0=全部可执行检查通过；1=检查失败；2=有宿主工具阻塞且未加
        --allow-missing-host-tools。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -b|--build-dir) [[ $# -ge 2 ]] || { usage >&2; exit 1; }; BUILD_DIR="$2"; shift ;;
        -z|--zip)       [[ $# -ge 2 ]] || { usage >&2; exit 1; }; ZIP_PATH="$2"; shift ;;
        -s|--sdk)       [[ $# -ge 2 ]] || { usage >&2; exit 1; }; SDK_ARG="$2"; shift ;;
        --build)        RUN_BUILD=1 ;;
        --allow-missing-host-tools) ALLOW_MISSING_HOST_TOOLS=1 ;;
        -h|--help)      usage; exit 0 ;;
        *) usage >&2; exit 1 ;;
    esac
    shift
done

cd -- "$SCRIPT_DIR"
[[ -f manifest.json ]] || { printf 'error: 不在插件仓库根目录\n' >&2; exit 1; }

version="$(python3 - <<'PY'
import json
print(json.load(open("manifest.json", encoding="utf-8"))["version"])
PY
)"
[[ -n "$version" ]] || { printf 'error: manifest.json 缺少 version\n' >&2; exit 1; }
PACKAGE_DIR="$BUILD_DIR/package/$PACKAGE_ID"
[[ -n "$ZIP_PATH" ]] || ZIP_PATH="$SCRIPT_DIR/dist/$PACKAGE_NAME-$version.zip"

if [[ ! -f "$PACKAGE_DIR/plugin.dll" || ! -f "$PACKAGE_DIR/manifest.json" ||
      ! -f "$ZIP_PATH" ]]; then
    if [[ $RUN_BUILD -eq 1 ]]; then
        info "产物不存在，运行 build-mingw.sh"
        args=(--zip)
        [[ -n "$SDK_ARG" ]] && args+=(--sdk "$SDK_ARG")
        "$SCRIPT_DIR/build-mingw.sh" "${args[@]}"
    else
        printf 'error: 缺少 %s\n先运行 ./build-mingw.sh --sdk <SDK> --zip，或加 --build\n' "$PACKAGE_DIR" >&2
        exit 1
    fi
fi

FAILED=0
BLOCKED=0

# ---------------------------------------------------------------- manifest
info "manifest / 版本 / ID"
if python3 - "$PACKAGE_DIR" "$ZIP_PATH" <<'PY'
import json
import pathlib
import sys
import zipfile

package = pathlib.Path(sys.argv[1])
zip_path = pathlib.Path(sys.argv[2])
root = pathlib.Path.cwd()
manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
master = json.loads((root / "pluginmaster.json").read_text(encoding="utf-8"))
expected_id = "robineatcorn.nfs2015-chase-cam"
expected_version = manifest["version"]
errors = []

if manifest.get("id") != expected_id:
    errors.append(f"manifest id is {manifest.get('id')!r}, expected {expected_id!r}")
if manifest.get("schemaVersion") != 2:
    errors.append(f"manifest schemaVersion is {manifest.get('schemaVersion')!r}, expected 2")
if manifest.get("entry") != "plugin.dll":
    errors.append("manifest entry must be plugin.dll")
services = [service.get("id") for service in manifest.get("services", [])]
if "anomaly.nte.vehicle" in services:
    errors.append("manifest still lists removed service anomaly.nte.vehicle")
capabilities = manifest.get("capabilities", [])
if "nte-vehicle" in capabilities:
    errors.append("manifest still lists removed capability nte-vehicle")
if not isinstance(master, list) or not master:
    errors.append("pluginmaster.json must be a non-empty array")
else:
    entries = [entry for entry in master if entry.get("InternalName") == expected_id]
    if not entries:
        errors.append("pluginmaster.json has no entry for the new id")
    for entry in entries:
        if entry.get("Version") != expected_version:
            errors.append(
                f"pluginmaster Version is {entry.get('Version')!r}, expected {expected_version!r}"
            )
        for key in ("DownloadLinkInstall", "DownloadLinkUpdate"):
            link = entry.get(key, "")
            if f"/v{expected_version}/" not in link and f"/download/v{expected_version}/" not in link:
                errors.append(f"pluginmaster {key} does not point at v{expected_version}")
        if entry.get("Name") != manifest.get("name"):
            errors.append("pluginmaster Name differs from manifest name")
        if entry.get("Author") != manifest.get("author"):
            errors.append("pluginmaster Author differs from manifest author")

schema_path = root.parent / "Anomaly" / "schemas" / "plugin-manifest.schema.json"
if schema_path.exists():
    try:
        import jsonschema  # type: ignore
        jsonschema.validate(manifest, json.loads(schema_path.read_text(encoding="utf-8")))
    except ImportError:
        print("warn: jsonschema 不可用，跳过 schema 深度校验")
    except Exception as error:
        errors.append(f"manifest schema validation failed: {error}")

required_package_files = {"manifest.json", "plugin.dll"}
package_files = {
    path.relative_to(package).as_posix()
    for path in package.rglob("*")
    if path.is_file()
}
missing = required_package_files - package_files
if missing:
    errors.append(f"package missing: {sorted(missing)}")
if "locales/zh-CN.json" not in package_files:
    errors.append("package missing locales/zh-CN.json")
if (package / "manifest.json").read_bytes() != (root / "manifest.json").read_bytes():
    errors.append("package manifest.json differs from repository manifest.json")

if not zip_path.is_file():
    errors.append(f"ZIP missing: {zip_path}")
else:
    with zipfile.ZipFile(zip_path) as archive:
        names = set(archive.namelist())
        if required_package_files - names:
            errors.append(f"ZIP root missing: {sorted(required_package_files - names)}")
        if "locales/zh-CN.json" not in names:
            errors.append("ZIP missing locales/zh-CN.json")
        if any(name.startswith("/") or name.startswith("../") for name in names):
            errors.append("ZIP contains an unsafe path")
        for name in sorted(package_files):
            if archive.read(name) != (package / name).read_bytes():
                errors.append(f"ZIP entry differs from package: {name}")
        if "manifest.json" in names:
            zipped_manifest = json.loads(archive.read("manifest.json"))
            if zipped_manifest.get("id") != expected_id:
                errors.append("ZIP manifest id is wrong")
            if zipped_manifest.get("version") != expected_version:
                errors.append("ZIP manifest version is wrong")

if errors:
    for error in errors:
        print(f"FAIL {error}")
    raise SystemExit(1)
print(f"OK manifest {expected_id} {expected_version}")
print(f"OK package {len(package_files)} files; ZIP {zip_path.name}")
PY
then
    ok "manifest / package / ZIP 校验通过"
else
    FAILED=$((FAILED + 1))
fi

# ---------------------------------------------------------------- source constants
info "源码常量"
# The sources are split across plugin.cpp and src/: the plugin ID lives in
# src/constants.hpp, the version string still sits in the entry point.
source_files=(plugin.cpp)
shopt -s nullglob
source_files+=(src/*.hpp src/*.cpp)
shopt -u nullglob
if grep -q "\"$PACKAGE_ID\"" "${source_files[@]}"; then
    ok "源码使用新插件 ID"
else
    fail "源码未使用 $PACKAGE_ID"
fi
if grep -q "\"$version\"" "${source_files[@]}"; then
    ok "源码版本常量与 $version 一致"
else
    fail "源码版本常量不是 $version"
fi
if grep -q 'anomaly\.nte\.vehicle\|nte-vehicle' manifest.json; then
    fail "manifest.json 仍包含已删除的 vehicle 服务/capability"
else
    ok "manifest.json 不再引用已删除的 vehicle 服务/capability"
fi

# ---------------------------------------------------------------- model fixture
info "离线模型验收"
if [[ -x "$BUILD_DIR/nfs2015_chase_cam_test.exe" || -f "$BUILD_DIR/nfs2015_chase_cam_test.exe" ]]; then
    if ctest --test-dir "$BUILD_DIR" --output-on-failure; then
        ok "ctest 通过"
    else
        fail "ctest 失败"
    fi
else
    fail "找不到 $BUILD_DIR/nfs2015_chase_cam_test.exe"
fi

# ---------------------------------------------------------------- DLL
info "DLL 导入/导出"
dll="$PACKAGE_DIR/plugin.dll"
objdump_bin="$(command -v x86_64-w64-mingw32-objdump || true)"
if [[ -n "$objdump_bin" ]]; then
    symbols="$("$objdump_bin" -p "$dll")"
    imports="$(sed -n 's/^[[:space:]]*DLL Name: //p' <<<"$symbols" | sort -u)"
    extra="$(grep -viE '^(KERNEL32|msvcrt)\.dll$' <<<"$imports" || true)"
    if [[ -n "$extra" ]]; then
        fail "plugin.dll 导入了非系统模块: $(tr '\n' ' ' <<<"$extra")"
    else
        ok "plugin.dll 仅导入系统模块: $(tr '\n' ' ' <<<"$imports")"
    fi
    if grep -q 'AnomalyPluginEntryV1' <<<"$symbols"; then
        ok "plugin.dll 导出 AnomalyPluginEntryV1"
    else
        fail "plugin.dll 未导出 AnomalyPluginEntryV1"
    fi
    printf '  sha256 %s\n' "$(sha256sum "$dll" | awk '{print $1}')"
    printf '  size   %s bytes\n' "$(stat -c '%s' "$dll")"
else
    blocked "找不到 x86_64-w64-mingw32-objdump，跳过 DLL 导入/导出校验"
fi

# ---------------------------------------------------------------- host tools
info "Windows 宿主工具"
host_package="$PACKAGE_DIR"
if command -v wslpath >/dev/null 2>&1; then
    host_package="$(wslpath -w "$PACKAGE_DIR")"
fi
if command -v anomaly-plugin >/dev/null 2>&1; then
    if anomaly-plugin validate "$host_package"; then
        ok "anomaly-plugin validate 通过"
    else
        fail "anomaly-plugin validate 失败"
    fi
else
    blocked "anomaly-plugin 不在 PATH（Windows 侧运行 validate）"
fi
if command -v anomaly-test-host >/dev/null 2>&1; then
    if anomaly-test-host --plugin "$host_package" --reload 10 --ticks 60; then
        ok "anomaly-test-host --reload 10 --ticks 60 通过"
    else
        fail "anomaly-test-host 失败"
    fi
else
    blocked "anomaly-test-host 不在 PATH（需在 Windows 侧运行）"
fi

printf '\n%s汇总%s\n' "$c_info" "$c_reset"
printf '  failed:  %d\n' "$FAILED"
printf '  blocked: %d\n' "$BLOCKED"
printf '  ZIP:     %s\n' "$ZIP_PATH"
if [[ -f "$ZIP_PATH" ]]; then
    printf '  sha256:  %s\n' "$(sha256sum "$ZIP_PATH" | awk '{print $1}')"
fi
if [[ $FAILED -gt 0 ]]; then
    exit 1
fi
if [[ $BLOCKED -gt 0 && $ALLOW_MISSING_HOST_TOOLS -eq 0 ]]; then
    printf '\n宿主工具阻塞：发布前必须在 Windows 侧补跑；或加 --allow-missing-host-tools 仅做本地检查。\n' >&2
    exit 2
fi
if [[ $BLOCKED -gt 0 ]]; then
    warn "本地检查通过；宿主工具仍标记为 blocked，不能视为完整发布验收"
fi
ok "离线发布检查通过"
