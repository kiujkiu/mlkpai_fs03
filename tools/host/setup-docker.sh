#!/usr/bin/env bash
#==============================================================================
# setup-docker.sh — 在 WSL(Ubuntu 26.04 resolute) 里装 Docker Engine 并配好当前用户
#
# 用法:   sudo bash setup-docker.sh
#         (装完**重开一个 shell** 或执行 newgrp docker, 组变更才对已有会话生效)
#
# 做四件事:
#   1. 从 Docker 官方 apt 源装 docker-ce + compose 插件
#   2. 把调用者(非 root 的那个用户)加入 docker 组
#   3. 启动 dockerd (本机无 systemd, 用 service; 顺带写好开机自启建议)
#   4. 跑 hello-world 自检
#
# ⚠ 已知坑: Docker 官方源尚无 Ubuntu 26.04 "resolute" 仓库, 直接用
#   $VERSION_CODENAME 会 404。故按 LTS 回退到 noble(24.04), 二进制兼容。
#==============================================================================
set -euo pipefail

log() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*"; }
die() { printf '\033[1;31m[fail] %s\033[0m\n' "$*" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || die "请用 sudo 执行: sudo bash $0"

# 谁在调用 (sudo 下 $USER 是 root, 要用 SUDO_USER)
TARGET_USER="${SUDO_USER:-}"
[ -n "$TARGET_USER" ] || die "取不到原用户名, 请显式指定: sudo TARGET_USER=<name> bash $0"
log "目标用户: $TARGET_USER"

#------------------------------------------------------------------ 1. 装 Docker
. /etc/os-release
CODENAME="${VERSION_CODENAME:-}"
# Docker 源支持的 Ubuntu 代号 (2026 年初): noble/jammy/focal 等; resolute 尚无
case "$CODENAME" in
    noble|jammy|focal|bookworm|bullseye) REPO_CODENAME="$CODENAME" ;;
    *)  REPO_CODENAME="noble"
        warn "Docker 官方源没有 '$CODENAME' 仓库, 回退用 'noble' (24.04 LTS), 二进制兼容" ;;
esac
log "系统 $NAME $VERSION_ID ($CODENAME) → 使用 Docker 源代号: $REPO_CODENAME"

if command -v docker >/dev/null 2>&1; then
    warn "已存在 docker: $(docker --version 2>/dev/null || true) — 跳过安装步骤"
else
    log "安装依赖并添加 Docker 官方 apt 源"
    apt-get update -qq
    apt-get install -y -qq ca-certificates curl gnupg

    install -m 0755 -d /etc/apt/keyrings
    if [ ! -s /etc/apt/keyrings/docker.gpg ]; then
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
            | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    fi
    chmod a+r /etc/apt/keyrings/docker.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $REPO_CODENAME stable" \
        > /etc/apt/sources.list.d/docker.list

    log "安装 docker-ce / cli / containerd / buildx / compose"
    apt-get update -qq
    apt-get install -y -qq \
        docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

#--------------------------------------------------------------- 2. 用户加进组
log "把 $TARGET_USER 加入 docker 组"
groupadd -f docker
usermod -aG docker "$TARGET_USER"
echo "  当前 docker 组成员: $(getent group docker | cut -d: -f4)"

#------------------------------------------------------------------ 3. 起 daemon
log "启动 dockerd"
if [ "$(ps -p 1 -o comm= 2>/dev/null)" = "systemd" ]; then
    systemctl enable --now docker
else
    # WSL 无 systemd: 用 SysV service; 并提示如何做开机自启
    service docker start || true
    cat <<'EOS'

  [提示] 本机 PID 1 不是 systemd, dockerd 不会开机自启。二选一:
    a) 每次开 WSL 后手动: sudo service docker start
    b) 免密自启, 在 ~/.bashrc 末尾加:
         if ! pgrep -x dockerd >/dev/null; then sudo service docker start >/dev/null 2>&1; fi
       并执行 sudo visudo 追加一行(把 wanqi 换成你的用户名):
         wanqi ALL=(ALL) NOPASSWD: /usr/sbin/service docker *
EOS
fi

# 等 socket 就绪
for i in $(seq 1 20); do
    [ -S /var/run/docker.sock ] && break
    sleep 1
done
[ -S /var/run/docker.sock ] || die "dockerd 未起来 (/var/run/docker.sock 不存在), 看 /var/log/docker.log"

#-------------------------------------------------------------------- 4. 自检
log "自检: docker version"
docker version --format '  Server: {{.Server.Version}}  API: {{.Server.APIVersion}}' 2>/dev/null \
    || docker version | head -20

log "自检: hello-world"
if docker run --rm hello-world >/dev/null 2>&1; then
    echo "  ✅ hello-world 跑通"
else
    warn "hello-world 失败 — 可能是网络/镜像源问题, 但 daemon 本身已就绪"
fi

log "自检: compose 插件"
docker compose version 2>/dev/null | sed 's/^/  /' || warn "compose 插件不可用"

cat <<EOS

============================================================
 ✅ 完成

 🔴 现在必须**重开一个 shell** (或执行 newgrp docker), 否则
    $TARGET_USER 仍不在 docker 组内, 会报 permission denied。

 验证(重开 shell 后, 不带 sudo):
    docker run --rm hello-world

 然后跑 aibrain-app:
    cd /mnt/d/claude_workspace/pov3d/aibrain-app
    docker compose up --build
    # 浏览器打开 http://localhost:9070

 ⚠ 提速建议: 仓库在 /mnt/d (Windows 文件系统, 9P 跨界访问很慢),
    docker build 会明显拖慢。建议复制到 WSL 原生路径再构建:
       cp -r /mnt/d/claude_workspace/pov3d/aibrain-app ~/aibrain-app
       cd ~/aibrain-app && docker compose up --build
============================================================
EOS
