# ~/.bashrc — clean & robust dev setup (ROS1/ROS2 + Conda)
# =========================================================

# 0) Non-interactive shells quit early
[[ $- != *i* ]] && return

# --------------------------
# 1) Basic history & shell
# --------------------------
HISTCONTROL=ignoreboth
HISTSIZE=10000
HISTFILESIZE=20000
shopt -s histappend
shopt -s checkwinsize
# 可选：容错 cd，支持 ** 递归通配
shopt -s cdspell
shopt -s globstar

# 将本次命令即时写入历史文件
PROMPT_COMMAND="history -a;${PROMPT_COMMAND}"

# --------------------------
# 2) Colors, aliases, utils
# --------------------------
if command -v dircolors >/dev/null 2>&1; then
  test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
fi

alias ls='ls --color=auto'
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'

# 安全别名（可按需注释）
alias rm='rm -i'
alias cp='cp -i'
alias mv='mv -i'

alias grep='grep --color=auto'
alias egrep='egrep --color=auto'
alias fgrep='fgrep --color=auto'

# 常用系统信息
alias df='df -h'
alias du='du -h'
alias free='free -m'

# --------------------------
# 3) Python / pip hygiene
# --------------------------
alias pip='python -m pip'
export PYTHONNOUSERSITE=1

# --------------------------
# 4) Conda init (keep your path)
# --------------------------
__conda_setup="$('/home/zuoyangjkpi/miniconda3/bin/conda' 'shell.bash' 'hook' 2>/dev/null)"
if [ $? -eq 0 ]; then
  eval "$__conda_setup"
# else
#   if [ -f "/home/zuoyangjkpi/miniconda3/etc/profile.d/conda.sh" ]; then
# # . "/home/zuoyangjkpi/miniconda3/etc/profile.d/conda.sh"  # commented out by conda initialize
#   else
# # export PATH="/home/zuoyangjkpi/miniconda3/bin:$PATH"  # commented out by conda initialize
#   fi
fi
unset __conda_setup

# --------------------------
# 5) Prompt (shows conda env safely)
# --------------------------
# 终端标题栏：user@host: cwd
case "$TERM" in
  xterm*|rxvt*)
    PS1="\[\e]0;\u@\h: \w\a\]"
    ;;
  *)
    PS1=""
    ;;
esac
# 彩色主提示 + 显示 Conda 环境（由 CONDA_PROMPT_MODIFIER 提供）
PS1+='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
# PS1+='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\] ${CONDA_PROMPT_MODIFIER:-${CONDA_DEFAULT_ENV:+(${CONDA_DEFAULT_ENV}) }}\$ '


# --------------------------
# 6) Bash completion
# --------------------------
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# --------------------------
# 7) Dev toolchains
# --------------------------
# Rust (opengen 等会用到)
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
export PATH="$HOME/.cargo/bin:$PATH"

# colcon 自动补全（若存在）
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  . /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# --------------------------
# 8) Project paths
# --------------------------
export AIRSHIP_WS="$HOME/Edited_AirshipMPC"
export DRONE_WS="$HOME/AVIANS_ROS2"
export PX4_WS="$HOME/AVIANS_PX4"

# --------------------------
# 9) ROS helpers
# --------------------------
_use_ros_noetic() {
  if [ -r "/opt/ros/noetic/setup.bash" ]; then
    . /opt/ros/noetic/setup.bash
    echo "✅ ROS1 Noetic loaded."
  else
    echo "⚠️  /opt/ros/noetic/setup.bash 不存在（宿主机未安装？或请在 Docker 内使用）。"
  fi
}

_use_ros_jazzy() {
  if [ -r "/opt/ros/jazzy/setup.bash" ]; then
    . /opt/ros/jazzy/setup.bash
    echo "✅ ROS2 Jazzy loaded."
  else
    echo "⚠️  /opt/ros/jazzy/setup.bash 不存在（Jazzy 通常在 Ubuntu 24.04 / Docker 中）。"
  fi
}

# --------------------------
# 10) One-shot env setters
# --------------------------
# ROS1（Noetic）开发环境
setup_airship_ros1() {
  echo "🚀 Setting up AirshipMPC (ROS1 / Noetic)..."
  conda activate airship_env || { echo "❌ conda env 'airship_env' 不存在"; return 1; }

  # 保持干净
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH}"
  hash -r

  _use_ros_noetic

  echo "✅ Conda: $CONDA_PREFIX"
  echo "✅ Python: $(python -V)"

  # source 工作区（catkin 或 colcon）
  if [ -f "$AIRSHIP_WS/devel/setup.bash" ]; then
    . "$AIRSHIP_WS/devel/setup.bash"
    echo "✅ Sourced: devel/setup.bash (catkin)"
  elif [ -f "$AIRSHIP_WS/install/setup.bash" ]; then
    . "$AIRSHIP_WS/install/setup.bash"
    echo "✅ Sourced: install/setup.bash (colcon)"
  else
    echo "ℹ️ 未找到 devel/ 或 install/，请先编译（catkin_make / colcon build）。"
  fi
  echo "🎯 ROS1 environment ready."
}

# ROS2（Jazzy）开发环境
setup_airship_ros2() {
  echo "🚀 Setting up AirshipMPC (ROS2 / Jazzy)..."
  conda activate airship_ros2 || { echo "❌ conda env 'airship_ros2' 不存在"; return 1; }

  # 保持干净
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH}"
  hash -r

  # 千问 API Key 环境变量
  export DASHSCOPE_API_KEY="sk-5db2e04d96f24a4bb2ccad84af9cdb4b"
  export QWEN_API_KEY="$DASHSCOPE_API_KEY"

  _use_ros_jazzy

  echo "✅ Conda: $CONDA_PREFIX"
  echo "✅ Python: $(python -V)"

  if [ -f "$AIRSHIP_WS/install/setup.bash" ]; then
    . "$AIRSHIP_WS/install/setup.bash"
    echo "✅ Sourced: install/setup.bash (colcon)"
  else
    echo "ℹ️ 未找到 install/，请先执行：colcon build"
  fi
  echo "🎯 ROS2 environment ready."
}

# ROS2（Jazzy）开发环境
setup_drone_ros2() {
  echo "🚀 Setting up AVIANS (ROS2 / Jazzy)..."
  conda activate airship_ros2 || { echo "❌ conda env 'airship_ros2' 不存在"; return 1; }

  # 保持干净
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH}"
  hash -r

  _use_ros_jazzy

  echo "✅ Conda: $CONDA_PREFIX"
  echo "✅ Python: $(python -V)"

  # ROS2 环境
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    . "/opt/ros/jazzy/setup.bash"
    echo "✅ Sourced: /opt/ros/jazzy/setup.bash"
  else
    echo "ℹ️ 未找到 /opt/ros/jazzy/setup.bash"
  fi

  # 工作区环境
  if [ -f "$DRONE_WS/install/setup.bash" ]; then
    . "$DRONE_WS/install/setup.bash"
    echo "✅ Sourced: install/setup.bash (colcon)"
  else
    echo "ℹ️ 未找到 install/，请先执行：colcon build"
  fi
  echo "🎯 ROS2 environment ready."

  # 进入ROS2工作区
  if [ -d "$DRONE_WS" ]; then
    cd "$DRONE_WS"
    echo "✅ 工作区: $DRONE_WS"
  else
    echo "⚠️  PX4工作区不存在: $DRONE_WS"
  fi
}

# PX4固件开发环境
setup_px4() {
  echo "🚀 Setting up PX4 Firmware Build Environment..."
  conda activate px4 || { echo "❌ conda env 'px4' 不存在"; return 1; }

  # 保持干净的Python环境
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1

  # 强制CMake使用px4环境的Python（避免找到其他conda环境的Python）
  export PYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python3"
  export Python3_EXECUTABLE="$PYTHON_EXECUTABLE"

  hash -r

  echo "✅ Conda环境: $CONDA_PREFIX"
  echo "✅ Python版本: $(python -V)"
  echo "✅ Python路径: $PYTHON_EXECUTABLE"

  # 验证关键依赖
  python -c "import em; print('✅ empy version:', em.__version__)" 2>/dev/null || echo "⚠️  empy未安装"
  python -c "import jsonschema; print('✅ jsonschema已安装')" 2>/dev/null || echo "⚠️  jsonschema未安装"

  # 进入PX4工作区
  if [ -d "$PX4_WS" ]; then
    cd "$PX4_WS"
    echo "✅ 工作区: $PX4_WS"
  else
    echo "⚠️  PX4工作区不存在: $PX4_WS"
  fi

  echo "🎯 PX4 build environment ready."
  echo "💡 可用命令: make px4_fmu-v6x_avians_v1"
}

# 快捷别名
alias airship='setup_airship_ros1'
alias airship2='setup_airship_ros2'
alias drone='setup_drone_ros2'
alias px4='setup_px4'

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/zuoyangjkpi/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/zuoyangjkpi/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/zuoyangjkpi/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/zuoyangjkpi/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<
