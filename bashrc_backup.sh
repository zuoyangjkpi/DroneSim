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
# Optional: forgiving cd with ** globbing
shopt -s cdspell
shopt -s globstar

# Write every command to the history file immediately
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

# Safer aliases (comment them out if you prefer defaults)
alias rm='rm -i'
alias cp='cp -i'
alias mv='mv -i'

alias grep='grep --color=auto'
alias egrep='egrep --color=auto'
alias fgrep='fgrep --color=auto'

# Handy system info aliases
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
# Terminal title bar: user@host: cwd
case "$TERM" in
  xterm*|rxvt*)
    PS1="\[\e]0;\u@\h: \w\a\]"
    ;;
  *)
    PS1=""
    ;;
esac
# Colored prompt + Conda indicator (via CONDA_PROMPT_MODIFIER)
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
# Rust (needed by opengen and friends)
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
export PATH="$HOME/.cargo/bin:$PATH"

# colcon autocomplete (if present)
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
    echo "⚠️  /opt/ros/noetic/setup.bash is missing (install it on the host or use Docker)."
  fi
}

_use_ros_jazzy() {
  if [ -r "/opt/ros/jazzy/setup.bash" ]; then
    . /opt/ros/jazzy/setup.bash
    echo "✅ ROS2 Jazzy loaded."
  else
    echo "⚠️  /opt/ros/jazzy/setup.bash is missing (Jazzy normally lives on Ubuntu 24.04 or Docker)."
  fi
}

# --------------------------
# 10) One-shot env setters
# --------------------------
# ROS1 (Noetic) development environment
setup_airship_ros1() {
  echo "🚀 Setting up AirshipMPC (ROS1 / Noetic)..."
  conda activate airship_env || { echo "❌ conda env 'airship_env' does not exist"; return 1; }

  # Keep the environment clean
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH}"
  hash -r

  _use_ros_noetic

  echo "✅ Conda: $CONDA_PREFIX"
  echo "✅ Python: $(python -V)"

  # Source the workspace (catkin or colcon)
  if [ -f "$AIRSHIP_WS/devel/setup.bash" ]; then
    . "$AIRSHIP_WS/devel/setup.bash"
    echo "✅ Sourced: devel/setup.bash (catkin)"
  elif [ -f "$AIRSHIP_WS/install/setup.bash" ]; then
    . "$AIRSHIP_WS/install/setup.bash"
    echo "✅ Sourced: install/setup.bash (colcon)"
  else
    echo "ℹ️ Could not find devel/ or install/—build first (catkin_make / colcon build)."
  fi
  echo "🎯 ROS1 environment ready."
}

# ROS2 (Jazzy) development environment
setup_airship_ros2() {
  echo "🚀 Setting up AirshipMPC (ROS2 / Jazzy)..."
  conda activate airship_ros2 || { echo "❌ conda env 'airship_ros2' does not exist"; return 1; }

  # Keep the environment clean
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH}"
  hash -r

  # Qwen API key environment variables
  export DASHSCOPE_API_KEY="sk-5db2e04d96f24a4bb2ccad84af9cdb4b"
  export QWEN_API_KEY="$DASHSCOPE_API_KEY"

  _use_ros_jazzy

  echo "✅ Conda: $CONDA_PREFIX"
  echo "✅ Python: $(python -V)"

  if [ -f "$AIRSHIP_WS/install/setup.bash" ]; then
    . "$AIRSHIP_WS/install/setup.bash"
    echo "✅ Sourced: install/setup.bash (colcon)"
  else
    echo "ℹ️ Could not find install/—run colcon build first."
  fi
  echo "🎯 ROS2 environment ready."
}

# ROS2 (Jazzy) development environment
setup_drone_ros2() {
  echo "🚀 Setting up AVIANS (ROS2 / Jazzy)..."
  conda activate airship_ros2 || { echo "❌ conda env 'airship_ros2' does not exist"; return 1; }

  # Keep the environment clean
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export CMAKE_PREFIX_PATH="$CONDA_PREFIX:${CMAKE_PREFIX_PATH}"
  hash -r

  _use_ros_jazzy

  echo "✅ Conda: $CONDA_PREFIX"
  echo "✅ Python: $(python -V)"

  # ROS2 environment
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    . "/opt/ros/jazzy/setup.bash"
    echo "✅ Sourced: /opt/ros/jazzy/setup.bash"
  else
    echo "ℹ️ /opt/ros/jazzy/setup.bash was not found."
  fi

  # Workspace environment
  if [ -f "$DRONE_WS/install/setup.bash" ]; then
    . "$DRONE_WS/install/setup.bash"
    echo "✅ Sourced: install/setup.bash (colcon)"
  else
    echo "ℹ️ Could not find install/—run colcon build first."
  fi
  echo "🎯 ROS2 environment ready."

  # Switch into the ROS2 workspace
  if [ -d "$DRONE_WS" ]; then
    cd "$DRONE_WS"
    echo "✅ Workspace: $DRONE_WS"
  else
    echo "⚠️  Workspace does not exist: $DRONE_WS"
  fi
}

# PX4 firmware development environment
setup_px4() {
  echo "🚀 Setting up PX4 Firmware Build Environment..."
  conda activate px4 || { echo "❌ conda env 'px4' does not exist"; return 1; }

  # Keep the Python environment clean
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1

  # Force CMake to use this env's python (avoid leaking other conda envs)
  export PYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python3"
  export Python3_EXECUTABLE="$PYTHON_EXECUTABLE"

  hash -r

  echo "✅ Conda env: $CONDA_PREFIX"
  echo "✅ Python version: $(python -V)"
  echo "✅ Python path: $PYTHON_EXECUTABLE"

  # Verify required deps
  python -c "import em; print('✅ empy version:', em.__version__)" 2>/dev/null || echo "⚠️  empy is not installed"
  python -c "import jsonschema; print('✅ jsonschema installed')" 2>/dev/null || echo "⚠️  jsonschema is not installed"

  # Switch into the PX4 workspace
  if [ -d "$PX4_WS" ]; then
    cd "$PX4_WS"
    echo "✅ Workspace: $PX4_WS"
  else
    echo "⚠️  PX4 workspace does not exist: $PX4_WS"
  fi

  echo "🎯 PX4 build environment ready."
  echo "💡 Example: make px4_fmu-v6x_avians_v1"
}

# Quick aliases
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
