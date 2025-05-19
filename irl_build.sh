source_setup () {
    if [ -n "$ZSH_VERSION" ]; then
        . install/setup.zsh
    elif [ -n "$BASH_VERSION" ]; then
        . install/setup.bash
    fi
}
export MAKEFLAGS="-j 2"
colcon build --packages-select interfaces topic_remapper intersection_detector
source_setup
colcon build --packages-ignore interfaces --executor sequential
source_setup
