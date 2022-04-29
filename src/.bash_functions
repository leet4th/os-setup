#! /bin/bash

# print paths, new line for each path
function printpath() {
    sed's/:/\n/g' <<< "$PATH"
}

# cd to venv project dir
function proj() {
    if [ -z "$VIRTUAL_ENV" ]; then
        echo "venv not set"
    else
        cd $VIRTUAL_ENV/../
    fi
}

# Uninstalls all currently installed pip packages, only works for virutal envs
function pipua() {
    if [ -z "$VIRTUAL_ENV" ]; then
        echo "venv not set"
    else
        pip uninstall -y r<(pip freeze)
    fi
}

# Open the profile files
function shark() {
    if [[ $# -eq 0 ]] ; then
        # No input args
        profile_dir=$HOME
    else
        # Any input args
        profile_dir=$PROFILE_ENV/src
    fi

    gvim -p "$profile_dir/.bashrc" \
            "$profile_dir/.bash_aliases" \
            "$profile_dir/.bash_functions" \
            "$profile_dir/.vimrc" \
            "$profile_dir/.vim" \
            "$profile_dir/.gitconfig"
}

# Diff the common profile files
function prodiff() {
    for item in $(comm -12 <(ls -A $HOME) <(ls -A $PROFILE_ENV/src)); do
        if [ -x "$(command -v xxdiff)" ]; then
            # use xxdiff as difftool
            xxdiff "$HOME/$item" "$PROFILE_ENV/src/$item"
        else
            # use diff as difftool
            echo "####################################"
            echo $item
            diff --color=auto "$HOME/$item" "$HOME/os-setup/$item"
            echo 
            echo 
            echo 
            echo 
        fi
    done
}

