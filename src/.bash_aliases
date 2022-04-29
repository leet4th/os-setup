#! /bin/bash

# ls
alias ll="ls -lh"
alias la="ll -A"  # ll showing hidden files
alias lk="ll -Sr" # sort by size
alias lc="ll -cr" # sort by change time
alias lu="ll -ur" # sort by access time
alias lt="ll -tr" # sort by date
alias lf="ll | grep -v '^d'" # files only
alias ld="ll | grep '^d'" # directories only
alias llt="lt" # habit :)

# vim
alias g='gvim -p '

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Find command in your history
alias gh='history | grep'

# Aliasing python to force python3 usage
alias python=python3
alias pip=pip3

# Prints out the python executable path
alias whichpy='python -c "import sys; print(sys.executable)"'

# Go to profile dir
alias pro="cd $PROFILE_ENV"
