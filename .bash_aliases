
alias rehash='source ~/.bashrc'
alias shark='g ~/.bashrc ~/.bash_aliases ~/.bash_mysetup ~/.bash_carla'

# some more ls aliases
alias ll='ls -l'
alias la='ls -A'
alias llt='ls -ltr'
alias lld='ls -ltd */'

# gVim
alias g='gvim -p '

# Starts a server on local network to share current directory
share () {
    # Print out host address
    printf 'host address: '; hostname -I | awk '{print $1}'

    # Start server
    python -m http.server $1
}


