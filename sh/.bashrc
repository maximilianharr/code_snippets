## Arrow completion search
bind '"\e[A": history-search-backward'
bind '"\e[B": history-search-forward'

# Colorize bash prefix line and show GIT branch
parse_git_branch() {
     git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'
}
export PS1="\[\033[1;32m\]\u@\h\[\033[0m\]:\[\033[1;34m\]\w\[\033[00m\]:\[\033[1;31m\]\$(parse_git_branch)\[\033[00m\]\n$ "
