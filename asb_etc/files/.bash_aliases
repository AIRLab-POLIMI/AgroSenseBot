#!/bin/bash

# (partially taken from: https://github.com/cykerway/complete-alias)

_use_alias=1

# Disable the use of alias for a command.
_disable_alias () {
    local cmd="$1"

    # Remove completion for this command.
    complete -r "$cmd"

    # Reset static completions.
    #
    # We don't know the original no-alias completion for $cmd because it has
    # been overwritten by the alias completion function. What we do here is that
    # we reset all static completions to those in vanilla bash_completion. This
    # may be an overkill because we only need to reset completion for $cmd, but
    # it works.
    complete -u groups slay w sux
    complete -A stopped -P '"%' -S '"' bg
    complete -j -P '"%' -S '"' fg jobs disown
    complete -v readonly unset
    complete -A setopt set
    complete -A shopt shopt
    complete -A helptopic help
    complete -a unalias
    complete -A binding bind
    complete -c command type which
    complete -b builtin
    complete -F _service service
    complete -F _known_hosts traceroute traceroute6 tracepath tracepath6 \
        fping fping6 telnet rsh rlogin ftp dig mtr ssh-installkeys showmount
    complete -F _command aoss command do else eval exec ltrace nice nohup \
        padsp then time tsocks vsound xargs
    complete -F _root_command fakeroot gksu gksudo kdesudo really
    complete -F _longopt a2ps awk base64 bash bc bison cat chroot colordiff cp \
        csplit cut date df diff dir du enscript env expand fmt fold gperf \
        grep grub head irb ld ldd less ln ls m4 md5sum mkdir mkfifo mknod \
        mv netstat nl nm objcopy objdump od paste pr ptx readelf rm rmdir \
        sed seq sha{,1,224,256,384,512}sum shar sort split strip sum tac tail tee \
        texindex touch tr uname unexpand uniq units vdir wc who
    complete -F _minimal ''
    complete -D -F _completion_loader

    # Reset _use_alias flag.
    _use_alias=0
}

# Enable the use of alias for a command.
_enable_alias () {
    local cmd="$1"

    # Set completion for this command.
    complete -F _complete_alias "$cmd"

    # Set _use_alias flag.
    _use_alias=1
}

# Expand the first command as an alias, stripping all leading redirections.
_expand_alias () {
    local alias_name="${COMP_WORDS[0]}"
    local alias_namelen="${#alias_name}"
    local alias_array=( $(alias "$alias_name" | sed -r 's/[^=]*=//' | xargs) )
    local alias_arraylen="${#alias_array[@]}"
    local alias_str="${alias_array[*]}"
    local alias_strlen="${#alias_str}"

    # Rewrite current completion context by expanding alias.
    COMP_WORDS=(${alias_array[@]} ${COMP_WORDS[@]:1})
    (( COMP_CWORD+=($alias_arraylen-1) ))
    COMP_LINE="$alias_str""${COMP_LINE:$alias_namelen}"
    (( COMP_POINT+=($alias_strlen-$alias_namelen) ))

    # Strip leading redirections in alias-expanded command line.
    local redir="@(?([0-9])<|?([0-9&])>?(>)|>&)"
    while [[ "${#COMP_WORDS[@]}" -gt 0 && "${COMP_WORDS[0]}" == $redir* ]]; do
        local word="${COMP_WORDS[0]}"
        COMP_WORDS=(${COMP_WORDS[@]:1})
        (( COMP_CWORD-- ))
        local linelen="${#COMP_LINE}"
        COMP_LINE="${COMP_LINE#$word+( )}"
        (( COMP_POINT-=($linelen-${#COMP_LINE}) ))
    done
}

# alias completion function.
_complete_alias () {
    local cmd="${COMP_WORDS[0]}"

    if [[ "$_use_alias" -eq 1 ]]; then
        _expand_alias
    fi
    _disable_alias "$cmd"
    _command_offset 0
    _enable_alias "$cmd"
}

# System utils aliases

alias ll='ls -halF'
alias la='ls -A'
alias l='ls -CF'

alias x='xdg-open'

alias b='byobu'

alias jn='jupyter notebook'

alias pj='ros2 run plotjuggler plotjuggler -n'

alias tl='trash-list'
alias te='trash-empty'
alias t='trash'
complete -F _complete_alias t

alias duh='clr; pwd; du -had1 | sort -hr'

alias clr='tput reset' # like clear, but actually clears the terminal, erasing previous output

alias S='source ~/.bashrc'

alias cda='cd $COLCON_WS/src/AgroSenseBot/'

alias sai='sudo apt install'
complete -F _complete_alias sai

alias s='clr; git status'
complete -F _complete_alias s

alias d='git diff'
complete -F _complete_alias d

alias a='git add'
complete -F _complete_alias a

alias c='git commit -m'
complete -F _complete_alias c

alias p='git push'
complete -F _complete_alias p

alias sr='BLUE="\e[34m"; END_COLOR="\e[0m"; clear; for i in `find . -name .git -type d`; do ( cd $i/..; echo ""; echo -e "${BLUE}git fetch and status in `pwd`${END_COLOR}"; git fetch; git status; echo -e "${END_COLOR}" ); done;'

alias push='git push'
complete -F _complete_alias push

alias cap='pygmentize -g' # replacement of cat with python-pygments to cat with colors

alias ipy='ipython3'
complete -F _complete_alias ipy

alias r='rviz2'
complete -F _complete_alias r

alias cb='cd $COLCON_WS && colcon build --continue-on-error --symlink-install ; cd -'
complete -F _complete_alias cb

alias rt='ros2 topic'
complete -F _complete_alias rt

alias rs='ros2 service'
complete -F _complete_alias rs

alias ra='ros2 action'
complete -F _complete_alias ra

alias ri='ros2 interface'
complete -F _complete_alias ri

alias rn='ros2 node'
complete -F _complete_alias rn

alias rl='ros2 launch'
complete -F _complete_alias rl

alias rr='ros2 run'
complete -F _complete_alias rr

alias rb='ros2 bag'
complete -F _complete_alias rb

alias rbp='ros2 bag play --clock --read-ahead-queue-size 100000'
complete -F _complete_alias rbp

alias rp='ros2 param'
complete -F _complete_alias rp

alias ust='ros2 param set use_sim_time true'

alias vf='rr rqt_tf_tree rqt_tf_tree'

alias sima='clr; cb; rl asb_sim nav_dual_rtk_sim.launch.py record:=false start_in_control_mode_GCU:=true fake_heartbeat:=true'
complete -F _complete_alias sima

alias sim='clr; cb; rl asb_sim nav_dual_rtk_sim.launch.py record:=false'
complete -F _complete_alias sim

alias reala='clr; cb; rl asb_real nav_dual_rtk_real.launch.py start_in_control_mode_GCU:=true fake_heartbeat:=true'
complete -F _complete_alias reala

alias real='clr; cb; rl asb_real nav_dual_rtk_real.launch.py'
complete -F _complete_alias real

alias gq='rl asb_real gq7_launch.py'
complete -F _complete_alias gq

