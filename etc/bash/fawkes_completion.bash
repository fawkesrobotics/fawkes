_fawkes()
{
  local cur prev opts base
  cur="${COMP_WORDS[COMP_CWORD]}"
  prev="${COMP_WORDS[COMP_CWORD-1]}"

  opts="-c -C -d -D -g -h -l -L -p -P -u --net-service-name"

  case "${prev}" in
    -L)
      #OPTIONS=( $(compgen -W "${available_plugins}" -- ${cur##*,}) )
      #COMPREPLY=( ${OPTIONS[@]/#/${cur}} )
      COMPREPLY=( $(compgen -W "console file:" -- ${cur}) )
      return 0
      ;;
    -D*)
      COMPREPLY=( $(compgen -W "-k -s" -- ${cur}) )
      return 0
      ;;
  esac
  case "${cur}" in
    -*)
      COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
      return 0
      ;;
  esac
  local available_plugins=$(basename -s .so\
    $(ls  $(dirname ${COMP_WORDS[0]/#\~/$HOME})/../plugins))
  COMPREPLY=( $(compgen -W "${available_plugins}" -- ${cur}) )
  return 0
}
complete -F _fawkes fawkes
