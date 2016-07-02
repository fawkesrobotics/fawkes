function parse_yaml {
   local prefix=$2
   local s='[[:space:]]*' w='[a-zA-Z0-9_]*' fs=$(echo @|tr @ '\034')
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p"  $1 |
   awk -F$fs '{
      indent = length($1)/2;
      vname[indent] = $2;
      for (i in vname) {if (i > indent) {delete vname[i]}}
      if (length($3) > 0) {
         vn=""; for (i=0; i<indent; i++) {vn=(vn)(vname[i])}
         printf("%s%s%s ", "'$prefix'",vn, $2, $3);
      }
   }'
}

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
  local meta_plugins=$(parse_yaml $(dirname\
    ${COMP_WORDS[0]/#\~/$HOME})/../cfg/conf.d/meta_plugins.yaml)
  COMPREPLY=( $(compgen -W "${available_plugins} ${meta_plugins}" -- ${cur}) )
  return 0
}
complete -F _fawkes fawkes
