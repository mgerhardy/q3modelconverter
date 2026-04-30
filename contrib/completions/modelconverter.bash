# bash completion for modelconverter
_modelconverter() {
    local cur prev subcmds opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    subcmds="info validate convert player"
    opts="-i --input -o --output --fps --skin --shader --shader-in --skin-in
          --no-auto-skin --no-auto-shader --player --subdivide --decimate
          --gen-lods --gen-lod-ratios --asset-root --shader-path --shader-depth
          --info --validate -v --verbose -q --quiet -h --help"

    if [[ ${COMP_CWORD} -eq 1 ]]; then
        COMPREPLY=( $(compgen -W "${subcmds} ${opts}" -- "${cur}") )
        return
    fi

    case "${prev}" in
        -i|--input|-o|--output|--shader-in|--skin-in|--asset-root|--shader-path)
            COMPREPLY=( $(compgen -f -- "${cur}") )
            return ;;
        --fps|--subdivide|--decimate|--gen-lods|--gen-lod-ratios|--shader-depth)
            return ;;
    esac

    COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
}
complete -o default -F _modelconverter modelconverter
