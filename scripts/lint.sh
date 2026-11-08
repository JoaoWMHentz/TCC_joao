#!/usr/bin/env bash
# Verifica o guia de estilo do projeto.
#
#   Python      -> ament_flake8: o linter oficial do ROS 2 (PEP 8 com linhas de
#                  até 99 colunas, ordem de imports no estilo Google, docstrings
#                  PEP 257, nomes que escondem builtins, compreensões)
#   TypeScript  -> tsc (verificação de tipos da interface), se o node_modules existir
#
# Uso: scripts/lint.sh            (sai com código != 0 se houver problema)
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1

if ! command -v ament_flake8 >/dev/null 2>&1; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

status=0
OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT
mapfile -t PY_FILES < <(git ls-files '*.py')

echo "[lint] ament_flake8 em ${#PY_FILES[@]} arquivos Python"
if ament_flake8 "${PY_FILES[@]}" > "$OUT" 2>&1; then
    echo "[lint] Python: ok"
else
    grep -E '^\S+\.py:[0-9]+' "$OUT"
    echo "[lint] Python: $(grep -cE '^\S+\.py:[0-9]+' "$OUT") problema(s)"
    status=1
fi

if [ -d web/frontend/node_modules ]; then
    echo "[lint] tsc (interface)"
    if (cd web/frontend && npm run -s typecheck); then
        echo "[lint] TypeScript: ok"
    else
        status=1
    fi
else
    echo "[lint] interface sem node_modules: verificação de tipos ignorada"
fi

exit $status
