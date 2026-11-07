#!/usr/bin/env bash
# Roda a suíte de testes do projeto com relatório de cobertura.
# Uso: tests/run_tests.sh [args extras do pytest]
#   ex.: tests/run_tests.sh -k pocket -x
#   HTML: tests/run_tests.sh --cov-report=html   (gera htmlcov/)
#
# Os testes dos nós ROS precisam do ROS 2: se ele ainda não estiver carregado,
# o script carrega /opt/ros/$ROS_DISTRO (padrão: jazzy). Sem ROS, esses testes
# são pulados e a cobertura mínima não é exigida.
# Cobertura mínima: COV_MIN (padrão 80).
set -eo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TESTS_DIR/.." && pwd)"
cd "$ROOT"

if ! python3 -c 'import rclpy' 2>/dev/null; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    if [ -f "$ROS_SETUP" ]; then
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
    fi
fi

FAIL_UNDER=()
if python3 -c 'import rclpy' 2>/dev/null; then
    FAIL_UNDER=(--cov-fail-under="${COV_MIN:-80}")
else
    echo "Aviso: ROS 2 não encontrado — testes dos nós serão pulados." >&2
fi

python3 -m pytest "$TESTS_DIR" -q \
    --cov="$ROOT/navigation/scripts" \
    --cov="$ROOT/hardware/hardware" \
    --cov-report=term-missing \
    "${FAIL_UNDER[@]}" \
    "$@"
