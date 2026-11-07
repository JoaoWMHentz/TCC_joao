#!/usr/bin/env bash
# Publica a documentação de docs/ (e o README como página inicial) na Wiki do GitHub.
#
# A fonte única é este repositório: edite os arquivos em docs/ e rode o script
# de novo. Ele reescreve os links para o formato da Wiki:
#   arquitetura.md#x         -> Arquitetura#x
#   ../hardware/protocol.py  -> https://github.com/<repo>/blob/main/hardware/protocol.py
#
# Pré-requisito (uma vez só): no GitHub, abra a aba "Wiki" do repositório e
# crie a primeira página (qualquer conteúdo). Antes disso o repositório da Wiki
# não existe e o clone falha.
#
# Uso: scripts/publish_wiki.sh [--dry-run]
#   --dry-run  gera as páginas numa pasta temporária e mostra o diff, sem enviar
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORIGIN="$(git -C "$ROOT" remote get-url origin)"
SLUG="$(sed -E 's#(git@github.com:|https://github.com/)##; s#\.git$##' <<<"$ORIGIN")"
WIKI_URL="https://github.com/${SLUG}.wiki.git"
BLOB="https://github.com/${SLUG}/blob/main"
TREE="https://github.com/${SLUG}/tree/main"
DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

# docs/<arquivo>.md -> nome da página na Wiki
declare -A PAGES=(
    [integracao.md]=Como-tudo-funciona
    [ros2.md]=ROS-2
    [slam-rtabmap.md]=SLAM-RTAB-Map
    [navegacao-nav2.md]=Navegacao-Nav2
    [controle-base.md]=Controle-da-Base
    [exploracao.md]=Exploracao-Autonoma
    [arquitetura.md]=Arquitetura
    [decisoes-arquitetura.md]=Decisoes-de-Arquitetura
    [diagramas.md]=Diagramas
    [conformidade-portfolio.md]=Conformidade-Portfolio
)

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "[wiki] clonando $WIKI_URL"
if ! git clone -q "$WIKI_URL" "$WORK/wiki"; then
    echo "Erro: a Wiki ainda não existe. Crie a primeira página pela aba Wiki do GitHub." >&2
    exit 1
fi

# Reescreve os links de um arquivo. $1 = arquivo, $2 = pasta do arquivo relativa à raiz.
rewrite() {
    local file="$1" dir="$2" expr="" src page
    for src in "${!PAGES[@]}"; do
        page="${PAGES[$src]}"
        # (arquitetura.md) e (docs/arquitetura.md), com ou sem âncora
        expr+="s#\]\((docs/)?${src//./\\.}(\#[^)]*)?\)#](${page}\2)#g;"
    done
    if [ "$dir" = "docs" ]; then
        expr+="s#\]\(\.\./([^)]*/)\)#](${TREE}/\1)#g;"   # pasta
        expr+="s#\]\(\.\./([^)]*)\)#](${BLOB}/\1)#g;"    # arquivo
    else
        # README na raiz: caminhos relativos que não começam com http/#/docs
        expr+="s#\]\(((LICENSE|[a-z_]+/)[^)]*/)\)#](${TREE}/\1)#g;"
        expr+="s#\]\(((LICENSE|[a-z_]+/)[^)]*)\)#](${BLOB}/\1)#g;"
    fi
    sed -E "$expr" "$file"
}

for src in "${!PAGES[@]}"; do
    rewrite "$ROOT/docs/$src" docs > "$WORK/wiki/${PAGES[$src]}.md"
done
rewrite "$ROOT/README.md" . > "$WORK/wiki/Home.md"

cat > "$WORK/wiki/_Sidebar.md" <<'EOF'
**Documentação**

- [Início](Home)
- [Como tudo funciona](Como-tudo-funciona)

**Componentes**

- [ROS 2](ROS-2)
- [SLAM com RTAB-Map](SLAM-RTAB-Map)
- [Navegação com Nav2](Navegacao-Nav2)
- [Controle da base](Controle-da-Base)
- [Exploração autônoma](Exploracao-Autonoma)

**Arquitetura**

- [Visão geral e dados](Arquitetura)
- [Decisões (ADRs)](Decisoes-de-Arquitetura)
- [Diagramas](Diagramas)

**Portfólio**

- [Conformidade e justificativas](Conformidade-Portfolio)
EOF

cd "$WORK/wiki"
git add -A
if git diff --cached --quiet; then
    echo "[wiki] nada mudou."
    exit 0
fi
git --no-pager diff --cached --stat
if [ "$DRY_RUN" = 1 ]; then
    echo "[wiki] --dry-run: páginas geradas, nada enviado."
    exit 0
fi
git commit -q -m "docs: atualiza a Wiki a partir de docs/ ($(git -C "$ROOT" rev-parse --short HEAD))"
git push -q
echo "[wiki] publicada: https://github.com/${SLUG}/wiki"
