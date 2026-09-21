#!/bin/bash
# ============================================================
# build.sh  —  编译 Beamer 幻灯片
# 依赖: xelatex (TeX Live 或 MacTeX)
# 运行: bash build.sh
# ============================================================
set -e
cd "$(dirname "$0")"

TEXFILE="slides.tex"
OUTFILE="slides.pdf"

if ! command -v xelatex &>/dev/null; then
    echo "错误: 未找到 xelatex，请安装 MacTeX 或 TeX Live"
    echo "  macOS: brew install --cask mactex"
    echo "  Ubuntu: sudo apt install texlive-full"
    exit 1
fi

echo "第1次编译 (生成目录)..."
xelatex -interaction=nonstopmode "$TEXFILE" > /dev/null

echo "第2次编译 (修正交叉引用)..."
xelatex -interaction=nonstopmode "$TEXFILE" > /dev/null

echo ""
echo "编译完成: $OUTFILE"

# 可选：打开 PDF
if [[ "$1" == "--open" ]]; then
    open "$OUTFILE"
fi
