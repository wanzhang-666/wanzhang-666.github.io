#!/bin/bash
# 一键创建 GitHub 仓库并推送 Hexo 博客源码
# 前置条件：本机 SSH 公钥已添加到 GitHub（见下方检查）

set -e

USER="wanzhang-666"
REPO="$USER.github.io"
REMOTE="git@github.com:$USER/$REPO.git"

echo "=========================================="
echo "  推送 Hexo 博客到 GitHub Pages"
echo "  目标仓库: $REPO"
echo "=========================================="

echo ""
echo "1. 检查 SSH 授权..."
if ! timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=10 -T git@github.com 2>&1 | grep -q "successfully authenticated"; then
  echo "   [FAIL] SSH 公钥未授权。请先添加这把公钥到 GitHub:"
  echo "   https://github.com/settings/keys"
  echo ""
  ssh-keygen -lf ~/.ssh/id_ed25519.pub
  cat ~/.ssh/id_ed25519.pub
  echo ""
  echo "   添加后重新运行本脚本。"
  exit 1
fi
echo "   [OK] SSH 授权通过"

echo ""
echo "2. 检查仓库是否存在..."
if git ls-remote "$REMOTE" HEAD >/dev/null 2>&1; then
  echo "   [OK] 仓库已存在，跳过创建"
else
  echo "   仓库不存在，尝试用 GitHub API 创建..."
  echo "   如果没有配置 TOKEN 环境变量，请手动创建："
  echo "   https://github.com/new  -> 仓库名填 $REPO，选 Public，不要勾选任何初始化选项"
  if [ -n "$GITHUB_TOKEN" ]; then
    curl -s -X POST https://api.github.com/user/repos \
      -H "Authorization: token $GITHUB_TOKEN" \
      -H "Accept: application/vnd.github.v3+json" \
      -d "{\"name\":\"$REPO\",\"description\":\"RK3588 ROS2 机器人实战博客\",\"homepage\":\"https://$USER.github.io\",\"private\":false}" \
      | grep -E '"(full_name|html_url|message)"' | head -5
    sleep 3
  else
    echo "   [WARN] 未设置 GITHUB_TOKEN，请在浏览器手动创建仓库后重新运行本脚本。"
    read -r -p "   手动创建好了吗？按回车继续..."
  fi
fi

echo ""
echo "3. 设置远程并推送..."
git remote remove origin 2>/dev/null || true
git remote add origin "$REMOTE"
git branch -M main
git push -u origin main

echo ""
echo "=========================================="
echo "  推送完成！"
echo "  下一步：去仓库 Settings -> Pages -> Source 选 GitHub Actions"
echo "  访问地址: https://$USER.github.io"
echo "=========================================="
