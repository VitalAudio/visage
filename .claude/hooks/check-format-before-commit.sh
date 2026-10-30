#!/bin/sh

input=$(cat)

if ! printf '%s' "$input" | grep -Eq '"command": *"([^"\\]|\\.)*git([^"\\]|\\.)* commit'; then
  exit 0
fi

dir=$(printf '%s' "$input" | sed -n 's/.*"cwd": *"\([^"]*\)".*/\1/p' | sed 's/\\\\/\\/g')
cd "$dir" 2>/dev/null || cd "$CLAUDE_PROJECT_DIR" || exit 0
root=$(git rev-parse --show-toplevel 2>/dev/null) || exit 0

if [ -f "$root/.githooks/pre-commit" ] && ! (cd "$root" && sh .githooks/pre-commit); then
  exit 2
fi
exit 0
