#!/bin/bash

# Pega o caminho absoluto da pasta nuttx
NUTTX_PATH=$(readlink -f nuttx)

echo "Generating compile_commands.json for $NUTTX_PATH..."

# -C nuttx: entra na pasta, lê o Makefile, mas o log sai aqui na raiz
make -C nuttx --always-make --dry-run \
 | grep -wE 'gcc|g\+\+' \
 | grep -w '\-c' \
 | jq -nR --arg dir "$NUTTX_PATH" '[inputs|{directory:$dir, command:., file: match(" [^ ]+$").string[1:]}]' \
 > compile_commands.json

echo "Done. compile_commands.json is now in the root."
