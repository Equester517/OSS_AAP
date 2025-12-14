#!/bin/sh

SCRIPT_DIR=$(dirname "$0")
INSTALL_DIR=$(cd "$SCRIPT_DIR" 2>/dev/null && pwd -P)

# configure run environment
export PARA_CORE="$INSTALL_DIR"
export PARA_CONF="$INSTALL_DIR/etc"
export PARA_DATA="$INSTALL_DIR/var"
export PARA_APPL="$INSTALL_DIR/opt"

exec "$PARA_CORE"/bin/EM "$@"
        