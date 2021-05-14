#!/usr/bin/env bash

function help {
  echo 'build.sh --app <app> [--version <version>] [<command>]'
  echo ''
  echo '    --app        Name of the app'
  echo '    --version    Version of the app'
  echo '    --help       Prints this help'
  echo ''
}

ROOT_DIR=$(realpath $(dirname $0)/..)
BUILD_DIR=$ROOT_DIR/build
CODE_DIR=$ROOT_DIR/code
DIST_DIR=$ROOT_DIR/dist

ARGS=()
APP=""
COMMAND=""
VERSION=""

while [[ $# > 0 ]]
do
  if [ $1 == '--app' ]; then APP=$2; shift; shift; continue; fi
  if [ $1 == '--version' ]; then VERSION=$2; shift; shift; continue; fi
  if [ $1 == '--help' ]; then help; exit 0; fi
  ARGS+=($1)
  shift
done

if [ -z $APP ]; then echo 'Missing app'; echo ''; help; exit 1; fi
if [ -z $VERSION ]; then VERSION='production'; fi
if [ ${#ARGS[@]} -eq 1 ]; then COMMAND="${ARGS[0]}"; fi

TARGET_DIR=$DIST_DIR/$APP/$VERSION
if [ ! -d $TARGET_DIR ]; then mkdir -p $TARGET_DIR; fi
APP_MAKEFILE=$BUILD_DIR/makefiles/Makefile.$APP
TEMPLATE_MAKEFILE=$BUILD_DIR/makefiles/Makefile.template
MAKEFILE=$TARGET_DIR/Makefile
function CreateMakefile {
  cat $APP_MAKEFILE $TEMPLATE_MAKEFILE > $MAKEFILE
}
if [ ! -f $MAKEFILE ]; then CreateMakefile; fi

case $COMMAND in
  cleanall)
    rm -rf $TARGET_DIR/*
    ;;
  update)
    rm -f $MAKEFILE
    CreateMakefile
    ;;
  *)
    make -C $TARGET_DIR "${ARGS[@]}"
    ;;
esac

echo "Done."
