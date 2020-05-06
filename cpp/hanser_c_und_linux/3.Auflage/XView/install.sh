#!/bin/sh
#
# (c) 2005 Martin Gräfe
# AuM.Graefe@t-online.de

if [ -d /usr/openwin/include ]; then
  echo "XView is already installed (/usr/openwin/)."
  exit
fi

SRC_PATH="$PWD"

echo -e "creating uninstall script \033[1m'remove_xview'\033[0m..."
echo > remove_xview "#!/bin/sh"
chmod a+x remove_xview

cd /usr

if [ -d openwin ]; then
  echo >> $SRC_PATH/remove_xview "rm -r /usr/openwin/include"
  echo >> $SRC_PATH/remove_xview "rm -r /usr/openwin/lib"
  if [ -d openwin/lib ]; then
    echo >> $SRC_PATH/remove_xview "mv /usr/openwin/lib_old /usr/openwin/lib"
    cp -r openwin/lib openwin/lib_old
  fi
else
  echo >> $SRC_PATH/remove_xview "rm -r /usr/openwin"
fi

echo "extracting XView files..."
tar -xzf "$SRC_PATH/openwin.tgz"

echo "creating symbolic links..."
if [ ! -e /usr/lib/libxview.so ]; then
  ln -s /usr/openwin/lib/libxview.so /usr/lib/libxview.so
  echo >> $SRC_PATH/remove_xview "rm /usr/lib/libxview.so"
fi
if [ ! -e /usr/lib/libxview.so.3 ]; then
  ln -s /usr/openwin/lib/libxview.so.3 /usr/lib/libxview.so.3
  echo >> $SRC_PATH/remove_xview "rm /usr/lib/libxview.so.3"
fi
if [ ! -e /usr/lib/libolgx.so ]; then
  ln -s /usr/openwin/lib/libolgx.so /usr/lib/libolgx.so
  echo >> $SRC_PATH/remove_xview "rm /usr/lib/libolgx.so"
fi
if [ ! -e /usr/lib/libolgx.so.3 ]; then
  ln -s /usr/openwin/lib/libolgx.so.3 /usr/lib/libolgx.so.3
  echo >> $SRC_PATH/remove_xview "rm /usr/lib/libolgx.so.3"
fi
if [ ! -e /usr/X11/lib/libX11.so ]; then
  ln -s /usr/X11/lib/libX11.so.6 /usr/X11/lib/libX11.so
  echo >> $SRC_PATH/remove_xview "rm /usr/X11/lib/libX11.so"
fi

cd "$SRC_PATH"

echo "done."
echo "To uninstall xview type './remove_xview' in this directory."
