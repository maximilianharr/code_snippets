# sudo it4ad-posix-client start
sudo it4ad-loopbacknfs-client start
sudo mount -o hard,nolock,sync,rsize=131072,wsize=131072 localhost:/mapr /mapr
