#!/bin/bash
cd src
for d in */ ; do
    echo "$d"
		cd $d
		git branch | grep '*' 
		git show HEAD --pretty=format:"latest commit: %h %ad %an" --raw | grep 'latest commit'
		cat .git/config | grep 'url'
		echo ""
		cd ../
done
cd ..
