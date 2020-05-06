# Clean all docker containers
docker container rm $(docker container ls -a -q)
# Clean all untagged docker images
docker image rm $(docker images -f "dangling=true" -q)

