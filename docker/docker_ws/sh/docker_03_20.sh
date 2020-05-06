# Check docker version
# CLI talks to docker engine to get output
docker version

# Detailed docker config values
docker info

# Download image 'nginx' from Docker Hub
# Start container from that image
# Open port 88 on host IP (if "bind error" choose other port)
# Route traffic to container IP, port 80
# Check in firefox: localhost:88
docker container run --publish 88:80 nginx

# --detach detaches docker from CLI and returns container ID
# --name specifies docker container name (else it uses a random name like frosty_roentgen)
docker container run --publish 88:80 --detach --name webhost nginx

# See running containers
docker container ls

# See running containers
docker container ls

# Stop a container
docker container stop <ContainerID>

# docker container run > starts new container
# docker container start > start an existing stopped container

# Shows logs for specific container
docker container logs <ContainerName>

# Show user, PID, .. of container
docker container top <ContainerName>

# Remove ran containers (docker container ls -a)
docker container rm <ContainerID>

