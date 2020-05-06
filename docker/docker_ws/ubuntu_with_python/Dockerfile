# Build image from Dockerfile:
# docker image build -t maximilianharr/ubuntu-with-python .

# Run a container from the newly created image:
# docker container run --rm maximilianharr/ubuntu-with-python

# Set the base image to be the latest Node.js
FROM ubuntu:18.04

# Update and install python
RUN apt update && apt -y upgrade && apt install -y python

# Copy everything of current directory of host into the image
RUN mkdir -p /home/playground

# Copy everything of the current directory of host into the WORKDIR of the container
WORKDIR /home/playground
COPY . .

# Run python script when starting the container
CMD [ "python", "./print_hello_world.py" ]
