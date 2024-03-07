# Build Docker Image

```
sudo docker build -t robot-studio .
```

# Enter a container

```
docker run -it --rm -v $(dirname "$(pwd)"):/root/dev/robot --privileged -v /dev:/dev robot-studio
```