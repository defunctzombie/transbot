IMAGE_NAME=transbot
USER=jetson
HOST_IP=192.168.1.207

all:

build-image:
	docker build . -t ${IMAGE_NAME}

deploy-image:
	docker save ${IMAGE_NAME} | bzip2 | pv | ssh ${USER}@${HOST_IP} docker load

restart-image:
	ssh ${USER}@${HOST_IP} docker stop ${IMAGE_NAME}
	ssh ${USER}@${HOST_IP} docker rm ${IMAGE_NAME}
	ssh ${USER}@${HOST_IP} docker run -it --name ${IMAGE_NAME} \
    	--net host \
    	--privileged \
    	--restart always \
    	--detach \
		-v /opt/transbot/var:/opt/transbot/var \
		-v /opt/transbot/etc:/opt/transbot/etc \
    	${IMAGE_NAME}
	ssh ${USER}@${HOST_IP} docker system prune -f

robot-cleanup:
	ssh ${USER}@${HOST_IP} docker system prune -f

.PHONY: all build-image deploy-image
