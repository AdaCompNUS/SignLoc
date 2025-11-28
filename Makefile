build:
ifdef noetic
	docker build -f NoeticDockerfile -t signlocnoetic .
endif
ifdef  humble
	docker build -f HumbleDockerfile -t signlochumble .
endif

run:
ifdef noetic
	docker run --rm -it --init   --gpus=all  --network host --privileged -e "DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	 -v $(CURDIR):/SignLoc signlocnoetic
endif
ifdef  humble
	docker run --rm -it --init   --gpus=all  --network host --privileged -e "DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	 -v $(CURDIR):/SignLoc signlochumble
endif