# Development image for Convex-MPC.
# Only the toolchain + Eigen + GLFW live here; qpOASES and MuJoCo are fetched
# automatically by CMake at configure time.
FROM ubuntu:22.04

ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
	tzdata \
	bash-completion \
	build-essential \
	ca-certificates \
	clang-format \
	cmake \
	gdb \
	git \
	libeigen3-dev \
	libglfw3-dev \
	sudo \
	vim \
	wget \
	&& rm -rf /var/lib/apt/lists/*

# Non-root user with passwordless sudo
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
	&& useradd --create-home --shell /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
	&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/convex-mpc
ENV SHELL=/bin/bash
