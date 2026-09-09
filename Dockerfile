# Development image for Convex-MPC.
# Native and browser toolchains live here; qpOASES and MuJoCo are fetched
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
	curl \
	gdb \
	git \
	libeigen3-dev \
	libglfw3-dev \
	openssh-client \
	python3 \
	sudo \
	vim \
	wget \
	&& rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
	&& apt-get install -y --no-install-recommends nodejs \
	&& rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/emscripten-core/emsdk.git /opt/emsdk \
	&& /opt/emsdk/emsdk install 3.1.56 \
	&& /opt/emsdk/emsdk activate 3.1.56 \
	&& ln -s /opt/emsdk/upstream/emscripten/em++ /usr/local/bin/em++ \
	&& ln -s /opt/emsdk/upstream/emscripten/emcc /usr/local/bin/emcc \
	&& ln -s /opt/emsdk/upstream/emscripten/emcmake /usr/local/bin/emcmake

# Non-root user with passwordless sudo
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
	&& useradd --create-home --shell /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
	&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/convex-mpc
ENV SHELL=/bin/bash
ENV EMSDK_QUIET=1
ENV EMSDK=/opt/emsdk
ENV EM_CONFIG=/opt/emsdk/.emscripten
ENV PATH=/opt/emsdk/upstream/emscripten:${PATH}
