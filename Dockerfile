# Use an official Ubuntu as a base image
FROM ubuntu:22.04


ARG USER_NAME
ARG GROUP_NAME=drcd
ARG WORKSPACE_NAME=drcd_ws
ARG PROJECT_NAME=convex-mpc

# Set non-interactive mode to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
	&& ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
	&& apt-get update \
	&& apt-get -y -q --no-install-recommends install \
	tzdata \
	&& apt-get clean


# Install packages
RUN apt-get update && apt-get install -y \
	apt-utils \
	bash-completion \
	curl \
	clang-format \
	g++ \
	git \
	gdb \
	gnupg2 \
	sshpass \
	sudo \
	vim \
	wget \
	xterm \
	unzip \
	libyaml-cpp-dev \
	&& rm -rf /var/lib/apt/lists/*

# Install required dependencies
RUN apt-get update && apt-get install -y \
	build-essential \
	gcc \
	make \
	libboost-all-dev \
	libhdf5-dev \
	&& rm -rf /var/lib/apt/lists/*

# Install CMake 3.27.4
RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.4/cmake-3.27.4-linux-x86_64.tar.gz && \
	tar -xzvf cmake-3.27.4-linux-x86_64.tar.gz && \
	cd cmake-3.27.4-linux-x86_64 && \
	rm -rf /usr/local/man && \
	cp -r * /usr/local/ && \
	cd .. && \
	rm -rf cmake-3.27.4-linux-x86_64 cmake-3.27.4-linux-x86_64.tar.gz

# Install Eigen 3.4.0	
RUN git clone --branch 3.4.0 https://gitlab.com/libeigen/eigen.git /opt/eigen \
	&& mkdir -p /opt/eigen/build && cd /opt/eigen/build \
	&& cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DEIGEN_BUILD_DOC=OFF \
	-DBUILD_TESTING=OFF \
	.. \
	&& make install \
	&& cd /opt && rm -r /opt/eigen

# Install raisimLib v1.1.7	
ENV RAISIM_DIR=/home/${USER_NAME}/raisimLib
ENV LOCAL_INSTALL=/home/${USER_NAME}/raisim_build
RUN mkdir -p ${LOCAL_INSTALL}
RUN git clone --branch v1.1.7 https://github.com/raisimTech/raisimLib.git ${RAISIM_DIR} \
	&& mkdir -p ${RAISIM_DIR}/build && cd ${RAISIM_DIR}/build \
	&& cmake \
	-DCMAKE_INSTALL_PREFIX=${LOCAL_INSTALL} \
	-DRAISIM_EXAMPLE=OFF \
	-DRAISIM_PY=OFF \
	.. \
	&& make install 

# Install qpSWIFT 
RUN git clone https://github.com/qpSWIFT/qpSWIFT /opt/qpSWIFT \
	&& cd /opt/qpSWIFT \
	&& cmake \
	-S . \
	-B build \
	&& cmake --build build --target install \
	&& cd /opt && rm -r /opt/qpSWIFT

# Install qpOASES
RUN git clone --branch stable/3.2 https://github.com/coin-or/qpOASES.git /opt/qpOASES \
	&& mkdir -p /opt/qpOASES/build && cd /opt/qpOASES/build \
	&& cmake \
	-DCMAKE_BUILD_TYPE=Release \
	-DQPOASES_BUILD_EXAMPLES=OFF \
	.. \
	&& make install \
	&& cd /opt && rm -r /opt/qpOASES


ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add user info
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${GROUP_NAME} 

RUN useradd --create-home --shell /bin/bash \
	--uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
	# Possible security risk
	&& echo "${USER_NAME}:${GROUP_NAME}" | sudo chpasswd \
	&& echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"

# Make workspace 
RUN mkdir -p /home/${USER_NAME}/${WORKSPACE_NAME}/src/${PROJECT_NAME} \
	&& chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME}
ENV HOME /home/${USER_NAME}
ENV WORKSPACE ${HOME}/${WORKSPACE_NAME}


RUN echo "export USER=${USER_NAME}" >> ${HOME}/.bashrc \
	&& echo "export GROUP=${GROUP_NAME}" >> ${HOME}/.bashrc \
	&& echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${RAISIM_DIR}/raisim/linux/lib" >> ${HOME}/.bashrc \
	&& echo "export PYTHONPATH=${PYTHONPATH}:${RAISIM_DIR}/raisim/linux/lib" >> ${HOME}/.bashrc 
# Shell
USER ${USER_NAME}
WORKDIR ${WORKSPACE}
ENV SHELL "/bin/bash"