FROM ubuntu:20.04

LABEL maintainer="Nicolas Perrault <nipe1783@colorado.edu>"

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    wget \
    bzip2 \
    ca-certificates \
    sudo \
    locales \
    fonts-liberation \
    lsb-release \
    tzdata \
    git \
    cmake \
    g++ \
    libboost-all-dev \
    python3 \
    python3-pip \
    libeigen3-dev \
    libyaml-cpp-dev \
    build-essential && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN ln -fs /usr/share/zoneinfo/America/Chicago /etc/localtime && \
    DEBIAN_FRONTEND=noninteractive dpkg-reconfigure --frontend noninteractive tzdata

RUN git clone --branch 1.6.0 https://github.com/ompl/ompl.git /ompl && \
    cd /ompl && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j $(nproc) && \
    make install

ENV CONDA_DIR=/opt/conda \
    SHELL=/bin/bash \
    ENV_USER=ompl \
    ENV_UID=1000 \
    ENV_GID=100 \
    LC_ALL=en_US.UTF-8 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV PATH=$CONDA_DIR/bin:$PATH \
    HOME=/home/$ENV_USER

RUN echo "${LANG} UTF-8" > /etc/locale.gen && \
    locale-gen

RUN useradd -m -s /bin/bash -N -u $ENV_UID $ENV_USER && \
    mkdir -p $CONDA_DIR && \
    chown $ENV_USER:$ENV_GID $CONDA_DIR && \
    chmod g+w /etc/passwd /etc/group && \
    mkdir /home/$ENV_USER/gbt && \
    chown -R $ENV_USER:$ENV_GID /home/$ENV_USER

USER $ENV_UID

RUN echo 'export PYTHONPATH=$(python3 -c "import sys; print(\":\".join(sys.path))"):/usr/local/lib/python3/dist-packages' >> $HOME/.bashrc

WORKDIR /home/$ENV_USER/gbt
COPY . /home/$ENV_USER/gbt

USER root
RUN chown -R $ENV_UID:$ENV_GID /home/$ENV_USER/gbt && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make
RUN chown -R $ENV_UID:$ENV_GID /home/$ENV_USER/gbt
USER $ENV_UID

CMD ["/bin/bash"]
