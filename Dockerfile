FROM tradr/base_image

RUN cd /tmp && git clone https://github.com/google/protobuf.git && cd /tmp/protobuf && \
    ./autogen.sh && ./configure && make -j 2 && make install && ldconfig 
 
RUN wget -O - http://vps.tradr-project.eu/repo/key.gpg | apt-key add - && echo "deb http://vps.tradr-project.eu/repo/apt/ubuntu trusty tradr-thirdparty" >>  /etc/apt/sources.list.d/tradr.list && apt-get update

RUN apt install -y libvlc-dev libvlccore-dev vlc libssh2-1-dev

RUN touch /root/.ssh/known_hosts

ADD files/.bashrc /home/tradr/

ADD src /home/tradr/tradr/src/tradr_uavs/src

RUN /bin/bash -c "cd /home/tradr/tradr && source /opt/ros/indigo/setup.bash && source devel/setup.bash && catkin_make"

COPY files/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
