FROM shadowrobot/build-tools:trusty-indigo-ide

MAINTAINER "Shadow Robot's Software Team <software@shadowrobot.com>"

LABEL Description="An image generated during the testing of Shadow's release procedures." Version="1.0"

ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$toolset_branch/bin/setup_dev_machine"

ENV rosinstall_repo=shadow_dexterous_hand
ENV rosinstall_repo_branch=indigo-devel

RUN set +x && \
    \
    echo "Getting ssh key from the http server" && \
    mkdir /home/user/.ssh && \
    wget --tries=5 -O /home/user/.ssh/id_rsa http://172.17.0.1:8008/id_rsa && \
    ssh-keyscan github.com >> /home/user/.ssh/known_hosts && \
    chmod 400 /home/user/.ssh/id_rsa && \
    chown -R $MY_USERNAME:$MY_USERNAME /home/user/.ssh && \
    \
    echo "Fixing file system" && \
    mkdir -p /etc/apt/sources.list.d && \
    rm -f /etc/apt/sources.list.d/ros-latest.list && \
    \
    echo "Running one-liner" && \
    wget -O /tmp/oneliner "$( echo "$remote_shell_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \
    gosu $MY_USERNAME /tmp/oneliner -r $rosinstall_repo -b $rosinstall_repo_branch -i repository.rosinstall -w /home/user/projects/shadow_robot/base -v "$ros_release_name" -s true  && \
    \
    echo "Installing production tools" && \
    wget -O /tmp/production_tools https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$(echo $toolset_branch | sed 's/#/%23/g')/bin/install-production-tools.sh && \
    bash /tmp/production_tools -v "$ros_release_name" -b "$toolset_branch"  && \
    \
    echo "Removing cache" && \
    apt-get clean && \
        rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /home/$MY_USERNAME/.ansible /home/$MY_USERNAME/.gitconfig /home/$MY_USERNAME/.ssh/*

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/bin/terminator"]
