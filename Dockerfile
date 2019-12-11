FROM shadowrobot/build-tools:bionic-melodic-ide

LABEL Description="This ROS Melodic image contains Shadow's dexterous hand software with build tools. It includes IDE environments." Vendor="Shadow Robot" Version="1.0"

ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$toolset_branch/ansible/deploy.sh"

ENV PROJECTS_WS=/home/user/projects/shadow_robot

ENV aurora_branch="master"
ENV aurora_script="https://raw.githubusercontent.com/shadow-robot/aurora/$aurora_branch/bin/run-ansible.sh"

RUN set +x && \
    \
    echo "Running one-liner" && \
    apt-get update && \
    wget -O /tmp/oneliner "$( echo "$remote_shell_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \
    gosu $MY_USERNAME /tmp/oneliner -w $PROJECTS_WS/base -r sr-build-tools -b $toolset_branch -i data/shadow_robot-melodic.rosinstall -v "melodic" -t mongodb,pyassimp,pyqtgraph && \
    \
    echo "Installing production tools" && \
    wget -O /tmp/production_tools https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$(echo $toolset_branch | sed 's/#/%23/g')/bin/install-production-tools.sh && \
    bash /tmp/production_tools -v "$ros_release_name" -b "$toolset_branch"  && \
    \
    echo "Installing AWS CLI and libglvnd" && \
    wget -O /tmp/aurora "$( echo "$aurora_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/aurora && \
    gosu $MY_USERNAME /tmp/aurora install_software --debug-branch $aurora_branch software=[aws-cli,libglvnd] && \
    \
    echo "Removing cache" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /home/$MY_USERNAME/.ansible /home/$MY_USERNAME/.gitconfig

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/bin/terminator"]
