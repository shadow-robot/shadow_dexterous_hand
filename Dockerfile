FROM shadowrobot/build-tools:xenial-kinetic-ide

LABEL Description="This ROS Kinetic image contains Shadow's dexterous hand software with build tools. It includes IDE environments." Vendor="Shadow Robot" Version="1.0"
ARG toolset_branch="B_parent_image_toivo"
ENV toolset_branch=${toolset_branch}

ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$toolset_branch/ansible/deploy.sh"

ENV PROJECTS_WS=/home/user/projects/shadow_robot

RUN set +x && \
    \
    echo "Running one-liner" && \
    apt-get update && \
    wget -O /tmp/oneliner "$( echo "$remote_shell_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \
    gosu $MY_USERNAME /tmp/oneliner -w $PROJECTS_WS/base -r sr-build-tools -b $toolset_branch -i data/shadow_robot-kinetic.rosinstall -v "kinetic" -t mongodb,pyassimp,pyqtgraph && \
    \
    echo "Installing production tools" && \
    wget -O /tmp/production_tools https://raw.githubusercontent.com/shadow-robot/sr-build-tools/$(echo $toolset_branch | sed 's/#/%23/g')/bin/install-production-tools.sh && \
    bash /tmp/production_tools -v "$ros_release_name" -b "$toolset_branch"  && \
    \
    echo "Installing AWS CLI" && \
    curl "https://s3.amazonaws.com/aws-cli/awscli-bundle.zip" -o "/tmp/awscli-bundle.zip" && \
    unzip -o /tmp/awscli-bundle.zip -d /tmp/ && \
    /tmp/awscli-bundle/install -b /usr/local/bin/aws && \
    \
    echo "Removing cache" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /home/$MY_USERNAME/.ansible /home/$MY_USERNAME/.gitconfig

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/bin/terminator"]
