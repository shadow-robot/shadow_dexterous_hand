FROM public.ecr.aws/shadowrobot/build-tools:focal-noetic

LABEL Description="This ROS Noetic image contains Shadow's dexterous hand software with build tools. It includes IDE environments." Vendor="Shadow Robot" Version="1.0"

ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh"

ENV PROJECTS_WS=/home/user/projects/shadow_robot
ENV rosinstall_repo=shadow_dexterous_hand
ENV rosinstall_repo_branch=noetic-devel

ENV aurora_branch="master"
ENV aurora_script="https://raw.githubusercontent.com/shadow-robot/aurora/$aurora_branch/bin/run-ansible.sh"

RUN set +x && \
    echo "Running one-liner" && \
    apt-get update && \
    \
    wget -O /tmp/oneliner "$( echo "$remote_shell_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \ 
    gosu $MY_USERNAME /tmp/oneliner -w $PROJECTS_WS/base -r $rosinstall_repo -b $rosinstall_repo_branch -i repository.rosinstall -v "noetic" -s false -t pyqtgraph && \
    \
    echo "Installing AWS CLI, libglvnd, vscode and warehouse_ros" && \
    wget -O /tmp/aurora "$( echo "$aurora_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/aurora && \
    gosu $MY_USERNAME /tmp/aurora install_software --branch $aurora_branch software=[production_tools,aws-cli,libglvnd,vscode,warehouse_ros] && \
    \
    echo "Removing cache" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /home/$MY_USERNAME/.ansible /home/$MY_USERNAME/.gitconfig /root/.cache

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/bin/terminator"]
