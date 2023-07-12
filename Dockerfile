FROM public.ecr.aws/shadowrobot/build-tools:focal-noetic

LABEL Description="This ROS Noetic image contains Shadow's dexterous hand software with build tools. It includes IDE environments." Vendor="Shadow Robot" Version="1.0"

ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh"

ENV PROJECTS_WS=/home/user/projects/shadow_robot
ENV rosinstall_repo=shadow_dexterous_hand
ENV rosinstall_repo_branch=noetic-devel
ARG AWS_CONTAINER_CREDENTIALS_RELATIVE_URI=$AWS_CONTAINER_CREDENTIALS_RELATIVE_URI
ARG AWS_DEFAULT_REGION=$AWS_DEFAULT_REGION

ENV aurora_branch="master"
ENV aurora_script="https://raw.githubusercontent.com/shadow-robot/aurora/$aurora_branch/bin/run-ansible.sh"

RUN set +x && \
    echo "Running one-liner" && \
    apt-get update && \
    \
    echo "toms test stuff" && \
    \
    echo 'Acquire::http::Proxy "http://ec2-18-132-143-60.eu-west-2.compute.amazonaws.com:3142";' | tee /etc/apt/apt.conf.d/00aptproxy && \
    \
    echo "aws pre install" && \
    wget -O /tmp/aurora "$( echo "$aurora_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/aurora && \
    gosu $MY_USERNAME /tmp/aurora install_software --branch $aurora_branch software=[aws-cli] && \
    \
    echo "s3 copying prebuilt bins"

RUN set +x && \
    mkdir -p $PROJECTS_WS/base/build && \
    mkdir -p $PROJECTS_WS/base/devel

RUN set +x && \
    touch /tmp/AWS_CRED && \
    echo $AWS_CONTAINER_CREDENTIALS_RELATIVE_URI >> /tmp/AWS_CRED && \
    echo $AWS_DEFAULT_REGION >> /tmp/AWS_CRED && \
    source /tmp/AWS_CRED && \
    gosu $MY_USERNAME aws s3 sync s3://backup-build-binaries/build/ $PROJECTS_WS/base/build && \
    gosu $MY_USERNAME aws s3 sync s3://backup-build-binaries/devel/ $PROJECTS_WS/base/devel

RUN set +x && \
    wget -O /tmp/oneliner "$( echo "$remote_shell_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \ 
    gosu $MY_USERNAME /tmp/oneliner -w $PROJECTS_WS/base -r $rosinstall_repo -b $rosinstall_repo_branch -i repository.rosinstall -v "noetic" -s false -t pyqtgraph && \
    \
    echo "Installing AWS CLI, libglvnd, vscode and warehouse_ros" && \
    gosu $MY_USERNAME /tmp/aurora install_software --branch $aurora_branch software=[production_tools,libglvnd,vscode,warehouse_ros] && \
    \
    echo "Removing cache" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /home/$MY_USERNAME/.ansible /home/$MY_USERNAME/.gitconfig /root/.cache

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/bin/terminator"]
